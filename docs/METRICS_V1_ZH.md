# RoadTailBench 指标说明 v5

本文档对应当前 `leaderboard-eval` 的离线指标实现。输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`，每个指标输出 `[0, 1]` 分数；综合驾驶分 `leaderboard_driving_score` 输出 0-100 分。

## 核心指标

- `route_completion`：优先把 ego 位置投影到 `reference_trajectory` 计算最大路线进度；如果 ego 到达 `ego_end`/`ego_goal` 容差范围，则直接计为完成。缺少参考轨迹但有起终点时使用起终点距离缩短比例兜底；轨迹和终点都缺失时返回 0。
- `collision_penalty`：通过 CARLA `sensor.other.collision` 记录碰撞，按对象类型加权扣分。默认权重为 walker=1.0、vehicle=0.75、static=0.35、prop=0.25、other=0.5。
- `driving_efficiency`：使用仿真时间 `frame.time` 评价完成路线耗时，不使用 wall-clock。期望时间由路线长度和 `speed_limit_kmh` / `reference_speed_kmh` 得到。
- `speed_appropriateness`：普通区域按 `speed_limit_kmh` 扣超速；进入 `hazards[].radius_m` 内时按 `hazards[].reference_speed_kmh` 扣过快。总分为逐帧均值。
- `trajectory_adherence`：评价 ego 相对合理参考轨迹的横向偏差、进度/时间偏差和航向偏差。缺少 `reference_trajectory` 时返回 0。
- `proximity_risk`：近距安全裕度指标。统一融合动态 actor 和环境 raycast 命中，不再使用传统 TTC/TLC 名义，而是计算纵向安全时距、横向安全时距、横向净距和最近距离裕度。
- `comfort`：在 ego 车体坐标系下计算纵/横向加速度、纵/横向 jerk、yaw rate 的 RMS 与峰值。
- `control_stability`：评价 steer、throttle、brake 相邻帧变化是否平滑。
- `energy_efficiency`：默认 ego 为轿车，用纵向动力学估算净能耗和 kWh/100 km。
- `long_tail_hazard_response`：进入 hazard 感知半径后，评价减速、制动、转向或油门释放响应。已废弃危险区进入惩罚；`hazards[].radius_m` 只表示隐患影响范围，不表示禁止进入区。

## 近距安全裕度

`proximity_risk` 对每帧收集候选对象：

- 动态 actor：车辆、行人、静态 actor 等 frame log 中记录的对象。
- 环境命中：`environment_hits` 中的 raycast 命中，包括相对角度、距离和命中位置。

动态距离阈值随 ego 速度变化：

```text
d_danger(v) = max(3 m, v * 0.7 s)
d_safe(v)   = max(12 m, v * 1.5 s)
q_dist      = clamp((d_min - d_danger) / (d_safe - d_danger))
```

纵向安全时距：

```text
T_long = |d_long| / max(v_closing_long, |v_ego_long|, v_min)
q_long = clamp((T_long - T_long,danger) / (T_long,safe - T_long,danger))
```

横向安全时距与横向净距：

```text
T_lat      = |d_lat| / max(v_closing_lat, |v_ego_lat|, v_min)
q_lat_time = clamp((T_lat - T_lat,danger) / (T_lat,safe - T_lat,danger))
q_lat_dist = clamp((|d_lat| - d_lat,danger) / (d_lat,safe - d_lat,danger))
```

每帧取可用子项的最小值：

```text
q_k = min(q_dist, q_long, q_lat_time, q_lat_dist, q_env)
score = clamp(mean(q_k) * (1 - 0.5 * danger_frame_ratio))
```

默认参数：

- 纵向安全时距 danger/safe：0.7 s / 1.5 s。
- 横向安全时距 danger/safe：0.7 s / 2.0 s。
- 横向净距 danger/safe：1.0 m / 3.0 m。
- 环境 clearance danger/safe：0.75 m / 2.5 m。
- 环境 raycast 最小有效命中距离：1.0 m；小于该值的命中会被视为自车、地面或贴身碰撞体伪命中并忽略。
- 小速度保护 `safety_margin_min_speed_mps`：0.5 m/s。

绘图中安全时距会截断到 8 s 显示，避免空旷场景导致 y 轴过大；JSON details 保留原始最小值。

## 长尾隐患响应

每个 hazard 至少建议包含：

```json
{
  "id": "hazard_1",
  "center": [x, y, z],
  "radius_m": 5.0,
  "perception_radius_m": 20.0,
  "reference_speed_kmh": 20.0
}
```

`perception_radius_m` 缺失时使用 `radius_m + 15 m`。进入感知半径后，满足以下任一响应条件并连续达到 `response_min_consecutive_frames` 帧，即记录首次响应时间：

```text
brake >= 0.20
|steer| >= 0.25
throttle <= 0.05
single-frame speed drop >= 0.4 m/s
cumulative speed drop from hazard entry >= 1.5 m/s
```

单个 hazard 得分：

```text
q_h = exp(-(t_response - t_enter) / response_tau_s)
```

默认 `response_tau_s=2.0`。若无响应则为 0；若发生碰撞则乘以 0.25。危险区字段和进入危险区乘子已废弃，不再参与计算。

## 能力评分

能力候选表位于 `metadata/capability_taxonomy.json`。metadata 中只应在 `capability_vector.behavior` 和 `capability_vector.hazard` 写 0/1；未选能力输出 `null` 且不参与均值。

`behavior_capability_score`：

```text
0.35 route_completion
+ 0.25 collision_penalty
+ 0.20 proximity_risk
+ 0.10 speed_appropriateness
+ 0.10 control_stability
```

`hazard_capability_score`：

```text
0.25 route_completion
+ 0.20 collision_penalty
+ 0.20 speed_appropriateness
+ 0.25 long_tail_hazard_response
+ 0.10 trajectory_adherence
```

`ability_score` 仅作兼容聚合：行为和隐患都存在时取两者均值；只有一类存在时取该类；都没有时为 0。

## 综合驾驶分

```text
task_gate   = 0.70 * route_completion + 0.30 * trajectory_adherence
safety_gate = 0.65 * collision_penalty + 0.35 * proximity_risk
```

之后乘以效率、速度、舒适性、稳定性、能耗和隐患响应修正项。路线未完成或安全表现差时，总分会被明显压低。

## 输出文件

每个 run 目录会保存：

```text
leaderboard_frame_log.jsonl
leaderboard_scenario_config.json
leaderboard_metrics.json
leaderboard_metrics.csv
leaderboard_run_summary.json
```

`leaderboard_metrics.csv` 是 `leaderboard_metrics.json` 的扁平化版本，列为 `scenario_id, route_id, metric_name, score, details_json`，便于 Excel、Origin 或 Python/pandas 汇总。
