# RoadTailBench 指标说明 v6

本文档对应当前 `leaderboard-eval` 的离线指标实现。输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`，每个核心指标输出 `[0, 1]` 分数，综合驾驶分 `leaderboard_driving_score` 输出 0-100 分。

## 核心指标

- `route_completion`：路线完成率。优先使用 `reference_trajectory` 投影计算最大路线进度；如果 metadata 同时提供 `ego_start` 和 `ego_end`/`ego_goal`，会用起终点距离缩短比例作为兜底，并取两者较大值。这样参考轨迹不完整或实际轨迹不严格贴合参考线时，不会直接把完成率压成 0。若参考轨迹和终点都缺失，则返回 0。
- `collision_penalty`：碰撞惩罚。通过 CARLA `sensor.other.collision` 记录碰撞，按对象类型加权扣分。默认权重为 walker=1.0、vehicle=0.75、static=0.35、prop=0.25、other=0.5。
- `driving_efficiency`：驾驶效率。使用仿真时间 `frame.time` 评价完成路线耗时，不使用 wall-clock。期望时间由路线长度和 `speed_limit_kmh` / `reference_speed_kmh` 得到。
- `speed_appropriateness`：速度适配度。普通区域按 `speed_limit_kmh` 扣超速；进入 `hazards[].radius_m` 内时按 `hazards[].reference_speed_kmh` 扣过快。
- `trajectory_adherence`：轨迹贴合度。评价 ego 相对合理参考轨迹的横向偏差、进度/时间偏差和航向偏差。缺少 `reference_trajectory` 时返回 0。
- `proximity_risk`：近距安全裕度。由三个清晰子项加权组成：actor 纵向安全、actor 横向安全、环境 raycast 全向距离安全。
- `comfort`：舒适性。计算车体坐标系下纵/横向加速度、纵/横向 jerk、yaw rate 的 RMS 与峰值。
- `control_stability`：控制稳定性。评价 steer、throttle、brake 相邻帧变化是否平滑。
- `energy_efficiency`：能耗效率。默认 ego 为轿车，用纵向动力学估算净能耗和 kWh/100 km。
- `long_tail_hazard_response`：长尾隐患响应。进入 hazard 感知半径后，评价减速、制动、转向或松油门响应。危险区进入惩罚已废弃；`hazards[].radius_m` 只表示隐患影响范围。

## 近距安全裕度

`proximity_risk` 不再把 actor 和 raycast 混成一个复杂 min 表达式，而是三项加权：

```text
score_proximity =
  w_long * score_actor_longitudinal
+ w_lat  * score_actor_lateral
+ w_env  * score_environment_clearance
```

默认权重：

```text
w_long = 0.40
w_lat  = 0.35
w_env  = 0.25
```

### Actor 纵向安全

纵向部分只看动态 actor。对 ego 坐标系前方 actor 计算 TTC：

```text
TTC = d_long / closing_speed_long, if d_long > 0 and closing_speed_long > 0
```

同时保留纵向距离裕度：

```text
d_safe_long = max(actor_longitudinal_distance_danger_m, v_ego * actor_longitudinal_headway_safe_s)
q_dist_long = clamp((|d_long| - d_danger_long) / (d_safe_long - d_danger_long))
q_ttc = clamp((TTC - TTC_danger) / (TTC_safe - TTC_danger))
score_actor_longitudinal = min(q_ttc, q_dist_long)
```

默认：`TTC_danger=1.0 s`，`TTC_safe=3.0 s`，`d_danger_long=3.0 m`，安全车头时距 `1.5 s`。

### Actor 横向安全

横向部分只看动态 actor。优先使用 frame log 中 ego waypoint 的 `lane_width_m`，由 CARLA OpenDRIVE waypoint 提供；若不可用，默认车道宽度为 `3.5 m`。横向 TLC 使用横向净距与横向闭合速度：

```text
lateral_clearance = max(0, |d_lat| - d_lat_danger)
TLC = lateral_clearance / closing_speed_lat
q_tlc = clamp((TLC - TLC_danger) / (TLC_safe - TLC_danger))
q_lat_dist = clamp((|d_lat| - d_lat_danger) / (0.5 * lane_width - d_lat_danger))
score_actor_lateral = min(q_tlc, q_lat_dist)
```

默认：`TLC_danger=1.0 s`，`TLC_safe=3.0 s`，`d_lat_danger=1.0 m`，默认车道宽度 `3.5 m`。

### 环境全向距离安全

环境部分只看 `environment_hits` 的 raycast 命中距离，不转换成时距：

```text
d_env = min(valid raycast hit distances)
score_environment_clearance = clamp((d_env - d_env_danger) / (d_env_safe - d_env_danger))
```

默认：`d_env_danger=0.75 m`，`d_env_safe=3.0 m`。小于 `environment_raycast_min_hit_distance_m=1.0 m` 的命中会被忽略，用于过滤自车、地面或贴身碰撞体伪命中。没有有效 raycast 命中时，该环境子项记为安全 1.0，并在 details 中通过 `sensor_range_censored_ratio` 记录。

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

默认 `response_tau_s=2.0`。若无响应则为 0；若发生碰撞则乘以 0.25。

## 能力评分和综合分

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

综合驾驶分：

```text
task_gate   = 0.70 * route_completion + 0.30 * trajectory_adherence
safety_gate = 0.65 * collision_penalty + 0.35 * proximity_risk
```

之后乘以效率、速度、舒适性、稳定性、能耗和隐患响应修正项。

## 输出文件

每个 run 目录保存：

```text
leaderboard_frame_log.jsonl
leaderboard_scenario_config.json
leaderboard_metrics.json
leaderboard_metrics.csv
leaderboard_run_summary.json
```

`leaderboard_metrics.csv` 是 `leaderboard_metrics.json` 的扁平化版本，列为 `scenario_id, route_id, metric_name, score, details_json`。
