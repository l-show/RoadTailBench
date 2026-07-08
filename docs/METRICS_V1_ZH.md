# RoadTailBench 指标说明 v7

本文档对应当前 `leaderboard-eval` 的离线指标实现。输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`，每个核心指标输出 `[0, 1]` 分数，综合驾驶分 `leaderboard_driving_score` 输出 0-100 分。

## 核心指标

- `route_completion`: 路线完成率。优先使用 `reference_trajectory` 投影计算最大路线进度；如果 metadata 同时提供 `ego_start` 和 `ego_end`/`ego_goal`，会用起终点距离缩短比例作为兜底，并取两者较大值。若参考路线和终点都缺失，返回 0。
- `trajectory_adherence`: 轨迹贴合度。默认模式为 `spatial_reference_deviation`，只评价 ego 相对 `reference_trajectory` 的横向偏差和航向偏差，不按时间或参考速度惩罚进度快慢。缺少 `reference_trajectory` 时返回 0。
- `collision_penalty`: 碰撞惩罚。通过 CARLA `sensor.other.collision` 记录碰撞，按对象类型加权扣分。
- `driving_efficiency`: 驾驶效率。使用仿真时间 `frame.time` 评价完成路线耗时；期望时间由路线长度和 `speed_limit_kmh` / `reference_speed_kmh` 得到。
- `speed_appropriateness`: 速度适配度。普通区域按 `speed_limit_kmh` 扣超速；进入 `hazards[].radius_m` 内时按 `hazards[].reference_speed_kmh` 扣过快。
- `proximity_risk`: 近距安全裕度。由 actor 纵向安全、actor 横向安全、环境 raycast 距离安全三项加权组成。
- `comfort`: 舒适性。计算车体坐标系下纵/横向加速度、纵/横向 jerk、yaw rate 的 RMS 与峰值。
- `control_stability`: 控制稳定性。评价 steer、throttle、brake 相邻帧变化是否平滑。
- `energy_efficiency`: 能耗效率。默认 ego 为轿车，用纵向动力学估算净能耗和 kWh/100 km。
- `long_tail_hazard_response`: 长尾隐患响应。进入 hazard 感知半径后，评价减速、制动、转向或松油门响应。

## 轨迹贴合模式

默认配置：

```json
{
  "trajectory_adherence_mode": "spatial",
  "reference_trajectory_format": "x_y_yaw",
  "reference_trajectory": [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]]
}
```

空间模式评分：

```text
score = 0.75 * score_lateral_deviation
      + 0.25 * score_heading_deviation
```

该模式不需要时空专家轨迹，也不要求记录每个时刻的位置。速度快慢由速度、效率、舒适性和控制稳定性指标承担。

如需复用旧版进度/时间偏差评分，可以显式设置：

```json
{"trajectory_adherence_mode": "spatiotemporal"}
```

时空模式会额外使用 `reference_speed_kmh` 推算期望进度，并输出 `max_progress_error_m`、`mean_progress_error_m` 等 details。

## 综合分

`behavior_capability_score` 现在按 metadata 里的 `capability_vector.ego_action` 选择动作维度：

```json
{
  "names": ["Overtaking", "Following", "Yielding", "Merging", "Crossing", "Braking", "Keeping"],
  "values": [0, 1, 0, 0, 0, 0, 1]
}
```

```text
0.35 route_completion
+ 0.25 collision_penalty
+ 0.20 proximity_risk
+ 0.10 speed_appropriateness
+ 0.10 control_stability
```

`hazard_capability_score` 现在按 metadata 里的 `capability_vector.hazard_type` 选择隐患类型维度：

```json
{
  "names": ["traffic_signs_markings", "separation_protection", "speed_control_facilities", "lighting_facilities", "road_intersection", "road_surface_condition", "road_alignment", "limited_sight_distance", "clearance_intrusion", "adverse_weather"],
  "values": [0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
}
```

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

之后再乘以效率、速度、舒适性、稳定性、能耗和隐患响应修正项。

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
