# RoadTailBench 指标说明 v4

本文档对应当前 `leaderboard-eval` 的离线指标实现，输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`。

## 设计原则

RoadTailBench 保留 Bench2Drive 式“路线完成 + 安全惩罚”骨架，但代码场景还需要评价速度适配、近距风险、隐患响应、舒适度、控制稳定性、能耗和能力标签。能力标签不再用 A/B/C 大类自由描述，而是从固定候选表写 0/1。

公开资料上，FHWA surrogate safety measures 和 SSAM 常用 TTC、PET 等替代安全指标分析冲突；横向安全可用 time-to-line/lateral-conflict 类时间指标补充 TTC。舒适度参考 ISO 2631-1 的轴向/RMS 思路，并结合自动驾驶常用的加速度、jerk、yaw rate 阈值。能耗采用车辆纵向动力学中的滚阻、风阻、坡度、加速功率和再生制动估算。

## 核心指标

- `route_completion`：优先把 ego 位置投影到 `reference_trajectory`。如果 ego 到达 `ego_end`/`ego_goal` 容差范围，即使未严格贴合参考轨迹，也计为完成。缺少参考轨迹但有起终点时使用起终点距离缩短比例兜底；参考轨迹和终点都缺失时返回 0。
- `collision_penalty`：通过 CARLA `sensor.other.collision` 记录碰撞，按对象类型加权扣分。默认 walker=1.0、vehicle=0.75、static=0.35、prop=0.25、other=0.5。
- `driving_efficiency`：使用仿真时间 `frame.time`，不使用 wall-clock。
- `speed_appropriateness`：普通区域按 `speed_limit_kmh` 扣超速；hazard 半径内按 `hazards[].reference_speed_kmh` 扣分。总分为逐帧均值。
- `trajectory_adherence`：评价 ego 相对合理参考轨迹的空间、进度/时间、航向偏差。没有 `reference_trajectory` 时返回 0。
- `proximity_risk`：融合 actor 距离、环境 raycast clearance、纵向 TTC、横向距离、横向冲突时间和危险帧比例。
- `comfort`：在 ego 车体坐标系下计算纵/横向加速度、纵/横向 jerk、yaw rate 的 RMS 与峰值。
- `control_stability`：评价 steer、throttle、brake 相邻帧变化。
- `energy_efficiency`：默认 ego 为轿车，用纵向动力学估算净能耗。
- `long_tail_hazard_response`：进入 hazard 感知半径后，评价减速、制动、转向或油门释放响应。

## 近距风险

`proximity_risk` 不再只用固定距离阈值：

- actor 纵向 danger 距离：`max(3 m, ego_speed_mps * 0.7 s)`。
- actor 纵向 caution 距离：`max(12 m, ego_speed_mps * 1.5 s)`。
- TTC 阈值：danger 2 s，caution 5 s，只对前向闭合风险生效。
- 横向冲突时间：`TLC-like = max(0, abs(lateral) - lateral_danger) / lateral_closing_speed`，danger 1 s，caution 3 s。
- 环境 raycast clearance 阈值较小：danger 0.75 m，caution 2.5 m，避免把正常路边护栏长期当成高风险。

raycast 默认每 5 帧采样 7 条射线 `[-90,-60,-30,0,30,60,90]`，最大距离 30 m。射线从 ego 包围盒外侧发出，并忽略贴身命中。绘图层会对缺失检测填充到上限或阈值以保证曲线连续，但 metrics details 保留原始检测语义。

## 能力评分

能力候选表在 `metadata/capability_taxonomy.json`。metadata 中只允许在 `capability_vector.behavior` 和 `capability_vector.hazard` 写 0/1，未选能力输出 `null` 且不参与均值。

`behavior_capability_score` 评价车辆动作能力，例如 lane change、overtaking、bypass obstacle、yielding、merge/cut-in、pedestrian interaction、emergency braking：

```text
0.35 route_completion
+ 0.25 collision_penalty
+ 0.20 proximity_risk
+ 0.10 speed_appropriateness
+ 0.10 control_stability
```

`hazard_capability_score` 评价隐患/环境应对能力，例如 limited sight distance、low friction、falling obstacle、construction/lane blockage、priority conflict、adverse weather/lighting：

```text
0.25 route_completion
+ 0.20 collision_penalty
+ 0.20 speed_appropriateness
+ 0.25 long_tail_hazard_response
+ 0.10 trajectory_adherence
```

`ability_score` 仅作为兼容聚合：行为和隐患都存在时取二者均值；只有一类存在时取该类。旧 `scenario_tags` / `ability_tags` 不再作为主评分依据。

## 总分

`leaderboard_driving_score` 是 0-100 分：

```text
task_gate = 0.70 * route_completion + 0.30 * trajectory_adherence
safety_gate = 0.65 * collision_penalty + 0.35 * proximity_risk
```

之后乘以效率、速度、舒适度、稳定性、能耗和隐患响应修正项。路线未完成或安全差时总分会明显受限。

## Metadata 要求

每个场景至少应提供：

- `ego_start` 和 `ego_end`：用于自然结束和路线完成兜底。
- `reference_trajectory`：用于路线投影和轨迹贴合。
- `speed_limit_kmh`：普通区域速度上限。
- `hazards[].center`、`radius_m`、`perception_radius_m`、`reference_speed_kmh`：用于速度适配和隐患响应。
- `capability_vector.behavior`、`capability_vector.hazard`：固定候选表 0/1 能力向量。
