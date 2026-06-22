# RoadTailBench 指标说明 v3

本文档对应当前 `leaderboard-eval` 的离线指标实现，输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`。

## 设计原则

RoadTailBench 保留 Bench2Drive 式“路线完成 + 安全惩罚”的骨架，但场景是代码驱动的长尾隐患场景，所以额外评价速度适配、近距风险、隐患响应、舒适度、控制稳定性、能耗和 A/B/C 能力标签。

公开资料中，乘坐舒适度常用车体坐标系加速度、jerk、RMS 和阈值法；ISO 2631-1 也以全身振动方向、幅值、频率和 RMS 等方法为核心。能耗估计采用车辆纵向动力学的滚阻、风阻、坡度阻力、加速功率和再生制动回收。

## 核心指标

- `route_completion`：优先把 ego 位置投影到 `reference_trajectory`，取最大进度除以路线长度。如果 ego 到达 `ego_end`/`ego_goal` 容差范围内，即使没有严格贴合参考轨迹，也计为完成。缺少参考轨迹但有 `ego_start` 和 `ego_end` 时，使用起终点距离缩短比例兜底。参考轨迹和终点都缺失时返回 `0.0`，避免 metadata 漏填导致虚高。
- `collision_penalty`：通过 CARLA `sensor.other.collision` 记录碰撞，按对象类型加权扣分。默认 walker=1.0、vehicle=0.75、static=0.35、prop=0.25、other=0.5。CARLA 碰撞事件会返回 other actor、碰撞位置和 normal impulse；纯地图几何不一定有清晰语义类型，因此由 unknown/环境类碰撞和 raycast 近距风险共同补充。
- `driving_efficiency`：使用仿真时间 `frame.time` 计算从开始到结束的耗时效率，不使用 wall-clock，避免设备性能影响。
- `speed_appropriateness`：普通区域按 `speed_limit_kmh` 和容忍值扣超速；进入 hazard 半径后，按该 hazard 的 `reference_speed_kmh` 和 `hazard_speed_tolerance_ratio` 扣分。总分是所有帧分数的均值，details 记录 hazard 帧比例和均值。
- `trajectory_adherence`：评价 ego 相对合理参考轨迹的时空偏差，不代表道路 polygon 可行域。没有 `reference_trajectory` 时返回 `0.0`。
- `proximity_risk`：融合最近 actor 距离、raycast 环境 clearance、纵向 TTC、横向距离和危险帧比例。TTC 只在 actor 位于 ego 前方且横向接近时生效；环境 raycast 使用更小的 clearance 阈值，避免把正常路边护栏/墙体长期计为高风险。
- `comfort`：在 ego 车体坐标系下分解纵向/横向加速度，计算纵向/横向 jerk 和 yaw rate 的 RMS 与峰值，再按权重聚合。默认权重：纵向加速度 25%、横向加速度 25%、纵向 jerk 20%、横向 jerk 20%、yaw rate 10%。
- `control_stability`：评价 steer、throttle、brake 相邻帧变化，避免只看转向。
- `energy_efficiency`：默认 ego 为轿车，用质量、Cd、迎风面积、滚阻、空气密度、传动效率和再生制动效率估算净能耗，输出 `estimated_energy_kwh`、`traction_energy_kwh`、`regenerated_energy_kwh`、`energy_per_100km_kwh`。
- `long_tail_hazard_response`：进入 hazard 感知半径后，评价是否出现足够的减速、制动、转向或油门释放响应。速度是否达标交给 `speed_appropriateness`。

`road_engineering_hazard_adaptation` 已移除，避免与其它指标重复解释。

## 轨迹贴合度

`trajectory_adherence` 使用 `reference_trajectory`，格式为 `[[x, y, yaw], ...]` 或 `[[x, y], ...]`。参考轨迹只代表“合理轨迹”，不是专家最优轨迹，因此阈值较宽：

- 横向偏差：允许 4 m，硬惩罚 12 m。
- 进度/时间偏差：允许 20 m 或 3 s，硬惩罚 60 m 或 8 s。
- 航向偏差：有 yaw 时参与，允许 45 度，硬惩罚 120 度。
- 聚合权重：横向 45%，进度/时间 35%，航向 20%。

details 输出 `mean/max_lateral_deviation_m`、`mean/max_progress_error_m`、`mean/max_heading_error_deg`、`final_progress_m`。

## Raycast 性能

环境近距风险不每帧 raycast。默认 `environment_raycast_interval_frames=5`，只发 7 条射线：`[-90, -60, -30, 0, 30, 60, 90]`，最大距离默认 30 m。非采样帧复用上一次结果，并在 frame log 中标记 `raycast_reused=true`。这能显著降低 CARLA RPC 压力。

raycast 从 ego 包围盒外侧发出，并忽略贴身命中，避免射线首先打到 ego 自己。环境 clearance 默认危险阈值 0.75 m、注意阈值 2.5 m；actor 距离仍使用 3 m/12 m 的危险/注意阈值。

## 总分和能力分

`leaderboard_driving_score` 是 0-100 分：

- `task_gate = 0.70 * route_completion + 0.30 * trajectory_adherence`
- `safety_gate = 0.65 * collision_penalty + 0.35 * proximity_risk`

再乘以效率、速度、舒适度、稳定性、能耗和隐患响应修正项。路线未完成或安全很差时，总分会明显受限；轻微静态碰撞不会像 Bench2Drive 一样直接极端归零。

`ability_score` 只按 A/B/C 大类聚合：

- A：道路/环境隐患适应，侧重路线完成、轨迹贴合、速度适配和隐患响应。
- B：交通参与者交互，侧重路线完成、碰撞、近距风险和隐患响应。
- C：天气/环境鲁棒与控制质量，侧重速度、舒适度、控制稳定性和能耗。

## Metadata 要求

每个场景至少应提供：

- `ego_start` 和 `ego_end`：用于自然结束和路线完成兜底。
- `reference_trajectory`：用于 `route_completion` 精确投影和 `trajectory_adherence`。如果实际路径可能不严格按参考轨迹走，仍可以给一条“合理参考轨迹”，route completion 会允许到达终点即完成。
- `speed_limit_kmh`：普通区域速度上限。
- `hazards[].center`、`hazards[].radius_m`、`hazards[].perception_radius_m`、`hazards[].reference_speed_kmh`：用于速度适配和隐患响应。

缺少 `reference_trajectory` 不再给满分；缺少终点也无法可靠判断完成度。
