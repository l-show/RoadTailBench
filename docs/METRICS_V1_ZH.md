# RoadTailBench 指标说明 v2

本文档对应当前 `leaderboard-eval` 的离线指标实现，输入为 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`。

## 与 Bench2Drive 的主要差异

Bench2Drive 的 leaderboard 评分核心是 `score_composed = score_route * score_penalty`。`score_route` 来自 `RouteCompletionTest`，`score_penalty` 由 infractions 乘法扣分得到，包含行人/车辆/静态碰撞、红灯、停牌、场景超时、应急车辆让行、车道外行驶、路线偏离、车辆阻塞等。对应实现可见 `G:\Bench2Drive\leaderboard\leaderboard\utils\statistics_manager.py` 和 `G:\Bench2Drive\scenario_runner\srunner\scenarios\route_scenario.py`。

RoadTailBench 保留路线完成和安全扣分，但不完全照搬 Bench2Drive。原因是这里的场景是代码场景和长尾隐患场景，除了“完成路线且少犯规”，还需要评估隐患响应、交互风险、速度适配、舒适性、控制稳定和 A/B/C 能力标签。

## 10 类核心指标

- `route_completion`：把每一帧 ego 位置投影到 `reference_trajectory`，取最大进度除以参考轨迹总长。缺少参考轨迹时返回 1.0，并在 details 里标注 `missing_route`，避免没有固定路线的场景被错误扣分。
- `collision_penalty`：合并短时间重复碰撞后按对象类型加权扣分。默认 walker=1.0、vehicle=0.75、static=0.35、prop=0.25、other=0.5。静态物和道具碰撞不会直接归零，但仍会降低安全分。
- `driving_efficiency`：逐帧计算 `ego_speed / reference_speed` 并 clamp 到 `[0,1]` 后取均值。
- `speed_appropriateness`：逐帧比较 ego 速度与全局 `reference_speed_kmh` 或局部 `speed_zones` 目标速度，偏差越大分数越低。
- `drivable_area`：名称保留兼容，但语义改为“预设路线时空偏差”。它不再是道路 polygon 可行域，而是比较 ego 与参考轨迹在空间、进度/时间、航向趋势上的偏差。
- `omnidirectional_interaction_risk`：基于 ego 与动态 actor 的最近距离、危险帧比例、谨慎帧比例和近似 TTC 评估全向交互风险。当前 TTC 仍较粗糙，后续可继续加入相对速度方向和车体包围盒。
- `road_engineering_hazard_adaptation`：在 `hazard_zones` 内综合碰撞、时空偏差、交互风险和速度适配，评价道路工程类隐患适应能力。
- `comfort`：基于 ego 平面加速度是否超过 `comfort_accel_limit_mps2`。
- `control_stability`：基于相邻帧方向盘输入变化是否超过 `control_steer_delta_limit`。
- `long_tail_hazard_response`：进入 hazard 感知半径后，检查刹车、转向、减速或速度达标的响应时间，并结合最低速度、碰撞和 danger zone 使用情况评分。让行类场景可以允许进入 danger zone。

## 新的 `drivable_area` 计算

`drivable_area` 现在使用 `reference_trajectory`，默认格式为 `[[x, y, yaw], ...]`。这个轨迹只代表“合理参考”，不是专家最优解，所以阈值故意宽松：

- 横向偏差：允许 4 m，硬惩罚 12 m。
- 进度/时间偏差：允许 20 m 或 3 s，硬惩罚 60 m 或 8 s。
- 航向偏差：有 yaw 时参与，允许 45 度，硬惩罚 120 度。
- 聚合权重：横向 45%，进度/时间 35%，航向 20%。

details 会输出 `mean/max_lateral_deviation_m`、`mean/max_progress_error_m`、`mean/max_heading_error_deg`、`final_progress_m`。如果没有参考轨迹，分数为 1.0，并标注 `missing_reference_trajectory`。

## 总分和能力分

`leaderboard_driving_score` 是 0-100 分。当前实现用任务 gate 和安全 gate 作为主约束：

- `task_gate = 0.60 * route_completion + 0.20 * drivable_area + 0.20 * road_engineering_hazard_adaptation`
- `safety_gate = 0.65 * collision_penalty + 0.35 * omnidirectional_interaction_risk`

之后再乘以效率、速度适配、舒适、稳定和长尾响应修正项。这样路线没完成或安全很差时，总分会明显受限；但轻微静态碰撞不会像 Bench2Drive 一样直接造成极端归零。

`ability_score` 按 `scenario_tags` 聚合：

- A：道路工程隐患适应，侧重路线完成、时空偏差、hazard adaptation、速度适配。
- B：交通参与者交互，侧重路线完成、碰撞、交互风险、hazard response。
- C：天气/环境鲁棒，侧重路线完成、速度、舒适、控制稳定。

## Metadata 依赖

当前 schema 是 `roadtailbench.code_scene_metadata.v4`。metadata 只保留测试需要的信息，不再保存完整 Excel 行、重复 route、重复 centerline 或伪造 z。

核心字段：

- `scenario_id`、`town`、`description`
- `ego.role_names`、`ego.type_id`、`ego.start_match_radius_m`
- 兼容字段 `ego_role_names`、`ego_type_id`、`ego_blueprint`
- `reference_trajectory_source`
- `reference_trajectory_format`: `x_y` 或 `x_y_yaw`
- `reference_trajectory`
- `ego_start`、`ego_end`
- `reference_speed_kmh`
- `scenario_tags`、`ability_tags`
- `hazards`、`hazard_zones`、`speed_zones`
- 指标阈值：横向、进度/时间、航向容忍度

`z=0.5` 已不再自动写入。原始场景轨迹第三列通常是 yaw，不是 z。只有源数据真实提供 `x,y,z,yaw` 时才应该保留 z。

## 当前审计结果

审计报告位于 `metadata/metadata_audit.json` 和 `metadata/metadata_audit.csv`。

缺少静态 ego 参考轨迹的 36 个场景：

`RTB001, RTB002, RTB003, RTB004, RTB005, RTB006, RTB007, RTB008, RTB010, RTB026, RTB027, RTB028, RTB056, RTB067, RTB070, RTB073, RTB076, RTB077, RTB078, RTB079, RTB080, RTB083, RTB084, RTB085, RTB101, RTB103, RTB104, RTB105, RTB106, RTB107, RTB108, RTB109, RTB110, RTB114, RTB115, RTB123`

疑似有 ego actor 但没有 `role_name='ego'/'hero'` 的 25 个场景：

`RTB002, RTB003, RTB008, RTB009, RTB010, RTB016, RTB020, RTB028, RTB029, RTB030, RTB031, RTB057, RTB076, RTB077, RTB080, RTB082, RTB083, RTB096, RTB097, RTB098, RTB099, RTB100, RTB121, RTB122, RTB124`

静态分析无法明确识别 ego actor 的 18 个场景：

`RTB004, RTB006, RTB007, RTB017, RTB018, RTB026, RTB027, RTB067, RTB070, RTB073, RTB078, RTB079, RTB084, RTB085, RTB114, RTB115, RTB123, RTB125`

注意：这三类不是同一个问题。缺参考轨迹不代表没有 ego；没有 role_name 也不代表没有 ego，只是 runner 发现 ego 会不稳定。

## Scene 手动修改模板

后续逐个修 scene py 时，优先做这三件事：

1. ego 生成前设置 role name。

```python
bp_ego = bp_lib.find("vehicle.xxx")
bp_ego.set_attribute("role_name", "ego")
ego = world.try_spawn_actor(bp_ego, transform)
```

如果用 `RTB.spawn_vehicle`：

```python
ego = RTB.spawn_vehicle(world, "vehicle.xxx", x=..., y=..., yaw=..., role_name="ego")
```

2. 如果 ego 有固定轨迹，把原始 `x,y,yaw` 轨迹保留为明确变量名，例如 `RAW_TRAJ_EGO` 或 `EGO_TRAJECTORY`，并在控制逻辑中让 ego 跟随该轨迹。

3. 如果 ego 是 TrafficManager、随机车道保持或动态路线，至少补一个明确的 `ego_end` 或终止区域。否则只能依赖 timeout，可能导致 ego 控制结束后继续靠惯性撞墙，污染指标。

## 运行终止规则

自然结束只看 ego：

- ego 到达 `ego_end` 或参考轨迹终点。
- ego actor 被销毁。
- 其他 actor 销毁不结束场景。
- timeout 只作为兜底。

每个场景结束、timeout 或异常退出时，runner 都会先恢复 CARLA 异步模式，再清理进程或进入下一个场景，避免 CARLA 卡死。
