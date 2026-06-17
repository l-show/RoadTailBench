# RoadTailBench 指标说明 v1

本文档描述当前 `leaderboard-eval` 使用的离线指标。输入是 `leaderboard_frame_log.jsonl` 和 `leaderboard_scenario_config.json`。

## 运行终止

- `scene_ego` 场景脚本自己控制 ego，runner 只记录和评价。
- 自然结束只看 ego：ego 到达 `ego_end` / route 终点，或 ego actor 被销毁。
- `scenario_timeout` 是现实墙钟时间兜底，不是 CARLA 仿真时间。同步模式和场景脚本调度可能导致 60s 现实时间只推进更少仿真时间。
- 每个场景退出时都会强制恢复 CARLA 异步模式。

## 10 类核心指标

- `route_completion`：把 ego 位置投影到 metadata route / centerline，取最大进度除以路线总长。
- `collision_penalty`：合并短时间重复碰撞后，按对象类型给连续惩罚。walker 权重最高，vehicle 次之，static/prop 较低；可通过 `collision_type_weights`、`collision_tolerance_weight`、`collision_penalty_scale` 调整。
- `driving_efficiency`：ego 平均速度相对 `reference_speed_kmh` 的比例，逐帧 clamp 到 `[0,1]`。
- `speed_appropriateness`：逐帧比较 ego 速度和全局/区域目标速度，偏离越大分越低。
- `drivable_area`：当前没有手工可行驶多边形时，使用 route/centerline corridor 近似；它评价的是“是否偏离预设路线”，不是严格道路可行域。
- `omnidirectional_interaction_risk`：基于 ego 与动态 actor 最近距离、danger/caution frame ratio、近似 TTC 计算交互风险。
- `road_engineering_hazard_adaptation`：在 `hazard_zones` 内综合碰撞、可行域、交互和速度适配。
- `comfort`：基于 ego 平面加速度是否超过 `comfort_accel_limit_mps2`。
- `control_stability`：基于相邻帧 steer 变化是否超过阈值。
- `long_tail_hazard_response`：进入 hazard perception radius 后，检查刹车、转向、减速或速度达标的响应时间，并结合最低速度、碰撞和 danger zone 使用情况评分。

## 总分和能力分

- `leaderboard_driving_score`：连续分，总分 0-100。当前由任务完成 gate、交互安全 gate、效率、速度、舒适、稳定、hazard response 共同决定。
- `ability_score`：按 `scenario_tags` 分 A/B/C 能力组。
  - A 组侧重道路工程隐患适应：route、drivable、hazard adaptation、speed。
  - B 组侧重交通参与者交互：route、collision、interaction、hazard response。
  - C 组侧重天气/环境鲁棒：route、speed、comfort、control stability。

## Metadata 依赖

100 个现有 `scenes/RTB*.py` 已由 `scripts/generate_metadata.py` 从 Excel 和脚本静态解析生成 metadata。字段来源：

- Excel E 列：`scenario_id`
- Excel F/G：典型场景和场景类型
- Excel L：场景描述
- 场景脚本：ego route、ego blueprint、reference speed、天气/交互关键词辅助

当前生成结果：

- metadata 文件数：100。
- 有静态 ego route 的场景：65。
- 未静态识别 ego route 的场景：35，`route_source=not_static_route_detected`。这些多为 TrafficManager、lane-keeping、函数动态生成路径或非显式 ego route 的脚本。
- A/B 标签覆盖 100 个场景，C 天气/环境标签覆盖 69 个场景。

已知限制：

- 仍有一部分场景没有静态 ego route，metadata 会标记 `route_source=not_static_route_detected`，需要人工或运行日志补齐。
- 自动 hazard center 默认取 route 中点，必须人工复核。RTB117 已暴露真实风险点与自动 center 可能错位。
- drivable polygon 暂未手工录入，后续应优先接 CARLA lane/waypoint 或 OpenDRIVE road/lane 判定。
- B 类交互能力标签来自 Excel 文本和代码关键词推断，正式发布前需要人工审核。
- Excel 母体行通常没有 L 列描述，因此母体场景的 description 会使用 family fallback，后续应补人工描述。

## RTB116-RTB118 当前观察

- RTB116：60s wall timeout 时只推进 37.25s 仿真，ego 距终点约 89m，结果不能作为完整成绩，只说明 timeout 太短或场景未自然结束。
- RTB117：ego 到终点，但在 y≈-100 附近发生 HGV/static mesh 碰撞；metadata hazard center 原先在 y≈-176，需修正。
- RTB118：ego 到终点且无碰撞，在行人/对向车 hazard 附近明显刹车减速；新 hazard response 不再简单因进入 danger radius 判失败。

## 复评观察

用旧 RTB116/117/118 run 日志重算新指标后：

- RTB116：总分约 10。原因仍是未完整跑完、车辆碰撞严重、hazard response 低。
- RTB117：总分约 22.6。车辆/静态物碰撞不再硬归零，但 B 交互能力仍明显受罚。
- RTB118：总分约 60.8。无碰撞、到终点、hazard 前减速，分数高于旧公式；ability 约 0.88，不再虚高为 1.0。

本轮尝试直接运行 CARLA 复核时，当前执行环境加载 `carla` egg 失败：`DLL load failed while importing libcarla`。需要在用户的 `(Carla-0915)` 环境中重新运行验证命令。

## 建议验证命令

```powershell
leaderboard-run `
  --host localhost --port 2000 `
  --scene-root scenes `
  --metadata-root metadata `
  --scenes RTB117-RTB118 `
  --ego-mode scene_ego `
  --scenario-timeout 90 `
  --tick-wait-timeout 5 `
  --natural-end-distance-m 5 `
  --natural-end-min-ticks 5 `
  --output-root outputs\metric_upgrade_validation
```

RTB116 建议使用更长 timeout，例如 150s wall-clock，因为 60s wall-clock 只推进了约 37s CARLA 仿真时间。
