# leaderboard Metadata v4

metadata 只服务于评测语义，不负责加载或运行场景。动态 actor、天气、同步模式和控制逻辑仍由 `scenes/RTBXXX.py` 定义。

## 核心字段

- `schema_version`: 当前为 `roadtailbench.code_scene_metadata.v4`。
- `scenario_id`: 场景编号，例如 `RTB116`。
- `town`: CARLA map 名称或代码场景使用的地图标识。
- `description`: 场景描述。
- `ego`: ego 发现和匹配信息，包含 `role_names`、`type_id`、`start_match_radius_m`。
- `ego_role_names` / `ego_type_id` / `ego_blueprint`: runner 兼容字段。
- `ego_start` / `ego_end`: ego 起终点。自然结束只看 ego 是否到达终点或 ego 是否被销毁。
- `reference_trajectory_source`: 静态识别到的 ego 轨迹变量名；没有时为 `not_static_ego_reference_detected`。
- `reference_trajectory_format`: `x_y` 或 `x_y_yaw`。
- `reference_trajectory`: ego 合理参考轨迹，格式为 `[[x, y, yaw], ...]` 或 `[[x, y], ...]`。
- `reference_speed_kmh`: 从场景代码推断的参考巡航速度，兼容旧字段。
- `speed_limit_kmh`: 普通区域速度上限。
- `hazards`: 隐患点列表，每项建议包含 `id`、`center`、`radius_m`、`perception_radius_m`、`reference_speed_kmh`。
- `capability_vector`: 固定候选表的 0/1 能力向量，分为 `behavior` 和 `hazard` 两类。
- `scenario_tags` / `ability_tags`: 兼容旧字段，不再作为能力评分主依据。

## 已删除或不再推荐字段

- `excel_metadata`: 不再写入。Excel 原始行只作为生成输入，不属于评测必要信息。
- `route_waypoints`: 删除，避免重复。
- `centerline_route`: 删除。评估统一从 `reference_trajectory` 读取；旧 run config 仍兼容。
- `route`: 新 metadata 不再写入；旧 run config 仍兼容。
- `hazard_zones`: 删除。道路工程隐患适应指标已移除，隐患语义统一放入 `hazards`。
- `speed_zones`: 不再推荐。速度适配只需要全局 `speed_limit_kmh` 和 hazard 的 `reference_speed_kmh`。
- `A.xxx/B.xxx/C.xxx` 子类标签：不再用于能力评分。
- 自由文本能力标签：不再推荐。新增能力必须先进入 `metadata/capability_taxonomy.json` 候选表。
- 伪造的 `z=0.5`: 删除。原始三元轨迹通常是 `x,y,yaw`，不是 `x,y,z`。

## 轨迹约定

`reference_trajectory` 是合理参考轨迹，不是专家最优轨迹。`trajectory_adherence` 指标会用宽松阈值比较实际 ego 与该轨迹的空间、时间/进度和航向趋势偏差。

缺少 `reference_trajectory` 时，`trajectory_adherence` 返回 0，不再默认满分。`route_completion` 可以在有 `ego_start` 和 `ego_end` 时使用起终点兜底，但这只能说明 ego 接近了终点，不能说明它沿合理轨迹通过了场景。

如果场景没有固定 ego 轨迹，例如 TrafficManager、随机车道保持或动态路线，metadata 可以暂时保留空 `reference_trajectory`，但必须在审计报告中标出，后续应人工补充 `ego_end` 或明确参考路线。

## 审计报告

运行：

```powershell
python scripts\generate_metadata.py
```

会更新 100 个 metadata，并生成：

- `metadata/metadata_audit.json`
- `metadata/metadata_audit.csv`
- `outputs/metadata_generation_report.json`

审计字段包括：

- `has_ego_actor`
- `has_role_name_ego`
- `has_static_ego_trajectory`
- `ego_actor_names`
- `reference_trajectory_points`
- `reference_trajectory_source`
- `needs_scene_edit_reason`
- `blocking_metadata_issues`
