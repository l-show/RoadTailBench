# leaderboard Metadata v4

metadata 只服务于评测语义，不负责加载或运行场景。动态 actor、天气、同步模式和控制逻辑仍由 `scenes/RTBXXX.py` 定义。

## 核心字段

- `schema_version`: 当前为 `roadtailbench.code_scene_metadata.v4`。
- `scenario_id`: 场景编号，例如 `RTB116`。
- `town`: CARLA map 名称或代码场景使用的地图标识。
- `description`: 场景描述。
- `ego`: ego 发现和匹配信息，包含 `role_names`、`type_id`、`start_match_radius_m`。
- `ego_role_names` / `ego_type_id` / `ego_blueprint`: runner 兼容字段。新场景建议 `ego_role_names` 只写 `["ego"]`。
- `ego_start` / `ego_end`: ego 起终点。自然结束只看 ego 是否到达终点或 ego 是否被销毁。
- `reference_trajectory_format`: `x_y` 或 `x_y_yaw`。
- `reference_trajectory`: ego 合理参考路线，格式为 `[[x, y, yaw], ...]` 或 `[[x, y], ...]`。
- `trajectory_adherence_mode`: 默认为 `spatial`。可显式写 `spatiotemporal` 复用旧的进度/时间偏差评分。
- `speed_limit_kmh`: 普通区域速度上限。
- `reference_speed_kmh`: 兼容旧字段，仍可被效率和速度指标读取，但默认不再用于轨迹贴合评分。
- `hazards`: 隐患点列表，每项建议包含 `id`、`center`、`radius_m`、`perception_radius_m`、`reference_speed_kmh`。
- `capability_vector`: 固定候选表的 0/1 能力向量，分为 `behavior` 和 `hazard` 两类。

## Ego 绑定约定

`scene_ego` 模式下场景脚本自己生成和控制 ego，runner 只负责发现它。发现优先级是：

1. 按 actor 的 `role_name` 匹配 metadata/CLI 中的 `ego_role_names`，默认 `ego,hero`。
2. 找不到时按 `ego_type_id` / `ego_blueprint` 匹配车型。
3. 如果同车型不唯一，再按 `ego_start` 和 `ego_start_match_radius_m` 选择最近车辆。

因此每个场景必须只给真正自车设置 `role_name='ego'`。背景车不要使用 `ego` 或 `hero`。

## Metadata 模板

```json
{
  "schema_version": "roadtailbench.code_scene_metadata.v4",
  "scenario_id": "RTBXXX",
  "town": "RTBXXX",
  "ego": {
    "role_names": ["ego"],
    "type_id": "vehicle.xxx",
    "start_match_radius_m": 8.0
  },
  "ego_role_names": ["ego"],
  "ego_type_id": "vehicle.xxx",
  "ego_blueprint": "vehicle.xxx",
  "ego_start_match_radius_m": 8.0,
  "ego_start": {
    "location": {"x": 0.0, "y": 0.0},
    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
  },
  "ego_end": {
    "location": {"x": 100.0, "y": 0.0},
    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
  },
  "reference_trajectory_format": "x_y_yaw",
  "reference_trajectory": [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
  "trajectory_adherence_mode": "spatial",
  "speed_limit_kmh": 50.0,
  "hazards": []
}
```

## 轨迹约定

`reference_trajectory` 是合理参考路线，不是专家最优时空轨迹。默认 `trajectory_adherence` 只评价 ego 相对参考路线的横向偏差和航向偏差；不要求 metadata 记录每个时刻的位置，也不因车辆快慢直接扣轨迹分。

速度相关行为由 `speed_appropriateness`、`driving_efficiency`、`comfort`、`control_stability` 等指标处理。若确实需要旧版“按参考速度推算期望进度”的评分，可在单个场景 metadata 中设置：

```json
{"trajectory_adherence_mode": "spatiotemporal"}
```

缺少 `reference_trajectory` 时，`trajectory_adherence` 返回 0。`route_completion` 可以在有 `ego_start` 和 `ego_end` 时使用起终点兜底，但这只能说明 ego 接近了终点，不能说明它沿合理路线通过了场景。

## 审计报告

运行：

```powershell
python scripts\generate_metadata.py
```

会更新 metadata，并生成：

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
- `missing_role_name_ego`
- `ambiguous_ego_actor`
- `missing_reference_trajectory`
- `missing_ego_start_end`
- `needs_scene_edit_reason`
- `blocking_metadata_issues`
