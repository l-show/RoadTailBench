# RoadTailBench Metadata v1

metadata 只服务于评测语义，不负责加载场景。动态场景由 `RTBXXX.py` 自己定义。

核心字段：

- `scenario_id`: 场景编号，例如 `RTB116`。
- `town`: CARLA map 名称。
- `ego_role_names`: scene_ego 模式优先匹配的 role name，默认 `["ego", "hero"]`。
- `ego_type_id` / `ego_blueprint`: 没有 role name 时用于匹配或生成 ego。
- `ego_start` / `ego_end`: ego 起终点。
- `route`: 路线进度指标使用。
- `centerline_route`: 中心线偏差指标使用。
- `centerline_segments`: 可选，多车道/换道中心线段；每帧使用最近 segment。
- `speed_zones`: 局部速度语义。
- `hazards` / `hazard_zones`: 长尾危险响应和道路工程适应性指标使用。
- `scenario_tags`: 能力标签，例如 `B.emergency_avoidance`。

`drivable_area` 指标名保留，但含义是中心线偏差，不再要求写不规则可行区域 polygon。
