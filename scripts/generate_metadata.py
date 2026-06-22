import ast
import csv
import json
import math
import re
from pathlib import Path

try:
    from openpyxl import load_workbook
except ImportError:  # pragma: no cover - optional generation dependency
    load_workbook = None


ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"
METADATA = ROOT / "metadata"
OUTPUTS = ROOT / "outputs"


A_KEYWORDS = [
    ("traffic_sign_marking", ["交通标志", "标志", "标线", "限速", "方向指示", "警告"]),
    ("separation_protection", ["隔离", "护栏", "防护", "中央分隔"]),
    ("speed_control_facility", ["减速带", "限速", "测速", "速度控制"]),
    ("lighting_facility", ["照明", "路灯", "远光", "眩光", "逆光", "低光"]),
    ("pavement_condition", ["湿滑", "积水", "坑洼", "路面", "低附着", "冰雪", "施工"]),
    ("alignment_geometry", ["急弯", "匝道", "坡", "螺旋", "弯道", "陡坡"]),
    ("sight_distance", ["视距", "遮挡", "盲区", "视线", "低能见度", "浓雾", "植被"]),
    ("clearance_intrusion", ["侵入", "掉落", "货物", "障碍", "树枝", "广告牌", "临停", "占道"]),
]

B_KEYWORDS = [
    ("overtaking_bypass", ["绕行", "避让", "变道", "跨越", "障碍", "施工", "占道", "掉落", "树枝"]),
    ("merging_flow", ["汇入", "并入", "合流", "匝道", "出口", "入口", "加塞"]),
    ("emergency_avoidance", ["紧急", "突发", "急刹", "碰撞", "追尾", "横穿", "避险", "冲突"]),
    ("yielding_priority", ["让行", "会车", "路口", "交叉", "行人", "对向", "优先", "丁字", "无信号"]),
]

C_KEYWORDS = [
    ("low_light", ["黄昏", "傍晚", "夜", "低光", "无光", "low_light", "low light"]),
    ("glare", ["眩光", "逆光", "落日", "glare"]),
    ("fog", ["雾", "浓雾", "fog", "低能见度"]),
    ("rain_wet", ["雨", "暴雨", "小雨", "湿", "积水", "wet", "rain"]),
    ("snow_low_friction", ["雪", "冰", "低附着", "低摩擦"]),
    ("wind_dust_visibility", ["风", "沙尘", "尘", "dust"]),
]


def read_json(path):
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def load_excel_rows():
    if load_workbook is None:
        return {}
    workbooks = sorted(ROOT.glob("RoadTailGen*.xlsx"))
    if not workbooks:
        return {}
    wb = load_workbook(workbooks[0], read_only=True, data_only=True)
    ws = wb.worksheets[0]
    rows = {}
    merged = {"source": None, "original_scene_id": None, "source_hazards": None}
    for row_num, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=2):
        source, original, hazards, variant, scenario_id, typical, scene_type, road, env, maker, variant_hazard, desc = row[:12]
        if source:
            merged["source"] = source
        if original:
            merged["original_scene_id"] = original
        if hazards:
            merged["source_hazards"] = hazards
        if not scenario_id:
            continue
        scenario_id = str(scenario_id).strip()
        if not re.fullmatch(r"RTB\d{3}", scenario_id):
            continue
        rows[scenario_id] = {
            "excel_row": row_num,
            "source": source or merged["source"],
            "original_scene_id": original or merged["original_scene_id"],
            "source_hazards": hazards or merged["source_hazards"],
            "variant": variant,
            "scenario_typical": typical,
            "scene_type": scene_type,
            "road": road,
            "environment": env,
            "maker": maker,
            "variant_hazard": variant_hazard,
            "description": desc,
        }
    return rows


def read_source(path):
    for encoding in ("utf-8", "gbk", "latin-1"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    return path.read_text(errors="replace")


def numeric_tuple(node):
    if isinstance(node, (ast.Tuple, ast.List)):
        values = []
        for item in node.elts:
            if isinstance(item, ast.Constant) and isinstance(item.value, (int, float)):
                values.append(float(item.value))
            elif isinstance(item, ast.UnaryOp) and isinstance(item.op, ast.USub) and isinstance(item.operand, ast.Constant):
                values.append(-float(item.operand.value))
            else:
                return None
        return values if len(values) >= 2 else None
    return None


def literal_sequences(source):
    tree = ast.parse(source)
    lists, strings = {}, {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        names = [target.id for target in node.targets if isinstance(target, ast.Name)]
        if not names:
            continue
        if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            for name in names:
                strings[name] = node.value.value
        elif isinstance(node.value, (ast.List, ast.Tuple)):
            seq = [numeric_tuple(item) for item in node.value.elts]
            if seq and all(item is not None for item in seq):
                for name in names:
                    lists[name] = seq
    return lists, strings


def parse_string_points(text):
    points = []
    for line in text.replace(",", " ").splitlines():
        nums = []
        for part in line.split():
            try:
                nums.append(float(part))
            except ValueError:
                pass
        if len(nums) >= 2:
            points.append(nums)
    return points


def clean_points(points, min_dist=0.5):
    if not points:
        return []
    out = [points[0]]
    for point in points[1:]:
        prev = out[-1]
        if math.hypot(point[0] - prev[0], point[1] - prev[1]) >= min_dist:
            out.append(point)
    return out


def assigned_name_before(source, position):
    prefix = source[max(0, position - 180):position]
    match = re.search(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?:[A-Za-z_][A-Za-z0-9_]*\.)?$", prefix)
    return match.group(1) if match else ""


def call_texts(source, function_name):
    texts = []
    for match in re.finditer(re.escape(function_name) + r"\s*\(", source):
        start = match.start()
        depth = 0
        end = None
        for idx in range(match.end() - 1, min(len(source), match.end() + 2000)):
            ch = source[idx]
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    end = idx + 1
                    break
        if end:
            texts.append((start, source[start:end]))
    return texts


def infer_ego_actor_names(source):
    names = set()
    for start, call in call_texts(source, "spawn_vehicle"):
        assigned = assigned_name_before(source, start)
        if re.search(r"role_name\s*=\s*['\"](?:ego|hero)['\"]", call, re.I) or "ego" in assigned.lower():
            if assigned:
                names.add(assigned)
    for start, call in call_texts(source, "try_spawn_actor"):
        assigned = assigned_name_before(source, start)
        local = source[max(0, start - 800):start]
        has_role = re.search(r"set_attribute\s*\(\s*['\"]role_name['\"]\s*,\s*['\"](?:ego|hero)['\"]", local, re.I)
        if has_role or "ego" in assigned.lower():
            if assigned:
                names.add(assigned)
    return sorted(names)


def infer_ego_route_names(source, ego_actor_names):
    names = set()
    literal_lists, literal_strings = literal_sequences(source)
    for name in list(literal_lists) + list(literal_strings):
        if "ego" in name.lower():
            names.add(name)
    for actor in ego_actor_names:
        pattern = (
            r"get_target_waypoint\s*\(\s*"
            + re.escape(actor)
            + r"\.get_location\(\)\s*,\s*([A-Za-z_][A-Za-z0-9_]*)"
        )
        names.update(re.findall(pattern, source))
    for match in re.finditer(r"RTB\.parse_string_trajectory\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)", source):
        assigned = assigned_name_before(source, match.start())
        if "ego" in assigned.lower():
            names.add(match.group(1))
            names.add(assigned)
    return names


def route_from_source(source):
    literal_lists, literal_strings = literal_sequences(source)
    ego_names = infer_ego_actor_names(source)
    route_names = infer_ego_route_names(source, ego_names)
    candidates = {}
    for name in route_names:
        if name in literal_lists:
            candidates[name] = clean_points(literal_lists[name])
        elif name in literal_strings:
            candidates[name] = clean_points(parse_string_points(literal_strings[name]))
    for name in sorted(candidates, key=lambda n: (0 if "ego" in n.lower() else 1, n)):
        points = candidates[name]
        if len(points) >= 2:
            return name, points
    return None, []


def point_to_reference(point):
    if len(point) >= 3:
        return [round(float(point[0]), 3), round(float(point[1]), 3), round(float(point[2]), 3)]
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def yaw_from_point(point, next_point=None):
    if len(point) >= 3:
        return float(point[2])
    if next_point:
        return math.degrees(math.atan2(next_point[1] - point[1], next_point[0] - point[0]))
    return 0.0


def infer_ego_blueprint(source):
    for start, call in call_texts(source, "spawn_vehicle"):
        assigned = assigned_name_before(source, start)
        if not (re.search(r"role_name\s*=\s*['\"](?:ego|hero)['\"]", call, re.I) or "ego" in assigned.lower()):
            continue
        vehicle = re.search(r"['\"](vehicle\.[A-Za-z0-9_.-]+)['\"]", call)
        if vehicle:
            return vehicle.group(1)
    for start, call in call_texts(source, "try_spawn_actor"):
        assigned = assigned_name_before(source, start)
        if "ego" not in assigned.lower():
            continue
        local = source[max(0, start - 700):start]
        bp_var = re.search(r"try_spawn_actor\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)", call)
        if bp_var:
            find = re.search(re.escape(bp_var.group(1)) + r"\s*=\s*bp_lib\.find\s*\(\s*['\"](vehicle\.[A-Za-z0-9_.-]+)['\"]", local)
            if find:
                return find.group(1)
    all_matches = re.findall(r"['\"](vehicle\.[A-Za-z0-9_.-]+)['\"]", source)
    return all_matches[0] if all_matches else "vehicle.audi.tt"


def infer_reference_speed(source):
    values = []
    values.extend(float(v) for v in re.findall(r"sm_ego\s*=\s*RTB\.MultiStageBehaviorMachine\(initial_speed\s*=\s*([0-9.]+)", source))
    values.extend(float(v) for v in re.findall(r"ego_sm\s*=\s*RTB\.MultiStageBehaviorMachine\(initial_speed\s*=\s*([0-9.]+)", source))
    values.extend(float(v) for v in re.findall(r"set_vehicle_initial_speed\(\s*ego(?:_vehicle)?\s*,\s*(?:target_speed_kmh\s*=\s*)?([0-9.]+)", source))
    return round(max(values) if values else 60.0, 1)


def keyword_tags(text, table, prefix):
    lower = text.lower()
    return [f"{prefix}.{subtype}" for subtype, words in table if any(word.lower() in lower for word in words)]


def build_tags(excel, source, old):
    excel_text = "\n".join(str(v) for v in [
        excel.get("source_hazards"),
        excel.get("scenario_typical"),
        excel.get("scene_type"),
        excel.get("environment"),
        excel.get("variant_hazard"),
        excel.get("description"),
    ] if v)
    tags = []
    tags.extend(keyword_tags(excel_text, A_KEYWORDS, "A"))
    tags.extend(keyword_tags(excel_text, B_KEYWORDS, "B"))
    tags.extend(keyword_tags(excel_text, C_KEYWORDS, "C"))
    code_text = source.lower()
    if any(word in code_text for word in ("lane_change", "change_lane", "overtak", "bypass")):
        tags.append("B.overtaking_bypass")
    if any(word in code_text for word in ("merge", "ramp", "yield")):
        tags.append("B.merging_flow")
    if any(word in code_text for word in ("walker.pedestrian", "pedestrian", "opposing", "junction")):
        tags.append("B.yielding_priority")
    if any(word in code_text for word in ("collision", "emergency", "brake", "obstacle")):
        tags.append("B.emergency_avoidance")
    if not tags and old.get("scenario_tags"):
        tags.extend(old["scenario_tags"])
    if not any(tag.startswith("A.") for tag in tags):
        tags.append("A.sight_distance")
    if not any(tag.startswith("B.") for tag in tags):
        tags.append("B.emergency_avoidance")
    return sorted(dict.fromkeys(tags))


def simplify_ability_tags(tags):
    groups = []
    for tag in tags:
        group = str(tag).split(".", 1)[0]
        if group in ("A", "B", "C") and group not in groups:
            groups.append(group)
    return groups or ["A", "B"]


BEHAVIOR_CAPABILITY_KEYS = [
    "lane_change",
    "overtaking",
    "bypass_obstacle",
    "car_following",
    "yielding",
    "merge_or_cut_in",
    "intersection_crossing",
    "pedestrian_interaction",
    "emergency_braking",
    "low_speed_maneuver",
]

HAZARD_CAPABILITY_KEYS = [
    "traffic_sign_marking",
    "road_geometry",
    "limited_sight_distance",
    "road_surface_low_friction",
    "static_obstacle_or_intrusion",
    "falling_or_moving_obstacle",
    "construction_or_lane_blockage",
    "priority_conflict",
    "adverse_weather_visibility",
    "adverse_lighting_glare",
    "adverse_lighting_low_light",
]


def zero_vector(keys):
    return {key: 0 for key in keys}


def build_capability_vector(tags, source, excel):
    behavior = zero_vector(BEHAVIOR_CAPABILITY_KEYS)
    hazard = zero_vector(HAZARD_CAPABILITY_KEYS)
    text = "\n".join(str(v) for v in [
        excel.get("source_hazards"),
        excel.get("scenario_typical"),
        excel.get("scene_type"),
        excel.get("road"),
        excel.get("environment"),
        excel.get("variant_hazard"),
        excel.get("description"),
        source,
    ] if v).lower()

    if any(word in text for word in ("lane_change", "change_lane", "变道", "并线")):
        behavior["lane_change"] = 1
    if any(word in text for word in ("overtak", "超车", "跨越")):
        behavior["overtaking"] = 1
    if any(word in text for word in ("bypass", "绕行", "避让", "障碍", "obstacle", "construction")):
        behavior["bypass_obstacle"] = 1
    if any(word in text for word in ("follow", "跟车", "carla.command", "autopilot")):
        behavior["car_following"] = 1
    if any(word in text for word in ("yield", "让行", "优先", "会车")):
        behavior["yielding"] = 1
    if any(word in text for word in ("merge", "cut_in", "cut-in", "汇入", "合流", "匝道")):
        behavior["merge_or_cut_in"] = 1
    if any(word in text for word in ("junction", "intersection", "crossing", "路口", "交叉", "丁字")):
        behavior["intersection_crossing"] = 1
    if any(word in text for word in ("walker.pedestrian", "pedestrian", "行人")):
        behavior["pedestrian_interaction"] = 1
    if any(word in text for word in ("emergency", "brake", "急刹", "紧急", "突发")):
        behavior["emergency_braking"] = 1
    if any(word in text for word in ("parking", "low_speed", "低速", "倒车", "泊车")):
        behavior["low_speed_maneuver"] = 1

    for tag in tags:
        if tag.endswith("traffic_sign_marking"):
            hazard["traffic_sign_marking"] = 1
        elif tag.endswith("alignment_geometry"):
            hazard["road_geometry"] = 1
        elif tag.endswith("sight_distance"):
            hazard["limited_sight_distance"] = 1
        elif tag.endswith("pavement_condition"):
            hazard["road_surface_low_friction"] = 1
        elif tag.endswith("clearance_intrusion"):
            hazard["static_obstacle_or_intrusion"] = 1
        elif tag.endswith("rain_wet") or tag.endswith("fog") or tag.endswith("wind_dust_visibility"):
            hazard["adverse_weather_visibility"] = 1
        elif tag.endswith("glare"):
            hazard["adverse_lighting_glare"] = 1
        elif tag.endswith("low_light"):
            hazard["adverse_lighting_low_light"] = 1
    if any(word in text for word in ("falling", "落石", "滚落", "掉落", "货物", "box")):
        hazard["falling_or_moving_obstacle"] = 1
    if any(word in text for word in ("construction", "施工", "占道", "lane_block")):
        hazard["construction_or_lane_blockage"] = 1
    if any(word in text for word in ("yield", "让行", "优先", "冲突", "priority")):
        hazard["priority_conflict"] = 1

    if not any(behavior.values()):
        behavior["car_following"] = 1
    if not any(hazard.values()):
        hazard["limited_sight_distance"] = 1
    return {"behavior": behavior, "hazard": hazard}


def expected_behavior_for_b(tags):
    if "B.yielding_priority" in tags:
        return "yield_or_stop_for_priority_conflict"
    if "B.merging_flow" in tags:
        return "maintain_safe_gap_for_merge"
    if "B.overtaking_bypass" in tags:
        return "bypass_or_change_lane_safely"
    if "B.emergency_avoidance" in tags:
        return "slow_or_avoid"
    return "slow_or_avoid"


def hazard_center(reference):
    if not reference:
        return [0.0, 0.0]
    point = reference[len(reference) // 2]
    return [point[0], point[1]]


def actor_audit(source, reference):
    ego_names = infer_ego_actor_names(source)
    has_role = bool(re.search(r"role_name\s*=\s*['\"](?:ego|hero)['\"]", source, re.I)) or bool(
        re.search(r"set_attribute\s*\(\s*['\"]role_name['\"]\s*,\s*['\"](?:ego|hero)['\"]", source, re.I)
    )
    has_spawn = bool(re.search(r"spawn_vehicle\s*\(|try_spawn_actor\s*\(", source))
    has_ego_actor = bool(ego_names) or has_role
    reasons = []
    if not has_ego_actor and has_spawn:
        reasons.append("no_clear_ego_actor_detected")
    if has_ego_actor and not has_role:
        reasons.append("ego_actor_missing_role_name")
    if len(reference) < 2:
        reasons.append("missing_static_ego_reference_trajectory")
    return {
        "has_ego_actor": has_ego_actor,
        "has_role_name_ego": has_role,
        "has_static_ego_trajectory": len(reference) >= 2,
        "ego_actor_names": ego_names,
        "needs_scene_edit_reason": reasons,
    }


def build_metadata(scene_path, excel, old):
    source = read_source(scene_path)
    route_name, points = route_from_source(source)
    reference = [point_to_reference(point) for point in points]
    tags = build_tags(excel, source, old)
    b_tags = [tag for tag in tags if tag.startswith("B.")]
    a_tags = [tag for tag in tags if tag.startswith("A.")]
    c_tags = [tag for tag in tags if tag.startswith("C.")]
    primary_b = b_tags[0].split(".", 1)[1] if b_tags else "emergency_avoidance"
    primary_a = a_tags[0].split(".", 1)[1] if a_tags else "sight_distance"
    reference_speed = infer_reference_speed(source)
    description = excel.get("description") or old.get("description") or (
        f"Original scenario body for {excel.get('original_scene_id') or old.get('scenario_family_id') or scene_path.stem}; detailed variant description is unavailable."
    )
    metadata = {
        "schema_version": "roadtailbench.code_scene_metadata.v4",
        "scenario_id": scene_path.stem,
        "town": old.get("town") or scene_path.stem,
        "description": description,
        "scenario_family_id": excel.get("original_scene_id") or old.get("scenario_family_id"),
        "scene_type": excel.get("scene_type") or old.get("scene_type"),
        "environment": excel.get("environment") or old.get("environment"),
        "source_hazards": excel.get("source_hazards") or old.get("source_hazards"),
        "ego": {
            "role_names": ["ego", "hero"],
            "type_id": infer_ego_blueprint(source),
            "start_match_radius_m": 8.0,
        },
        "ego_role_names": ["ego", "hero"],
        "ego_type_id": infer_ego_blueprint(source),
        "ego_blueprint": infer_ego_blueprint(source),
        "ego_start_match_radius_m": 8.0,
        "reference_trajectory_source": route_name or "not_static_ego_reference_detected",
        "reference_trajectory_format": "x_y_yaw" if reference and len(reference[0]) >= 3 else "x_y",
        "reference_trajectory": reference,
        "reference_speed_kmh": reference_speed,
        "speed_limit_kmh": reference_speed,
        "allowed_lateral_error_m": 4.0,
        "hard_lateral_error_m": 12.0,
        "allowed_progress_error_m": 20.0,
        "hard_progress_error_m": 60.0,
        "allowed_time_error_s": 3.0,
        "hard_time_error_s": 8.0,
        "allowed_heading_error_deg": 45.0,
        "hard_heading_error_deg": 120.0,
        "scenario_tags": simplify_ability_tags(tags),
        "ability_tags": simplify_ability_tags(tags),
        "capability_vector": build_capability_vector(tags, source, excel),
    }
    if reference:
        metadata["ego_start"] = {
            "location": {"x": reference[0][0], "y": reference[0][1]},
            "rotation": {"pitch": 0.0, "yaw": round(yaw_from_point(points[0], points[1] if len(points) > 1 else None), 3), "roll": 0.0},
        }
        metadata["ego_end"] = {
            "location": {"x": reference[-1][0], "y": reference[-1][1]},
            "rotation": {"pitch": 0.0, "yaw": round(yaw_from_point(points[-1]), 3), "roll": 0.0},
        }
        center = hazard_center(reference)
        expected = expected_behavior_for_b(tags)
        metadata["hazards"] = [{
            "id": f"{scene_path.stem.lower()}_primary_hazard",
            "type": primary_b,
            "center": center,
            "radius_m": 10.0,
            "perception_radius_m": 40.0,
            "danger_radius_m": 4.0,
            "reference_speed_kmh": min(40.0, reference_speed),
            "allow_enter_danger_zone": expected.startswith("yield"),
            "expected_behavior": expected,
        }]
    else:
        metadata["notes"] = [
            "No fixed ego reference trajectory was statically detected; natural end by ego goal is unavailable until metadata or scene code is refined.",
            "Scenario timeout remains the termination fallback for this scene.",
        ]
        metadata["blocking_metadata_issues"] = ["missing_reference_trajectory"]
    if not reference and not (metadata.get("ego_start") and metadata.get("ego_end")):
        metadata.setdefault("blocking_metadata_issues", []).append("missing_reference_trajectory_and_ego_end")
    return {key: value for key, value in metadata.items() if value not in (None, [], {}) or key in {"reference_trajectory", "scenario_tags"}}


def main():
    excel_rows = load_excel_rows()
    old_metadata = {path.stem: read_json(path) for path in METADATA.glob("RTB*.json")}
    METADATA.mkdir(exist_ok=True)
    OUTPUTS.mkdir(exist_ok=True)
    report = []
    for scene_path in sorted(SCENES.glob("RTB*.py")):
        scene_id = scene_path.stem
        metadata = build_metadata(scene_path, excel_rows.get(scene_id, {}), old_metadata.get(scene_id, {}))
        write_json(METADATA / f"{scene_id}.json", metadata)
        source = read_source(scene_path)
        audit = actor_audit(source, metadata.get("reference_trajectory", []))
        row = {
            "scenario_id": scene_id,
            "has_ego_actor": audit["has_ego_actor"],
            "has_role_name_ego": audit["has_role_name_ego"],
            "has_static_ego_trajectory": audit["has_static_ego_trajectory"],
            "ego_actor_names": audit["ego_actor_names"],
            "reference_trajectory_points": len(metadata.get("reference_trajectory", [])),
            "reference_trajectory_source": metadata.get("reference_trajectory_source"),
            "needs_scene_edit_reason": audit["needs_scene_edit_reason"],
            "blocking_metadata_issues": metadata.get("blocking_metadata_issues", []),
        }
        report.append(row)
    write_json(OUTPUTS / "metadata_generation_report.json", report)
    write_json(METADATA / "metadata_audit.json", report)
    with (METADATA / "metadata_audit.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(report[0].keys()) if report else [])
        writer.writeheader()
        writer.writerows(report)
    missing_routes = [r["scenario_id"] for r in report if not r["has_static_ego_trajectory"]]
    missing_roles = [r["scenario_id"] for r in report if r["has_ego_actor"] and not r["has_role_name_ego"]]
    missing_ego = [r["scenario_id"] for r in report if not r["has_ego_actor"]]
    print(f"wrote {len(report)} metadata files")
    print(f"missing static ego reference trajectories: {len(missing_routes)} {missing_routes}")
    print(f"ego actor missing role_name: {len(missing_roles)} {missing_roles}")
    print(f"no clear ego actor detected: {len(missing_ego)} {missing_ego}")


if __name__ == "__main__":
    main()
