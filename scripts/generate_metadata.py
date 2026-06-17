import ast
import json
import math
import re
from pathlib import Path

from openpyxl import load_workbook


ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"
METADATA = ROOT / "metadata"


A_KEYWORDS = [
    ("traffic_sign_marking", ["交通标志", "标志", "标线", "限速标志", "方向指示牌", "警告标志"]),
    ("separation_protection", ["隔离", "护栏", "防护", "中央分隔"]),
    ("speed_control_facility", ["减速带", "限速", "测速", "速度控制"]),
    ("lighting_facility", ["照明", "路灯", "高 beam", "远光", "眩光", "逆光", "低光", "光照"]),
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


def load_excel_rows():
    workbook = next(ROOT.glob("RoadTailGen*.xlsx"))
    wb = load_workbook(workbook, read_only=True, data_only=True)
    ws = wb.worksheets[0]
    rows = {}
    last_source = last_original = last_hazards = None
    for row_num, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=2):
        source, original, hazards, variant, scenario_id, typical, scene_type, road, env, maker, variant_hazard, desc = row[:12]
        if source:
            last_source = source
        if original:
            last_original = original
        if hazards:
            last_hazards = hazards
        if not scenario_id:
            continue
        scenario_id = str(scenario_id).strip()
        if not re.fullmatch(r"RTB\d{3}", scenario_id):
            continue
        rows[scenario_id] = {
            "excel_row": row_num,
            "source": source or last_source,
            "original_scene_id": original or last_original,
            "source_hazards": hazards or last_hazards,
            "variant": variant,
            "scenario_id": scenario_id,
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
        if len(values) >= 2:
            return values
    return None


def literal_sequences(source):
    tree = ast.parse(source)
    values = {}
    strings = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
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
                        values[name] = seq
    return values, strings


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


def route_from_source(source):
    literal_lists, literal_strings = literal_sequences(source)
    candidates = {}
    for name, points in literal_lists.items():
        lname = name.lower()
        if "ego" in lname or lname in {"raw_path_a2", "path_points", "vehicle_path_points"}:
            candidates[name] = clean_points(points)
    for name, text in literal_strings.items():
        lname = name.lower()
        if "ego" in lname:
            points = parse_string_points(text)
            if points:
                candidates[name] = clean_points(points)

    preferred_names = sorted(
        candidates,
        key=lambda n: (
            0 if "ego" in n.lower() else 1,
            0 if "traj" in n.lower() or "path" in n.lower() else 1,
            n,
        ),
    )
    for name in preferred_names:
        points = candidates[name]
        if len(points) >= 2:
            return name, points
    return None, []


def route_to_locations(points):
    locations = []
    for point in points:
        z = point[2] if len(point) > 3 else 0.5
        locations.append([round(float(point[0]), 3), round(float(point[1]), 3), round(float(z), 3)])
    return locations


def yaw_from_point(point, next_point=None):
    if len(point) >= 3:
        return float(point[2])
    if next_point:
        return math.degrees(math.atan2(next_point[1] - point[1], next_point[0] - point[0]))
    return 0.0


def infer_ego_blueprint(source):
    for match in re.finditer(r"(?:RTB\.)?(?:safe_|force_)?spawn_vehicle\s*\(", source):
        start = match.start()
        depth = 0
        end = None
        for idx in range(match.end() - 1, min(len(source), match.end() + 1200)):
            ch = source[idx]
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    end = idx + 1
                    break
        if end is None:
            continue
        call = source[start:end]
        if not re.search(r"role_name\s*=\s*['\"](?:ego|hero)['\"]", call):
            continue
        vehicle = re.search(r"['\"](vehicle\.[A-Za-z0-9_.-]+)['\"]", call)
        if vehicle:
            return vehicle.group(1)
    all_matches = re.findall(r"['\"](vehicle\.[A-Za-z0-9_.-]+)['\"]", source)
    return all_matches[0] if all_matches else "vehicle.audi.tt"


def infer_reference_speed(source):
    values = []
    values.extend(float(v) for v in re.findall(r"sm_ego\s*=\s*RTB\.MultiStageBehaviorMachine\(initial_speed\s*=\s*([0-9.]+)", source))
    values.extend(float(v) for v in re.findall(r"ego_sm\s*=\s*RTB\.MultiStageBehaviorMachine\(initial_speed\s*=\s*([0-9.]+)", source))
    values.extend(float(v) for v in re.findall(r"set_vehicle_initial_speed\(\s*ego(?:_vehicle)?\s*,\s*(?:target_speed_kmh\s*=\s*)?([0-9.]+)", source))
    values.extend(float(v) for v in re.findall(r"set_vehicle_initial_speed\(\s*ego(?:_vehicle)?\s*,\s*target_speed_kmh\s*=\s*([0-9.]+)", source))
    return round(max(values) if values else 60.0, 1)


def keyword_tags(text, table, prefix):
    tags = []
    for subtype, words in table:
        if any(word.lower() in text.lower() for word in words):
            tags.append(f"{prefix}.{subtype}")
    return tags


def expected_behavior_for_b(tags, text):
    if "B.yielding_priority" in tags:
        return "yield_or_stop_for_priority_conflict"
    if "B.merging_flow" in tags:
        return "maintain_safe_gap_for_merge"
    if "B.overtaking_bypass" in tags:
        return "bypass_or_change_lane_safely"
    if "B.emergency_avoidance" in tags:
        return "slow_or_avoid"
    if "追尾" in text:
        return "avoid_rear_end_conflict"
    return "slow_or_avoid"


def build_tags(excel, source):
    excel_text = "\n".join(str(v) for v in [
        excel.get("source_hazards"),
        excel.get("scenario_typical"),
        excel.get("scene_type"),
        excel.get("environment"),
        excel.get("variant_hazard"),
        excel.get("description"),
    ] if v)
    code_text = source.lower()
    tags = []
    tags.extend(keyword_tags(excel_text, A_KEYWORDS, "A"))
    tags.extend(keyword_tags(excel_text, B_KEYWORDS, "B"))
    tags.extend(keyword_tags(excel_text, C_KEYWORDS, "C"))
    if any(word in code_text for word in ("lane_change", "change_lane", "overtak", "bypass")):
        tags.append("B.overtaking_bypass")
    if any(word in code_text for word in ("merge", "ramp", "yield")):
        tags.append("B.merging_flow")
    if any(word in code_text for word in ("walker.pedestrian", "pedestrian", "opposing", "junction")):
        tags.append("B.yielding_priority")
    if any(word in code_text for word in ("collision", "emergency", "brake", "obstacle")):
        tags.append("B.emergency_avoidance")
    if not any(tag.startswith("A.") for tag in tags):
        tags.append("A.sight_distance")
    if not any(tag.startswith("B.") for tag in tags):
        tags.append("B.emergency_avoidance")
    return sorted(dict.fromkeys(tags))


def hazard_center(route):
    if not route:
        return [0.0, 0.0, 0.5]
    return route[len(route) // 2]


def build_metadata(scene_path, excel):
    source = read_source(scene_path)
    route_name, points = route_from_source(source)
    route = route_to_locations(points)
    tags = build_tags(excel, source)
    blueprint = infer_ego_blueprint(source)
    reference_speed = infer_reference_speed(source)
    center = hazard_center(route)
    b_tags = [tag for tag in tags if tag.startswith("B.")]
    a_tags = [tag for tag in tags if tag.startswith("A.")]
    c_tags = [tag for tag in tags if tag.startswith("C.")]
    primary_b = b_tags[0].split(".", 1)[1] if b_tags else "emergency_avoidance"
    primary_a = a_tags[0].split(".", 1)[1] if a_tags else "sight_distance"
    expected = expected_behavior_for_b(tags, "\n".join(str(excel.get(k, "")) for k in excel))
    description = excel.get("description") or (
        f"Original scenario body for {excel.get('original_scene_id')}; detailed variant description is unavailable in the source spreadsheet."
    )

    metadata = {
        "schema_version": "roadtailbench.code_scene_metadata.v3",
        "scenario_id": scene_path.stem,
        "town": scene_path.stem,
        "description": description,
        "metadata_quality": "auto_generated_from_excel_and_script_review_recommended",
        "excel_metadata": excel,
        "scenario_family_id": excel.get("original_scene_id"),
        "scene_type": excel.get("scene_type"),
        "scenario_typical": excel.get("scenario_typical"),
        "environment": excel.get("environment"),
        "source_hazards": excel.get("source_hazards"),
        "ego_role_names": ["ego", "hero"],
        "ego_type_id": blueprint,
        "ego_blueprint": blueprint,
        "ego_start_match_radius_m": 8.0,
        "route_source": route_name or "not_static_route_detected",
        "route": route,
        "route_waypoints": route,
        "centerline_route": route,
        "centerline_source": "ego_trajectory_from_RTB_script" if route else "not_static_route_detected",
        "allowed_lateral_error_m": 2.0,
        "hard_lateral_error_m": 4.0,
        "reference_speed_kmh": reference_speed,
        "scenario_tags": tags,
        "ability_tags": {"A": a_tags, "B": b_tags, "C": c_tags},
        "drivable_polygons": [],
    }
    if route:
        metadata["ego_start"] = {
            "location": {"x": route[0][0], "y": route[0][1], "z": route[0][2]},
            "rotation": {"pitch": 0.0, "yaw": round(yaw_from_point(points[0], points[1] if len(points) > 1 else None), 3), "roll": 0.0},
        }
        metadata["ego_end"] = {
            "location": {"x": route[-1][0], "y": route[-1][1], "z": route[-1][2]},
            "rotation": {"pitch": 0.0, "yaw": round(yaw_from_point(points[-1]), 3), "roll": 0.0},
        }
        metadata["speed_zones"] = [{
            "id": f"{scene_path.stem.lower()}_primary_speed_zone",
            "center": center,
            "radius": 35.0,
            "target_speed_kmh": min(40.0, reference_speed),
            "reason": expected,
            "metadata_quality": "auto_generated_review_recommended",
        }]
        metadata["hazard_zones"] = [{
            "id": f"{scene_path.stem.lower()}_primary_hazard_zone",
            "category": primary_a and "A",
            "subtype": primary_a,
            "behavior_subtype": primary_b,
            "center": center,
            "radius_m": 10.0,
            "target_speed_kmh": min(40.0, reference_speed),
            "metadata_quality": "auto_generated_review_recommended",
        }]
        metadata["hazards"] = [{
            "id": f"{scene_path.stem.lower()}_primary_hazard",
            "type": primary_b,
            "center": center,
            "radius_m": 10.0,
            "perception_radius_m": 40.0,
            "danger_radius_m": 4.0,
            "allow_enter_danger_zone": expected.startswith("yield"),
            "expected_behavior": expected,
            "metadata_quality": "auto_generated_review_recommended",
        }]
    else:
        metadata["notes"] = [
            "No fixed ego route was statically detected; natural end by ego goal is unavailable until metadata is manually refined.",
            "Use scenario-timeout as the termination fallback for this scene.",
        ]
    metadata.setdefault("notes", []).extend([
        "Generated from RoadTailGen spreadsheet columns E/F/G/L and static scene-code parsing.",
        "Hazard centers are route midpoint defaults unless manually reviewed.",
        "Drivable area metric uses centerline corridor until map/lane based evaluation is available.",
    ])
    return metadata


def main():
    excel_rows = load_excel_rows()
    METADATA.mkdir(exist_ok=True)
    scene_ids = sorted(path.stem for path in SCENES.glob("RTB*.py"))
    report = []
    for scene_id in scene_ids:
        scene_path = SCENES / f"{scene_id}.py"
        metadata = build_metadata(scene_path, excel_rows.get(scene_id, {"scenario_id": scene_id}))
        out = METADATA / f"{scene_id}.json"
        out.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        report.append({
            "scenario_id": scene_id,
            "route_points": len(metadata.get("route", [])),
            "route_source": metadata.get("route_source"),
            "tags": metadata.get("scenario_tags"),
            "description_present": bool(metadata.get("description")),
        })
    report_path = ROOT / "outputs" / "metadata_generation_report.json"
    report_path.parent.mkdir(exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    missing_routes = [r for r in report if r["route_points"] < 2]
    print(f"wrote {len(report)} metadata files")
    print(f"missing static routes: {len(missing_routes)}")
    for item in missing_routes[:20]:
        print(item)
    print(f"report: {report_path}")


if __name__ == "__main__":
    main()
