import ast
import json
import re
import shutil
from pathlib import Path

try:
    from generate_metadata import load_rows_and_taxonomy
except ImportError:  # pragma: no cover
    from scripts.generate_metadata import load_rows_and_taxonomy


ROOT = Path(__file__).resolve().parents[1]
SCENARIO_SOURCE = ROOT / "scenarios"
SCENE_EGO = ROOT / "scene_ego"
AGENT_EGO = ROOT / "agent_ego"
OUTPUTS = ROOT / "outputs"


AGENT_HELPER = r'''


def _rtb_agent_is_alive(actor):
    return bool(actor is not None and getattr(actor, "is_alive", False))


def _rtb_agent_find_ego(world, role_names=("ego", "hero"), type_id=None, start_xy=None, radius_m=12.0):
    try:
        actors = list(world.get_actors().filter("vehicle.*"))
    except Exception:
        return None
    for actor in actors:
        try:
            if actor.attributes.get("role_name", "") in role_names:
                return actor
        except Exception:
            pass
    candidates = actors
    if type_id:
        candidates = [actor for actor in candidates if getattr(actor, "type_id", "") == type_id]
    if start_xy and candidates:
        sx, sy = start_xy
        def dist(actor):
            loc = actor.get_location()
            return ((loc.x - sx) ** 2 + (loc.y - sy) ** 2) ** 0.5
        candidates = sorted(candidates, key=dist)
        try:
            if dist(candidates[0]) <= radius_m:
                return candidates[0]
        except Exception:
            return None
    return candidates[0] if len(candidates) == 1 else None
'''


SPAWN_NAMES = {
    "spawn_vehicle",
    "spawn_vehicle_at_xy",
    "spawn_vehicle_by_tf",
    "safe_spawn_vehicle",
    "force_spawn_vehicle",
    "try_spawn_actor",
    "try_spawn_with_debug",
    "spawn_actor",
}

VISUAL_DEBUG_FUNCTIONS = {
    "draw_spawn_debug",
    "draw_path_debug",
    "draw_debug_path",
    "draw_debug_target",
    "draw_preset_trajectory",
    "draw_lookahead_point",
    "safe_draw_preset_trajectory",
    "safe_draw_lookahead_point",
    "build_lane_debug_path",
}

VISUAL_DEBUG_CALL_RE = re.compile(
    r"\b(?:RTB\.)?draw_(?:preset_trajectory|lookahead_point)\s*\("
    r"|\bworld\.debug\.draw_(?:point|string|line|arrow|box)\s*\("
    r"|\b(?:draw_spawn_debug|draw_path_debug|draw_debug_path|draw_debug_target|safe_draw_preset_trajectory|safe_draw_lookahead_point|build_lane_debug_path)\s*\(",
    re.I,
)

VISUAL_DEBUG_ASSIGN_RE = re.compile(r"\bDRAW_[A-Z0-9_]*DEBUG[A-Z0-9_]*\b")
VISUAL_DEBUG_TEST_RE = re.compile(r"\bDRAW_[A-Z0-9_]*DEBUG[A-Z0-9_]*\b|\bdraw_debug\b", re.I)
VISUAL_DEBUG_TEXT_RE = re.compile(
    r"Debug\s*(?:&|可视化)|debug\s*(?:box|可视化|展示|绘制)|调试绘制|可视化|视觉参考|"
    r"画出.*轨迹|绘制.*轨迹|绘制.*预瞄|绘制.*牵引|绘制.*锚点|"
    r"绘制.*生成点|绘制.*投影点|预绘制|直观检查|方便调试观察|方便开发者观察",
    re.I,
)


def read_source(path):
    return path.read_text(encoding="utf-8-sig", errors="replace").replace("main()s", "main()")


def write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="")


def call_name(node):
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Attribute):
            return func.attr
        if isinstance(func, ast.Name):
            return func.id
    return ""


def source_segment(source, node):
    return ast.get_source_segment(source, node) or ""


def target_names(node):
    if not isinstance(node, ast.Assign):
        return []
    names = []
    for target in node.targets:
        if isinstance(target, ast.Name):
            names.append(target.id)
    return names


def call_has_role_name_ego(call_text):
    return bool(re.search(r"role_name\s*=\s*['\"](?:ego|hero)['\"]", call_text, re.I))


def assignment_sets_role_ego(source, node):
    text = source_segment(source, node)
    return bool(re.search(r"\.set_attribute\s*\(\s*['\"]role_name['\"]\s*,\s*['\"](?:ego|hero)['\"]", text, re.I))


def numeric_constants(node):
    values = []
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, (int, float)):
            values.append(float(child.value))
        elif isinstance(child, ast.UnaryOp) and isinstance(child.op, ast.USub) and isinstance(child.operand, ast.Constant):
            if isinstance(child.operand.value, (int, float)):
                values.append(-float(child.operand.value))
    return values


def near_start(node, row, tolerance=4.0):
    start = row.get("ego_start")
    if not start:
        return False
    sx, sy = float(start[0]), float(start[1])
    nums = numeric_constants(node)
    for left, right in zip(nums, nums[1:]):
        if abs(left - sx) <= tolerance and abs(right - sy) <= tolerance:
            return True
    return False


def string_constants(node):
    values = []
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, str):
            values.append(child.value)
    return values


def collect_ego_spawn_info(source, row):
    tree = ast.parse(source)
    blueprint_vars = set()
    transform_vars = set()
    ego_vars = set()
    line_ranges = []
    ego_blueprint = row.get("ego_model")

    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and assignment_sets_role_ego(source, node):
            expr = node.value
            if isinstance(expr, ast.Call) and isinstance(expr.func, ast.Attribute):
                owner = expr.func.value
                if isinstance(owner, ast.Name):
                    blueprint_vars.add(owner.id)
                    line_ranges.append((node.lineno, getattr(node, "end_lineno", node.lineno), "set ego role_name"))
        if isinstance(node, ast.Assign):
            names = target_names(node)
            if not names:
                continue
            text = source_segment(source, node)
            if call_name(node.value) in {"find", "filter"} and ego_blueprint and ego_blueprint in text:
                for name in names:
                    blueprint_vars.add(name)
            if near_start(node, row) and any("transform" in name.lower() or "loc" in name.lower() for name in names):
                transform_vars.update(names)

    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        names = target_names(node)
        if not names or not isinstance(node.value, ast.Call):
            continue
        name = call_name(node.value)
        if name not in SPAWN_NAMES:
            continue
        text = source_segment(source, node)
        args_text = " ".join(string_constants(node.value))
        is_ego = (
            call_has_role_name_ego(text)
            or any("ego" in var.lower() for var in names)
            or any(var in text for var in blueprint_vars)
            or any(var in text for var in transform_vars)
            or (ego_blueprint and ego_blueprint in args_text and near_start(node, row))
        )
        if is_ego:
            ego_vars.update(names)
            line_ranges.append((node.lineno, getattr(node, "end_lineno", node.lineno), "scene-side ego spawn"))
    return tree, sorted(ego_vars), line_ranges


def remove_vehicle_config_ego(source):
    pattern = re.compile(
        r"\n\s*\{[^\{\}]*?['\"]id['\"]\s*:\s*['\"]ego['\"][^\{\}]*?\}\s*,?",
        re.S,
    )
    return pattern.sub("", source)


def remove_opt_guard(source):
    source = re.sub(
        r"\n# === RoadTailBench Opt: ego endpoint cleanup guard ===.*?# === End RoadTailBench Opt guard ===\n",
        "\n",
        source,
        flags=re.S,
    )
    source = re.sub(
        r"\n# === RoadTailBench Opt: ego endpoint cleanup guard ===.*?(?=\ndef main\(\):)",
        "\n",
        source,
        flags=re.S,
    )
    source = re.sub(r"\n\s*if _rtb_opt_goal_guard\(locals\(\), client, world\):\n\s*break\n", "\n", source)
    source = re.sub(r"\n\s*if _rtb_opt_goal_guard\(locals\(\), client, world\):\n\s*return\n", "\n", source)
    return source


def overlaps_removed(node, removed):
    if not hasattr(node, "lineno"):
        return False
    start, end = node.lineno, getattr(node, "end_lineno", node.lineno)
    return any(not (end < left or start > right) for left, right, _ in removed)


def statement_mentions(text, names):
    return any(re.search(rf"\b{re.escape(name)}\b", text) for name in names)


def collect_agent_removals(source, row, ego_vars, initial_removed):
    tree = ast.parse(source)
    removed = list(initial_removed)
    extra_control_vars = set()
    all_ego_vars = set(ego_vars)

    for node in ast.walk(tree):
        if overlaps_removed(node, removed):
            continue
        if isinstance(node, ast.Assign):
            names = target_names(node)
            text = source_segment(source, node)
            lower_names = [name.lower() for name in names]
            if any("ego" in name for name in lower_names) and (
                "PID" in text or "VehicleLightManager" in text or "MultiStageBehaviorMachine" in text
            ):
                extra_control_vars.update(names)
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "ego controller setup"))
            elif names and statement_mentions(text, all_ego_vars) and (
                "PID" in text or "VehicleLightManager" in text or "MultiStageBehaviorMachine" in text
            ):
                extra_control_vars.update(names)
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "ego controller setup"))
    control_vars = extra_control_vars | {name for name in extra_control_vars if "ego" in name.lower()}

    for node in ast.walk(tree):
        if overlaps_removed(node, removed):
            continue
        text = source_segment(source, node)
        if isinstance(node, ast.Expr):
            if statement_mentions(text, control_vars) or (
                statement_mentions(text, all_ego_vars)
                and any(token in text for token in ("set_vehicle_initial_speed", "set_initial_velocity", "set_target_velocity", "apply_control", "set_light_state"))
            ):
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "ego setup/control call"))
        elif isinstance(node, ast.Assign):
            names = target_names(node)
            if any("ego" in name.lower() for name in names) and any(
                token in text for token in ("tick(", "get_target_waypoint", "get_location", "get_velocity")
            ):
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "ego runtime state"))
        elif isinstance(node, ast.If):
            body = "\n".join(source_segment(source, item) for item in node.body)
            if statement_mentions(source_segment(source, node.test), all_ego_vars | {"ego_active"}) and any(
                token in body for token in ("apply_pid_control", "set_target_velocity", "apply_control", "get_target_waypoint", "check_vehicle_out_of_bounds")
            ):
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "ego control block"))
            elif statement_mentions(source_segment(source, node.test), all_ego_vars) and "active_pid_vehicles.append" in body:
                removed.append((node.lineno, getattr(node, "end_lineno", node.lineno), "external ego PID registration"))

    for lineno, line in enumerate(source.splitlines(), start=1):
        if any(left <= lineno <= right for left, right, _ in removed):
            continue
        if any(re.search(rf"\bactor_list\.append\(\s*{re.escape(name)}\s*\)", line) for name in all_ego_vars):
            removed.append((lineno, lineno, "do not register external ego for scene cleanup"))
        elif re.search(r"spawn_vehicle\s*\(\s*ego_config\b", line) or re.search(r"\bvehicles_config\s*=\s*\[[^\]]*\bego_config\b", line):
            removed.append((lineno, lineno, "skip configured scene-side ego"))
    return removed, sorted(control_vars)


def line_start_offsets(source):
    offsets = []
    total = 0
    for line in source.splitlines(keepends=True):
        offsets.append(total)
        total += len(line)
    offsets.append(total)
    return offsets


def replace_ranges(source, replacements):
    lines = source.splitlines(keepends=True)
    ranges = {}
    for start, end, replacement in replacements:
        ranges.setdefault((start, end), replacement)
    output = []
    line = 1
    max_line = len(lines)
    while line <= max_line:
        match = None
        for start, end in ranges:
            if start == line:
                match = (start, end, ranges[(start, end)])
                break
        if match:
            output.append(match[2])
            line = match[1] + 1
        else:
            output.append(lines[line - 1])
            line += 1
    return "".join(output)


def fill_empty_blocks(source):
    lines = source.splitlines()
    out = []
    block_headers = re.compile(r"^(\s*)(if|elif|else|for|while|try|except|finally)\b.*:\s*(?:#.*)?$")
    for index, line in enumerate(lines):
        out.append(line)
        match = block_headers.match(line)
        if not match:
            continue
        indent = len(match.group(1))
        child_indent = indent + 4
        has_body = False
        cursor = index + 1
        while cursor < len(lines):
            candidate = lines[cursor]
            stripped = candidate.strip()
            if not stripped or stripped.startswith("#"):
                cursor += 1
                continue
            current_indent = len(candidate) - len(candidate.lstrip())
            has_body = current_indent > indent
            break
        if not has_body:
            out.append(" " * child_indent + "pass")
    return "\n".join(out) + ("\n" if source.endswith("\n") else "")


def collect_visual_debug_removals(source):
    tree = ast.parse(source)
    removed = []
    for node in ast.walk(tree):
        if not hasattr(node, "lineno"):
            continue
        start = node.lineno
        end = getattr(node, "end_lineno", node.lineno)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in VISUAL_DEBUG_FUNCTIONS:
            removed.append((start, end, f"remove visual helper {node.name}"))
            continue
        if isinstance(node, ast.Assign):
            text = source_segment(source, node)
            if VISUAL_DEBUG_ASSIGN_RE.search(text):
                removed.append((start, end, "remove visual debug flag"))
                continue
        if isinstance(node, ast.If):
            test_text = source_segment(source, node.test)
            body_text = "\n".join(source_segment(source, item) for item in node.body)
            if VISUAL_DEBUG_TEST_RE.search(test_text) and VISUAL_DEBUG_CALL_RE.search(body_text):
                removed.append((start, end, "remove visual debug block"))
                continue
        if isinstance(node, (ast.Expr, ast.Assign)):
            text = source_segment(source, node)
            if VISUAL_DEBUG_CALL_RE.search(text):
                removed.append((start, end, "remove visual debug call"))
    removed.sort()
    merged = []
    for start, end, reason in removed:
        if merged and start <= merged[-1][1] + 1:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end), merged[-1][2])
        else:
            merged.append((start, end, reason))
    return merged


def remove_visual_debug(source):
    source = fill_empty_blocks(source)
    removals = collect_visual_debug_removals(source)
    if removals:
        source = replace_ranges(source, [(start, end, "") for start, end, _ in removals])
    source = source.replace("try_spawn_with_debug", "try_spawn_with_fallback")
    source = re.sub(
        r"def spawn_friction_region\(world, bp_lib, center_loc, friction=0\.0, extent=\(10\.0, 10\.0, 10\.0\), draw_debug=False, debug_life=100\.0\):",
        "def spawn_friction_region(world, bp_lib, center_loc, friction=0.0, extent=(10.0, 10.0, 10.0)):",
        source,
    )
    source = re.sub(r",\s*draw_debug=(?:True|False)", "", source)
    source = re.sub(r",\s*debug_life=[^,\n)]+", "", source)
    source = re.sub(r"(?m)^\s*draw_debug=(?:True|False),?\s*(?:#.*)?\n", "", source)
    source = re.sub(r"(?m)^\s*debug_life=[^,\n)]+,?\s*(?:#.*)?\n", "", source)
    source = re.sub(
        r"\n\s*if friction_trigger and draw_debug:\n(?:\s{8}.*\n)+",
        "\n",
        source,
    )
    source = re.sub(
        r"(?m)^\s*#.*(?:draw_preset_trajectory|draw_lookahead_point|draw_spawn_debug|draw_path_debug|draw_debug_path|draw_debug_target|safe_draw_preset_trajectory|safe_draw_lookahead_point|world\.debug\.draw_|draw_debug|debug_life).*\n?",
        "",
        source,
    )
    lines = []
    for line in source.splitlines():
        stripped = line.strip()
        if VISUAL_DEBUG_TEXT_RE.search(line) and (
            stripped.startswith("#")
            or stripped.startswith("print(")
            or stripped.startswith("-")
            or re.match(r"^\d+\.", stripped)
            or "画面 debug 展示" in stripped
        ):
            continue
        lines.append(line)
    source = "\n".join(lines) + ("\n" if source.endswith("\n") else "")
    source = re.sub(r"\n{3,}", "\n\n", source)
    return fill_empty_blocks(source)


def sync_scene_ego(rows):
    SCENE_EGO.mkdir(exist_ok=True)
    for stale in SCENE_EGO.glob("RTB*.py"):
        stale.unlink()
    helper = SCENARIO_SOURCE / "RoadTailBenchInitV9.py"
    if helper.exists():
        write_text(SCENE_EGO / helper.name, remove_visual_debug(read_source(helper)))
    readme = SCENARIO_SOURCE / "README.md"
    if readme.exists():
        shutil.copy2(readme, SCENE_EGO / "README.md")
    for scenario_id in sorted(rows):
        src = SCENARIO_SOURCE / f"{scenario_id}.py"
        text = remove_visual_debug(read_source(src))
        write_text(SCENE_EGO / f"{scenario_id}_scene_ego.py", text)


def agent_helper_for(row):
    start = row.get("ego_start") or [None, None]
    return (
        AGENT_HELPER
        + f"\n_RTB_AGENT_EGO_TYPE_ID = {row.get('ego_model')!r}\n"
        + f"_RTB_AGENT_EGO_START_XY = ({float(start[0]) if start[0] is not None else 0.0}, {float(start[1]) if start[1] is not None else 0.0})\n"
    )


def insert_agent_helper(source, row):
    if "_rtb_agent_find_ego" in source:
        return source
    marker = "\ndef main():"
    helper = agent_helper_for(row)
    if marker in source:
        return source.replace(marker, helper + marker, 1)
    return source + helper


def build_agent_source(source, row):
    source = remove_vehicle_config_ego(remove_opt_guard(source))
    tree, ego_vars, spawn_ranges = collect_ego_spawn_info(source, row)
    if not ego_vars:
        tree = ast.parse(source)
        ego_vars = sorted({
            name.id for name in ast.walk(tree)
            if isinstance(name, ast.Name) and "ego" in name.id.lower() and not name.id.startswith("_RTB_")
        })
    removals, control_vars = collect_agent_removals(source, row, ego_vars, spawn_ranges)
    replacements = []
    for start, end, reason in removals:
        if reason == "scene-side ego spawn":
            line = " " * (len(source.splitlines()[start - 1]) - len(source.splitlines()[start - 1].lstrip()))
            name = ego_vars[0] if ego_vars else "ego"
            replacements.append((
                start,
                end,
                f"{line}{name} = _rtb_agent_find_ego(world, type_id=_RTB_AGENT_EGO_TYPE_ID, start_xy=_RTB_AGENT_EGO_START_XY)  # scene-side ego spawn removed\n",
            ))
        else:
            replacements.append((start, end, ""))
    source = replace_ranges(source, replacements)
    source = remove_visual_debug(source)
    source = insert_agent_helper(source, row)
    return source, {
        "ego_vars": ego_vars,
        "control_vars": control_vars,
        "removed_ranges": len(removals),
    }


def sync_agent_ego(rows):
    AGENT_EGO.mkdir(exist_ok=True)
    for stale in AGENT_EGO.glob("RTB*.py"):
        stale.unlink()
    helper = SCENARIO_SOURCE / "RoadTailBenchInitV9.py"
    if helper.exists():
        write_text(AGENT_EGO / helper.name, remove_visual_debug(read_source(helper)))
    report = []
    for scenario_id in sorted(rows):
        src = SCENARIO_SOURCE / f"{scenario_id}.py"
        source = read_source(src)
        agent_source, info = build_agent_source(source, rows[scenario_id])
        dst = AGENT_EGO / f"{scenario_id}_agent_ego.py"
        write_text(dst, agent_source)
        report.append({"scenario_id": scenario_id, **info})
    write_text(AGENT_EGO / "README.md", (
        "# RoadTailBench Agent Ego Scenes\n\n"
        "These files are generated from `../scenarios` by `scripts/sync_scene_variants.py`.\n"
        "Scene-side ego spawn and PID/control code is removed; the runner or an external agent is expected to spawn `role_name=\"ego\"`.\n"
    ))
    OUTPUTS.mkdir(exist_ok=True)
    write_text(OUTPUTS / "agent_ego_generation_report.json", json.dumps(report, ensure_ascii=False, indent=2) + "\n")


def main():
    rows, _taxonomy = load_rows_and_taxonomy()
    if len(rows) != 100:
        raise RuntimeError(f"expected 100 final workbook scenarios, found {len(rows)}")
    sync_scene_ego(rows)
    sync_agent_ego(rows)
    print(f"synced {len(rows)} scene_ego and agent_ego scenario files")


if __name__ == "__main__":
    main()
