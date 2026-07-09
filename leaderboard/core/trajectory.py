import math

from .geometry import angle_delta_deg, distance2, point_xy, polyline_lengths, sample_polyline_at_s


def _parse_trajectory_text(raw):
    rows = []
    for line in raw.splitlines():
        clean = line.strip().replace(",", " ")
        if not clean or clean.startswith("#"):
            continue
        parts = clean.split()
        if len(parts) < 2:
            continue
        try:
            rows.append([float(part) for part in parts])
        except ValueError:
            continue
    return rows


def _yaw_index(fmt):
    tokens = [part for part in str(fmt).lower().split("_") if part]
    if "yaw" not in tokens:
        return None
    return tokens.index("yaw") if len(tokens) > 1 else 2


def normalize_reference_trajectory(config):
    source = "reference_trajectory"
    raw = config.get("reference_trajectory") or config.get("ego_reference_trajectory")
    if not raw:
        source = "legacy_route"
        raw = config.get("route") or config.get("centerline_route") or []
    fmt = str(config.get("reference_trajectory_format", "x_y_yaw" if source == "reference_trajectory" else "x_y"))
    yaw_index = _yaw_index(fmt) if source == "reference_trajectory" else None
    if isinstance(raw, str):
        raw = _parse_trajectory_text(raw)
    points = []
    for item in raw:
        if isinstance(item, dict):
            loc = item.get("location", item)
            rot = item.get("rotation", {})
            if isinstance(loc, dict):
                x, y = loc.get("x", 0.0), loc.get("y", 0.0)
            else:
                x, y = loc[0], loc[1]
            yaw = item.get("yaw", rot.get("yaw")) if isinstance(rot, dict) else item.get("yaw")
        else:
            if len(item) < 2:
                continue
            x, y = item[0], item[1]
            yaw = item[yaw_index] if yaw_index is not None and len(item) > yaw_index else None
        point = {"x": float(x), "y": float(y)}
        if yaw is not None:
            point["yaw"] = float(yaw)
        points.append(point)
    return points


def reference_xy(config):
    return [(p["x"], p["y"]) for p in normalize_reference_trajectory(config)]


def reference_yaw_at(points, segment_index):
    if not points:
        return None
    idx = min(max(int(segment_index), 0), len(points) - 1)
    if "yaw" in points[idx]:
        return points[idx]["yaw"]
    if idx + 1 < len(points):
        a, b = points[idx], points[idx + 1]
        return math.degrees(math.atan2(b["y"] - a["y"], b["x"] - a["x"]))
    if idx > 0:
        a, b = points[idx - 1], points[idx]
        return math.degrees(math.atan2(b["y"] - a["y"], b["x"] - a["x"]))
    return None


def trajectory_goal_xy(config):
    for key in ("ego_end", "ego_goal"):
        loc = config.get(key)
        if loc:
            try:
                return point_xy(loc)
            except (TypeError, ValueError, IndexError):
                pass
    points = reference_xy(config)
    return points[-1] if points else None


def estimate_reference_duration_s(points, reference_speed_kmh):
    if len(points) < 2:
        return 0.0
    distances = polyline_lengths([(p["x"], p["y"]) for p in points])
    speed_mps = max(float(reference_speed_kmh or 50.0) / 3.6, 0.1)
    return distances[-1] / speed_mps


def nearest_reference_summary(config, xy):
    points = normalize_reference_trajectory(config)
    route = [(p["x"], p["y"]) for p in points]
    if len(route) < 2:
        return None
    from .geometry import project_point_to_polyline

    s, lateral, seg = project_point_to_polyline(xy, route)
    distances = polyline_lengths(route)
    sample_xy, sample_seg = sample_polyline_at_s(route, distances, s)
    return {
        "progress_m": s,
        "lateral_m": lateral,
        "segment_index": seg,
        "sample_xy": sample_xy,
        "sample_segment_index": sample_seg,
        "route_length_m": distances[-1],
        "heading_yaw": reference_yaw_at(points, seg),
        "distance_to_goal_m": distance2(xy, route[-1]),
    }


def heading_error_deg(actual_yaw, reference_yaw):
    if actual_yaw is None or reference_yaw is None:
        return None
    return angle_delta_deg(actual_yaw, reference_yaw)
