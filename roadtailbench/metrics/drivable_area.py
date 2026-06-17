from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy
from ..core.geometry import clamp, point_xy, project_point_to_polyline


def _segments(config):
    segments = []
    for idx, raw in enumerate(config.get("centerline_segments") or []):
        points = raw.get("points", raw) if isinstance(raw, dict) else raw
        points = [point_xy(p) for p in points]
        if len(points) >= 2:
            segments.append({"id": raw.get("id", f"segment_{idx}") if isinstance(raw, dict) else f"segment_{idx}", "points": points})
    if segments:
        return segments
    points = [point_xy(p) for p in config.get("centerline_route") or config.get("route") or []]
    return [{"id": "centerline_route", "points": points}] if len(points) >= 2 else []


class DrivableAreaMetric(BaseMetric):
    name = "drivable_area"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        segments = _segments(config)
        allowed = float(config.get("allowed_lateral_error_m", 2.0))
        hard = float(config.get("hard_lateral_error_m", max(allowed * 2.0, allowed + 1.0)))
        if not segments:
            return MetricResult.make(self.name, 1.0, {"mode": "centerline_deviation", "reason": "missing_centerline", "used_polygon": False})
        scores, deviations, counts = [], [], {}
        for frame in frames:
            pos = location_xy(ego(frame))
            best_id, best_d = "unknown", float("inf")
            for seg in segments:
                _, d, _ = project_point_to_polyline(pos, seg["points"])
                if d < best_d:
                    best_id, best_d = seg["id"], d
            violation = max(0.0, best_d - allowed)
            scores.append(1.0 - clamp(violation / max(hard - allowed, 0.1)))
            deviations.append(best_d)
            counts[best_id] = counts.get(best_id, 0) + 1
        return MetricResult.make(self.name, sum(scores) / len(scores), {
            "mode": "centerline_deviation",
            "used_polygon": False,
            "max_centerline_deviation_m": max(deviations),
            "mean_centerline_deviation_m": sum(deviations) / len(deviations),
            "allowed_lateral_error_m": allowed,
            "hard_lateral_error_m": hard,
            "selected_segment_counts": counts,
        })
