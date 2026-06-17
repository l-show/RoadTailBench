from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy
from ..core.geometry import clamp, point_xy, project_point_to_polyline


class RouteCompletionMetric(BaseMetric):
    name = "route_completion"

    def compute(self, frames, config, context=None):
        route = [point_xy(p) for p in config.get("route") or config.get("centerline_route") or []]
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        if len(route) < 2:
            return MetricResult.make(self.name, 1.0, {"reason": "missing_route"})
        final_s = 0.0
        total_s, _, _ = project_point_to_polyline(route[-1], route)
        for frame in frames:
            s, _, _ = project_point_to_polyline(location_xy(ego(frame)), route)
            final_s = max(final_s, s)
        score = clamp(final_s / max(total_s, 0.1))
        return MetricResult.make(self.name, score, {"progress_m": final_s, "route_length_m": total_s})
