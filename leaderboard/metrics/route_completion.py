from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy
from ..core.geometry import clamp, distance2, point_xy, project_point_to_polyline
from ..core.trajectory import reference_xy, trajectory_goal_xy


def _metadata_point(config, key):
    value = config.get(key)
    if not value:
        return None
    try:
        return point_xy(value)
    except (TypeError, ValueError, IndexError):
        return None


class RouteCompletionMetric(BaseMetric):
    name = "route_completion"

    def compute(self, frames, config, context=None):
        runtime_summary = config.get("runtime_summary") or {}
        status = runtime_summary.get("status") or config.get("status")
        termination_reason = runtime_summary.get("termination_reason") or config.get("termination_reason")
        if status == "completed" and termination_reason in {"ego_destroyed", "ego_reached_goal"}:
            return MetricResult.make(self.name, 1.0, {
                "mode": "runtime_terminal_event",
                "status": status,
                "termination_reason": termination_reason,
            })

        route = reference_xy(config)
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        threshold = float(config.get("route_goal_tolerance_m", config.get("natural_end_distance_m", 5.0)))
        goal = trajectory_goal_xy(config)
        if len(route) < 2:
            start = _metadata_point(config, "ego_start") or _metadata_point(config, "ego_spawn") or location_xy(ego(frames[0]))
            if not goal:
                return MetricResult.make(self.name, 0.0, {"reason": "invalid_missing_route_and_goal"})
            initial_distance = max(distance2(start, goal), 0.1)
            min_goal_distance = min(distance2(location_xy(ego(frame)), goal) for frame in frames)
            score = 1.0 if min_goal_distance <= threshold else clamp((initial_distance - min_goal_distance) / initial_distance)
            return MetricResult.make(self.name, score, {
                "mode": "start_goal_fallback",
                "start_to_goal_distance_m": initial_distance,
                "min_goal_distance_m": min_goal_distance,
                "goal_tolerance_m": threshold,
            })
        final_s = 0.0
        total_s, _, _ = project_point_to_polyline(route[-1], route)
        min_goal_distance = float("inf")
        for frame in frames:
            s, _, _ = project_point_to_polyline(location_xy(ego(frame)), route)
            final_s = max(final_s, s)
            if goal:
                min_goal_distance = min(min_goal_distance, distance2(location_xy(ego(frame)), goal))
        projection_score = clamp(final_s / max(total_s, 0.1))
        start_goal_score = None
        if goal:
            start = _metadata_point(config, "ego_start") or _metadata_point(config, "ego_spawn") or route[0]
            initial_distance = max(distance2(start, goal), 0.1)
            start_goal_score = 1.0 if min_goal_distance <= threshold else clamp((initial_distance - min_goal_distance) / initial_distance)
        score = max(projection_score, start_goal_score if start_goal_score is not None else 0.0)
        if goal and min_goal_distance <= threshold:
            score = 1.0
        return MetricResult.make(self.name, score, {
            "mode": "reference_trajectory_projection",
            "projection_score": projection_score,
            "start_goal_fallback_score": start_goal_score,
            "progress_m": final_s,
            "route_length_m": total_s,
            "min_goal_distance_m": None if min_goal_distance == float("inf") else min_goal_distance,
            "goal_tolerance_m": threshold,
        })
