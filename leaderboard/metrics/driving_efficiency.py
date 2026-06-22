from .base import BaseMetric, MetricResult
from ..core.geometry import clamp, polyline_lengths
from ..core.trajectory import reference_xy


class DrivingEfficiencyMetric(BaseMetric):
    name = "driving_efficiency"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        route = reference_xy(config)
        if len(route) < 2:
            return MetricResult.make(self.name, 1.0, {"reason": "missing_route"})

        times = [float(frame.get("time", 0.0)) for frame in frames]
        elapsed = max(0.0, times[-1] - times[0])
        route_length = polyline_lengths(route)[-1]
        speed_kmh = float(config.get("speed_limit_kmh", config.get("reference_speed_kmh", 50.0)))
        expected = route_length / max(speed_kmh / 3.6, 0.1)
        route_score = float((context or {}).get("route_completion", {}).get("score", 1.0))

        if elapsed <= expected:
            time_score = 1.0
        else:
            hard_ratio = float(config.get("efficiency_hard_time_ratio", 2.0))
            ratio = elapsed / max(expected, 0.1)
            time_score = 1.0 - clamp((ratio - 1.0) / max(hard_ratio - 1.0, 0.1))
        score = clamp(time_score * route_score)
        return MetricResult.make(self.name, score, {
            "mode": "simulation_time_to_goal",
            "elapsed_sim_seconds": elapsed,
            "expected_duration_s": expected,
            "route_length_m": route_length,
            "speed_limit_kmh": speed_kmh,
            "route_completion_gate": route_score,
        })
