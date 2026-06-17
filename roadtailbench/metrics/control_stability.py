from .base import BaseMetric, MetricResult
from ..core.extractors import control, ego
from ..core.geometry import clamp


class ControlStabilityMetric(BaseMetric):
    name = "control_stability"

    def compute(self, frames, config, context=None):
        if len(frames) < 2:
            return MetricResult.make(self.name, 1.0, {"reason": "too_few_frames"})
        steer_limit = float(config.get("steer_delta_limit", 0.20))
        vals = []
        prev = control(ego(frames[0]))
        for frame in frames[1:]:
            cur = control(ego(frame))
            delta = abs(float(cur.get("steer", 0.0)) - float(prev.get("steer", 0.0)))
            vals.append(1.0 - clamp(max(0.0, delta - steer_limit) / max(steer_limit, 0.01)))
            prev = cur
        return MetricResult.make(self.name, sum(vals) / len(vals), {"steer_delta_limit": steer_limit})
