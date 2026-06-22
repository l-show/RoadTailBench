from .base import BaseMetric, MetricResult
from ..core.extractors import control, ego
from ..core.geometry import clamp


class ControlStabilityMetric(BaseMetric):
    name = "control_stability"

    def compute(self, frames, config, context=None):
        if len(frames) < 2:
            return MetricResult.make(self.name, 1.0, {"reason": "too_few_frames"})
        steer_limit = float(config.get("steer_delta_limit", 0.20))
        throttle_limit = float(config.get("throttle_delta_limit", 0.25))
        brake_limit = float(config.get("brake_delta_limit", 0.25))
        vals = []
        max_steer = max_throttle = max_brake = 0.0
        prev = control(ego(frames[0]))
        for frame in frames[1:]:
            cur = control(ego(frame))
            steer_delta = abs(float(cur.get("steer", 0.0)) - float(prev.get("steer", 0.0)))
            throttle_delta = abs(float(cur.get("throttle", 0.0)) - float(prev.get("throttle", 0.0)))
            brake_delta = abs(float(cur.get("brake", 0.0)) - float(prev.get("brake", 0.0)))
            max_steer = max(max_steer, steer_delta)
            max_throttle = max(max_throttle, throttle_delta)
            max_brake = max(max_brake, brake_delta)
            steer_score = 1.0 - clamp(max(0.0, steer_delta - steer_limit) / max(steer_limit, 0.01))
            throttle_score = 1.0 - clamp(max(0.0, throttle_delta - throttle_limit) / max(throttle_limit, 0.01))
            brake_score = 1.0 - clamp(max(0.0, brake_delta - brake_limit) / max(brake_limit, 0.01))
            vals.append(0.45 * steer_score + 0.30 * throttle_score + 0.25 * brake_score)
            prev = cur
        return MetricResult.make(self.name, sum(vals) / len(vals), {
            "steer_delta_limit": steer_limit,
            "throttle_delta_limit": throttle_limit,
            "brake_delta_limit": brake_limit,
            "max_steer_delta": max_steer,
            "max_throttle_delta": max_throttle,
            "max_brake_delta": max_brake,
        })
