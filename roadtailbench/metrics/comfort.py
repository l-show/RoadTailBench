from .base import BaseMetric, MetricResult
from ..core.extractors import acceleration_xy, ego
from ..core.geometry import clamp, norm2


class ComfortMetric(BaseMetric):
    name = "comfort"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        limit = float(config.get("comfort_accel_limit_mps2", 4.0))
        vals = [1.0 - clamp(max(0.0, norm2(acceleration_xy(ego(f))) - limit) / limit) for f in frames]
        return MetricResult.make(self.name, sum(vals) / len(vals), {"accel_limit_mps2": limit})
