from .base import BaseMetric, MetricResult
from ..core.extractors import ego, speed_mps
from ..core.geometry import clamp


class DrivingEfficiencyMetric(BaseMetric):
    name = "driving_efficiency"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        ref = max(float(config.get("reference_speed_kmh", 50.0)) / 3.6, 0.1)
        vals = [clamp(speed_mps(ego(f)) / ref) for f in frames]
        return MetricResult.make(self.name, sum(vals) / len(vals), {"reference_speed_kmh": ref * 3.6})
