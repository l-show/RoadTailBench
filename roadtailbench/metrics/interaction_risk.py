from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy
from ..core.geometry import clamp, distance2


class InteractionRiskMetric(BaseMetric):
    name = "omnidirectional_interaction_risk"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        danger = float(config.get("interaction_danger_distance_m", 3.0))
        caution = float(config.get("interaction_caution_distance_m", 12.0))
        vals, min_d = [], float("inf")
        for frame in frames:
            pos = location_xy(ego(frame))
            dists = [distance2(pos, tuple(a.get("location", [0.0, 0.0])[:2])) for a in frame.get("actors", [])]
            d = min(dists) if dists else caution
            min_d = min(min_d, d)
            vals.append(clamp((d - danger) / max(caution - danger, 0.1)))
        return MetricResult.make(self.name, sum(vals) / len(vals), {"min_actor_distance_m": min_d})
