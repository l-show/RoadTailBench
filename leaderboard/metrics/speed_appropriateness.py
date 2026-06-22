from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy, speed_mps
from ..core.geometry import clamp, distance2


def active_hazard_speed(frame, config):
    pos = location_xy(ego(frame))
    for hazard in config.get("hazards", []):
        center = tuple(hazard.get("center", [0.0, 0.0])[:2])
        radius = float(hazard.get("radius_m", hazard.get("radius", 0.0)))
        if radius > 0 and distance2(pos, center) <= radius:
            speed = hazard.get("reference_speed_kmh", hazard.get("target_speed_kmh"))
            if speed is not None:
                return float(speed) / 3.6, hazard.get("id")
    for zone in config.get("speed_zones", []):
        center = tuple(zone.get("center", [0.0, 0.0])[:2])
        radius = float(zone.get("radius", zone.get("radius_m", 0.0)))
        if radius > 0 and distance2(pos, center) <= radius:
            return float(zone.get("target_speed_kmh", zone.get("reference_speed_kmh", config.get("speed_limit_kmh", 50.0)))) / 3.6, zone.get("id")
    return None, None


def target_speed_for_frame(frame, config):
    hazard_speed, _ = active_hazard_speed(frame, config)
    if hazard_speed is not None:
        return max(hazard_speed, 0.1)
    return max(float(config.get("speed_limit_kmh", config.get("reference_speed_kmh", 50.0))) / 3.6, 0.1)


class SpeedAppropriatenessMetric(BaseMetric):
    name = "speed_appropriateness"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        vals = []
        hazard_frames = 0
        speed_limit_kmh = float(config.get("speed_limit_kmh", config.get("reference_speed_kmh", 50.0)))
        tolerance = float(config.get("speed_limit_tolerance_kmh", 3.0)) / 3.6
        hazard_tolerance_ratio = float(config.get("hazard_speed_tolerance_ratio", 0.25))
        for frame in frames:
            hazard_speed, hazard_id = active_hazard_speed(frame, config)
            speed = speed_mps(ego(frame))
            if hazard_speed is not None:
                hazard_frames += 1
                allowed = hazard_speed * (1.0 + hazard_tolerance_ratio)
                hard = max(allowed + hazard_speed, allowed + 0.1)
                vals.append(1.0 - clamp(max(0.0, speed - allowed) / max(hard - allowed, 0.1)))
            else:
                limit = speed_limit_kmh / 3.6
                vals.append(1.0 - clamp(max(0.0, speed - limit - tolerance) / max(limit, 0.1)))
        return MetricResult.make(self.name, sum(vals) / len(vals), {
            "mode": "speed_limit_and_hazard_reference",
            "speed_limit_kmh": speed_limit_kmh,
            "speed_limit_tolerance_kmh": tolerance * 3.6,
            "hazard_speed_tolerance_ratio": hazard_tolerance_ratio,
            "hazard_frame_ratio": hazard_frames / len(frames),
            "mean_frame_score": sum(vals) / len(vals),
        })
