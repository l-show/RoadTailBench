from .base import BaseMetric, MetricResult
from .speed_appropriateness import target_speed_for_frame
from ..core.extractors import ego, location_xy, speed_mps
from ..core.geometry import clamp, distance2


A_SUBTYPES = {
    "traffic_sign_marking": "Traffic Sign & Marking Robustness",
    "separation_protection": "Separation & Protection Robustness",
    "speed_control_facility": "Speed-Control Facility Robustness",
    "lighting_facility": "Lighting Facility Robustness",
    "pavement_condition": "Pavement Condition Robustness",
    "alignment_geometry": "Alignment Geometry Robustness",
    "sight_distance": "Sight-Distance Robustness",
    "clearance_intrusion": "Clearance Intrusion Robustness",
}
B_SUBTYPES = {
    "overtaking_bypass": "Overtaking & Obstacle Bypassing",
    "merging_flow": "Merging & Flow Negotiation",
    "emergency_avoidance": "Emergency Avoidance",
    "yielding_priority": "Yielding & Priority Negotiation",
}
C_SUBTYPES = {
    "low_light": "Low-Light Robustness",
    "glare": "Glare Robustness",
    "fog": "Fog Robustness",
    "rain_wet": "Rain & Wet-Road Robustness",
    "snow_low_friction": "Snow or Low-Friction Robustness",
    "wind_dust_visibility": "Wind/Dust Visibility Robustness",
}


class RoadEngineeringHazardAdaptationMetric(BaseMetric):
    name = "road_engineering_hazard_adaptation"

    def compute(self, frames, config, context=None):
        context = context or {}
        zones = config.get("hazard_zones", [])
        if not zones:
            return MetricResult.make(self.name, 1.0, {"reason": "no_hazard_zones"})
        scores = []
        details = []
        for zone in zones:
            center = tuple(zone.get("center", [0.0, 0.0])[:2])
            radius = float(zone.get("radius_m", zone.get("radius", 10.0)))
            local = [f for f in frames if distance2(location_xy(ego(f)), center) <= radius]
            if not local:
                zone_score = 1.0
            else:
                speed_vals = []
                for frame in local:
                    target = float(zone.get("target_speed_kmh", target_speed_for_frame(frame, config) * 3.6)) / 3.6
                    speed_vals.append(1.0 - clamp(abs(speed_mps(ego(frame)) - max(target, 0.1)) / max(target, 0.1)))
                drivable = float(context.get("drivable_area", {}).get("score", 1.0))
                interaction = float(context.get("omnidirectional_interaction_risk", {}).get("score", 1.0))
                collision = float(context.get("collision_penalty", {}).get("score", 1.0))
                zone_score = clamp(0.35 * collision + 0.25 * drivable + 0.20 * interaction + 0.20 * (sum(speed_vals) / len(speed_vals)))
            scores.append(zone_score)
            details.append({"id": zone.get("id"), "subtype": zone.get("subtype"), "score": zone_score})
        return MetricResult.make(self.name, sum(scores) / len(scores), {"zone_scores": details})
