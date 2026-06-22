from .base import BaseMetric, MetricResult
from ..core.geometry import clamp


BEHAVIOR_CAPABILITIES = [
    "lane_change",
    "overtaking",
    "bypass_obstacle",
    "car_following",
    "yielding",
    "merge_or_cut_in",
    "intersection_crossing",
    "pedestrian_interaction",
    "emergency_braking",
    "low_speed_maneuver",
]

HAZARD_CAPABILITIES = [
    "traffic_sign_marking",
    "road_geometry",
    "limited_sight_distance",
    "road_surface_low_friction",
    "static_obstacle_or_intrusion",
    "falling_or_moving_obstacle",
    "construction_or_lane_blockage",
    "priority_conflict",
    "adverse_weather_visibility",
    "adverse_lighting_glare",
    "adverse_lighting_low_light",
]


def metric_score(context, name, default=0.0):
    return float((context or {}).get(name, {}).get("score", default))


def selected_capabilities(config, group, candidates):
    vector = (config.get("capability_vector") or {}).get(group, {})
    return {name: int(bool(vector.get(name, 0))) for name in candidates}


def mean_present(scores):
    present = [value for value in scores.values() if value is not None]
    return sum(present) / len(present) if present else 0.0


class BehaviorCapabilityScoreMetric(BaseMetric):
    name = "behavior_capability_score"

    def compute(self, frames, config, context=None):
        route = metric_score(context, "route_completion")
        collision = metric_score(context, "collision_penalty")
        proximity = metric_score(context, "proximity_risk")
        speed = metric_score(context, "speed_appropriateness", 1.0)
        stability = metric_score(context, "control_stability", 1.0)
        scenario_score = clamp(
            0.35 * route
            + 0.25 * collision
            + 0.20 * proximity
            + 0.10 * speed
            + 0.10 * stability
        )
        selected = selected_capabilities(config, "behavior", BEHAVIOR_CAPABILITIES)
        per_capability = {
            name: scenario_score if selected[name] else None
            for name in BEHAVIOR_CAPABILITIES
        }
        return MetricResult.make(self.name, mean_present(per_capability), {
            "mode": "selected_binary_capability_vector",
            "scenario_pass_score": scenario_score,
            "selected_capabilities": [name for name, value in selected.items() if value],
            "per_capability_scores": per_capability,
            "inputs": {
                "route_completion": route,
                "collision_penalty": collision,
                "proximity_risk": proximity,
                "speed_appropriateness": speed,
                "control_stability": stability,
            },
        })


class HazardCapabilityScoreMetric(BaseMetric):
    name = "hazard_capability_score"

    def compute(self, frames, config, context=None):
        route = metric_score(context, "route_completion")
        collision = metric_score(context, "collision_penalty")
        speed = metric_score(context, "speed_appropriateness", 1.0)
        hazard = metric_score(context, "long_tail_hazard_response", 1.0)
        trajectory = metric_score(context, "trajectory_adherence")
        base_score = clamp(
            0.25 * route
            + 0.20 * collision
            + 0.20 * speed
            + 0.25 * hazard
            + 0.10 * trajectory
        )
        response_details = (context or {}).get("long_tail_hazard_response", {}).get("details", {})
        if response_details.get("reason") == "no_hazard_events":
            response_gate = 0.0
        else:
            response_gate = 0.35 + 0.65 * hazard
        scenario_score = clamp(base_score * response_gate)
        selected = selected_capabilities(config, "hazard", HAZARD_CAPABILITIES)
        per_capability = {
            name: scenario_score if selected[name] else None
            for name in HAZARD_CAPABILITIES
        }
        return MetricResult.make(self.name, mean_present(per_capability), {
            "mode": "selected_binary_capability_vector",
            "scenario_pass_score": scenario_score,
            "base_score": base_score,
            "hazard_response_gate": response_gate,
            "selected_capabilities": [name for name, value in selected.items() if value],
            "per_capability_scores": per_capability,
            "inputs": {
                "route_completion": route,
                "collision_penalty": collision,
                "speed_appropriateness": speed,
                "long_tail_hazard_response": hazard,
                "trajectory_adherence": trajectory,
            },
        })
