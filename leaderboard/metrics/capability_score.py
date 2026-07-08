from .base import BaseMetric, MetricResult
from ..core.geometry import clamp


EGO_ACTIONS = [
    "Overtaking",
    "Following",
    "Yielding",
    "Merging",
    "Crossing",
    "Braking",
    "Keeping",
]

HAZARD_TYPES = [
    "traffic_signs_markings",
    "separation_protection",
    "speed_control_facilities",
    "lighting_facilities",
    "road_intersection",
    "road_surface_condition",
    "road_alignment",
    "limited_sight_distance",
    "clearance_intrusion",
    "adverse_weather",
]

LEGACY_EGO_ACTION_ALIASES = {
    "lane_change": "Merging",
    "overtaking": "Overtaking",
    "bypass_obstacle": "Overtaking",
    "car_following": "Following",
    "yielding": "Yielding",
    "merge_or_cut_in": "Merging",
    "intersection_crossing": "Crossing",
    "pedestrian_interaction": "Yielding",
    "emergency_braking": "Braking",
    "low_speed_maneuver": "Following",
}

LEGACY_HAZARD_TYPE_ALIASES = {
    "traffic_sign_marking": "traffic_signs_markings",
    "road_geometry": "road_alignment",
    "limited_sight_distance": "limited_sight_distance",
    "road_surface_low_friction": "road_surface_condition",
    "static_obstacle_or_intrusion": "clearance_intrusion",
    "falling_or_moving_obstacle": "clearance_intrusion",
    "construction_or_lane_blockage": "clearance_intrusion",
    "priority_conflict": "road_intersection",
    "adverse_weather_visibility": "adverse_weather",
    "adverse_lighting_glare": "adverse_weather",
    "adverse_lighting_low_light": "lighting_facilities",
}


def metric_score(context, name, default=0.0):
    return float((context or {}).get(name, {}).get("score", default))


def selected_capabilities(config, group, candidates, legacy_group=None, legacy_aliases=None):
    capability_vector = config.get("capability_vector") or {}
    selected = {name: 0 for name in candidates}
    vector = capability_vector.get(group)

    if isinstance(vector, dict) and "names" in vector and "values" in vector:
        for name, value in zip(vector.get("names", []), vector.get("values", [])):
            if name in selected:
                selected[name] = int(bool(value))
        return selected

    if isinstance(vector, list):
        for name, value in zip(candidates, vector):
            selected[name] = int(bool(value))
        return selected

    if isinstance(vector, dict):
        for name in candidates:
            selected[name] = int(bool(vector.get(name, 0)))
        return selected

    legacy = capability_vector.get(legacy_group or group, {})
    if isinstance(legacy, dict):
        for old_name, value in legacy.items():
            new_name = (legacy_aliases or {}).get(old_name, old_name)
            if new_name in selected and value:
                selected[new_name] = 1
    return selected


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
        selected = selected_capabilities(
            config,
            "ego_action",
            EGO_ACTIONS,
            legacy_group="behavior",
            legacy_aliases=LEGACY_EGO_ACTION_ALIASES,
        )
        per_capability = {
            name: scenario_score if selected[name] else None
            for name in EGO_ACTIONS
        }
        return MetricResult.make(self.name, mean_present(per_capability), {
            "mode": "selected_binary_capability_array",
            "group": "ego_action",
            "capability_names": EGO_ACTIONS,
            "capability_values": [selected[name] for name in EGO_ACTIONS],
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
        selected = selected_capabilities(
            config,
            "hazard_type",
            HAZARD_TYPES,
            legacy_group="hazard",
            legacy_aliases=LEGACY_HAZARD_TYPE_ALIASES,
        )
        per_capability = {
            name: scenario_score if selected[name] else None
            for name in HAZARD_TYPES
        }
        return MetricResult.make(self.name, mean_present(per_capability), {
            "mode": "selected_binary_capability_array",
            "group": "hazard_type",
            "capability_names": HAZARD_TYPES,
            "capability_values": [selected[name] for name in HAZARD_TYPES],
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
