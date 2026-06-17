from .base import BaseMetric, MetricResult


class AbilityScoreMetric(BaseMetric):
    name = "ability_score"

    def compute(self, frames, config, context=None):
        context = context or {}
        route = float(context.get("route_completion", {}).get("score", 0.0))
        collision = float(context.get("collision_penalty", {}).get("score", 0.0))
        drivable = float(context.get("drivable_area", {}).get("score", 0.0))
        interaction = float(context.get("omnidirectional_interaction_risk", {}).get("score", 0.0))
        hazard = float(context.get("long_tail_hazard_response", {}).get("score", 1.0))
        adaptation = float(context.get("road_engineering_hazard_adaptation", {}).get("score", 1.0))
        speed = float(context.get("speed_appropriateness", {}).get("score", 1.0))
        comfort = float(context.get("comfort", {}).get("score", 1.0))
        stability = float(context.get("control_stability", {}).get("score", 1.0))
        base_completion = min(route, collision)
        group_base = {
            "A": max(0.0, min(1.0, 0.30 * route + 0.25 * drivable + 0.25 * adaptation + 0.20 * speed)),
            "B": max(0.0, min(1.0, 0.25 * route + 0.30 * collision + 0.25 * interaction + 0.20 * hazard)),
            "C": max(0.0, min(1.0, 0.25 * route + 0.25 * speed + 0.25 * comfort + 0.25 * stability)),
        }
        tags = config.get("scenario_tags", [])
        subtype_scores = {}
        for tag in tags:
            if "." not in tag:
                continue
            group = tag.split(".", 1)[0]
            if group in group_base:
                subtype_scores[tag] = min(group_base[group], base_completion if group == "B" else 1.0)
        groups = {}
        for group in ("A", "B", "C"):
            vals = [v for k, v in subtype_scores.items() if k.startswith(group + ".")]
            groups[group] = sum(vals) / len(vals) if vals else None
        present = [v for v in groups.values() if v is not None]
        score = sum(present) / len(present) if present else (0.5 * route + 0.5 * collision)
        success = bool(config.get("scenario_success", False)) or (
            route >= 0.95 and collision >= 0.75 and drivable >= 0.80 and interaction >= 0.60 and hazard >= 0.40
        )
        return MetricResult.make(self.name, score, {
            "success": success,
            "scenario_tags": tags,
            "subtype_scores": subtype_scores,
            "group_scores": groups,
            "inputs": {
                "route_completion": route,
                "collision_penalty": collision,
                "drivable_area": drivable,
                "interaction": interaction,
                "hazard_response": hazard,
                "road_engineering_hazard_adaptation": adaptation,
                "speed_appropriateness": speed,
                "comfort": comfort,
                "control_stability": stability,
            },
        })
