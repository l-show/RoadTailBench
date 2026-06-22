from .base import BaseMetric, MetricResult


class AbilityScoreMetric(BaseMetric):
    name = "ability_score"

    def compute(self, frames, config, context=None):
        context = context or {}
        route = float(context.get("route_completion", {}).get("score", 0.0))
        collision = float(context.get("collision_penalty", {}).get("score", 0.0))
        trajectory = float(context.get("trajectory_adherence", {}).get("score", 0.0))
        proximity = float(context.get("proximity_risk", {}).get("score", 0.0))
        hazard = float(context.get("long_tail_hazard_response", {}).get("score", 1.0))
        speed = float(context.get("speed_appropriateness", {}).get("score", 1.0))
        comfort = float(context.get("comfort", {}).get("score", 1.0))
        stability = float(context.get("control_stability", {}).get("score", 1.0))
        energy = float(context.get("energy_efficiency", {}).get("score", 1.0))
        group_base = {
            "A": max(0.0, min(1.0, 0.30 * route + 0.30 * trajectory + 0.20 * speed + 0.20 * hazard)),
            "B": max(0.0, min(1.0, 0.25 * route + 0.35 * collision + 0.25 * proximity + 0.15 * hazard)),
            "C": max(0.0, min(1.0, 0.25 * speed + 0.25 * comfort + 0.25 * stability + 0.25 * energy)),
        }
        tags = config.get("ability_tags") or config.get("scenario_tags") or ["A", "B", "C"]
        if isinstance(tags, dict):
            raw_tags = [key for key, vals in tags.items() if vals or key in ("A", "B", "C")]
        else:
            raw_tags = tags
        groups_present = []
        for tag in raw_tags:
            group = str(tag).split(".", 1)[0]
            if group in group_base and group not in groups_present:
                groups_present.append(group)
        inferred = False
        if not groups_present:
            groups_present = ["A", "B", "C"]
            inferred = True
        groups = {}
        for group in ("A", "B", "C"):
            groups[group] = group_base[group] if group in groups_present else None
        present = [v for v in groups.values() if v is not None]
        score = sum(present) / len(present) if present else (0.5 * route + 0.5 * collision)
        success = bool(config.get("scenario_success", False)) or (
            route >= 0.95 and collision >= 0.75 and trajectory >= 0.80 and proximity >= 0.60 and hazard >= 0.40
        )
        return MetricResult.make(self.name, score, {
            "success": success,
            "ability_tags": groups_present,
            "ability_tags_inferred": inferred,
            "group_scores": groups,
            "inputs": {
                "route_completion": route,
                "collision_penalty": collision,
                "trajectory_adherence": trajectory,
                "proximity_risk": proximity,
                "hazard_response": hazard,
                "speed_appropriateness": speed,
                "comfort": comfort,
                "control_stability": stability,
                "energy_efficiency": energy,
            },
        })
