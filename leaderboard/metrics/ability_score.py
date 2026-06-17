from .base import BaseMetric, MetricResult


class AbilityScoreMetric(BaseMetric):
    name = "ability_score"

    def compute(self, frames, config, context=None):
        context = context or {}
        success = bool(config.get("scenario_success", False))
        if not success:
            success = (
                float(context.get("route_completion", {}).get("score", 0.0)) >= 0.95
                and float(context.get("collision_penalty", {}).get("score", 0.0)) >= 0.999
                and float(context.get("drivable_area", {}).get("score", 0.0)) >= 0.80
                and float(context.get("omnidirectional_interaction_risk", {}).get("score", 0.0)) >= 0.70
            )
        tags = config.get("scenario_tags", [])
        subtype_scores = {tag: 1.0 if success else 0.0 for tag in tags if "." in tag}
        groups = {}
        for group in ("A", "B", "C"):
            vals = [v for k, v in subtype_scores.items() if k.startswith(group + ".")]
            groups[group] = sum(vals) / len(vals) if vals else None
        present = [v for v in groups.values() if v is not None]
        score = sum(present) / len(present) if present else (1.0 if success else 0.0)
        return MetricResult.make(self.name, score, {"success": success, "scenario_tags": tags, "group_scores": groups})
