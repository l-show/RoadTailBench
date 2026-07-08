from .base import BaseMetric, MetricResult


class AbilityScoreMetric(BaseMetric):
    name = "ability_score"

    def compute(self, frames, config, context=None):
        context = context or {}
        behavior = float(context.get("behavior_capability_score", {}).get("score", 0.0))
        hazard = float(context.get("hazard_capability_score", {}).get("score", 0.0))
        behavior_selected = context.get("behavior_capability_score", {}).get("details", {}).get("selected_capabilities", [])
        hazard_selected = context.get("hazard_capability_score", {}).get("details", {}).get("selected_capabilities", [])
        if behavior_selected and hazard_selected:
            score = 0.5 * behavior + 0.5 * hazard
        elif behavior_selected:
            score = behavior
        elif hazard_selected:
            score = hazard
        else:
            score = 0.0
        success = bool(config.get("scenario_success", False)) or score >= 0.70
        return MetricResult.make(self.name, score, {
            "mode": "compatibility_aggregate_of_two_capability_scores",
            "success": success,
            "deprecated": "A/B/C ability_tags are deprecated; use capability_vector ego_action/hazard_type names+values arrays.",
            "group_scores": {
                "ego_action": behavior if behavior_selected else None,
                "hazard_type": hazard if hazard_selected else None,
            },
            "selected_capabilities": {
                "ego_action": behavior_selected,
                "hazard_type": hazard_selected,
            },
            "inputs": {
                "behavior_capability_score": behavior,
                "hazard_capability_score": hazard,
            },
        })
