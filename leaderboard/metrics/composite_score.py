from .base import BaseMetric, MetricResult


class CompositeScoreMetric(BaseMetric):
    name = "leaderboard_driving_score"

    def compute(self, frames, config, context=None):
        context = context or {}

        def s(name, default=1.0):
            return float(context.get(name, {}).get("score", default))

        rc = s("route_completion", 0.0)
        col = s("collision_penalty", 0.0)
        traj = s("trajectory_adherence")
        eff = s("driving_efficiency")
        spd = s("speed_appropriateness")
        prox = s("proximity_risk")
        comfort = s("comfort")
        stable = s("control_stability")
        energy = s("energy_efficiency")
        response = s("long_tail_hazard_response")
        safety_gate = max(0.0, min(1.0, 0.65 * col + 0.35 * prox))
        task_gate = max(0.0, min(1.0, 0.70 * rc + 0.30 * traj))
        score = 100.0 * task_gate * safety_gate
        score *= (0.5 + 0.5 * eff) * (0.6 + 0.4 * spd) * (0.8 + 0.2 * comfort)
        score *= (0.85 + 0.15 * stable) * (0.85 + 0.15 * energy) * (0.7 + 0.3 * response)
        return MetricResult.make(self.name, score, {
            "route_completion": rc,
            "collision_penalty": col,
            "trajectory_adherence": traj,
            "driving_efficiency": eff,
            "speed_appropriateness": spd,
            "proximity_risk": prox,
            "comfort": comfort,
            "control_stability": stable,
            "energy_efficiency": energy,
            "long_tail_hazard_response": response,
            "task_gate": task_gate,
            "safety_gate": safety_gate,
        })
