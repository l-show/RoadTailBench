from .base import BaseMetric, MetricResult


class CollisionPenaltyMetric(BaseMetric):
    name = "collision_penalty"

    def compute(self, frames, config, context=None):
        window_s = float(config.get("collision_merge_window_s", 5.0))
        seen = {}
        collisions = []
        for frame in frames:
            t = float(frame.get("time", frame.get("frame", 0) * 0.05))
            for event in frame.get("collisions", []):
                key = (
                    event.get("other_actor_id"),
                    event.get("other_actor_type", "unknown"),
                    event.get("type", "collision"),
                )
                if key in seen and t - seen[key] <= window_s:
                    continue
                seen[key] = t
                collisions.append(event)
        score = 1.0 if not collisions else 0.0
        return MetricResult.make(self.name, score, {"collision_count": len(collisions), "collisions": collisions})
