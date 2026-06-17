from .base import BaseMetric, MetricResult


DEFAULT_WEIGHTS = {
    "walker": 1.0,
    "vehicle": 0.75,
    "static": 0.35,
    "prop": 0.25,
    "other": 0.5,
}


def collision_weight(event, config):
    type_id = str(event.get("other_actor_type", "unknown")).lower()
    role = str(event.get("role_name", "")).lower()
    custom = config.get("collision_type_weights", {})
    for key, value in custom.items():
        if str(key).lower() in type_id:
            return float(value)
    if role in {"ignored", "sensor", "decorative"}:
        return 0.0
    if type_id.startswith("walker."):
        return DEFAULT_WEIGHTS["walker"]
    if type_id.startswith("vehicle."):
        return DEFAULT_WEIGHTS["vehicle"]
    if type_id.startswith("static.prop") or "prop" in type_id:
        return DEFAULT_WEIGHTS["prop"]
    if type_id.startswith("static."):
        return DEFAULT_WEIGHTS["static"]
    return DEFAULT_WEIGHTS["other"]


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
                enriched = dict(event)
                enriched["severity_weight"] = collision_weight(event, config)
                enriched["time"] = t
                collisions.append(enriched)
        weighted = sum(float(event.get("severity_weight", 0.0)) for event in collisions)
        tolerance = float(config.get("collision_tolerance_weight", 0.0))
        scale = max(float(config.get("collision_penalty_scale", 2.0)), 0.1)
        score = max(0.0, 1.0 - max(0.0, weighted - tolerance) / scale)
        return MetricResult.make(self.name, score, {
            "collision_count": len(collisions),
            "weighted_collision_count": weighted,
            "collision_tolerance_weight": tolerance,
            "collision_penalty_scale": scale,
            "collisions": collisions,
        })
