from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy, velocity_xy
from ..core.geometry import clamp, distance2


def closing_ttc(ego_record, actor_record, distance):
    ex, ey = velocity_xy(ego_record)
    av = actor_record.get("velocity", [0.0, 0.0, 0.0])
    ax, ay = float(av[0]), float(av[1])
    px, py = location_xy(ego_record)
    aloc = actor_record.get("location", [0.0, 0.0, 0.0])
    dx, dy = float(aloc[0]) - px, float(aloc[1]) - py
    if distance <= 1e-3:
        return 0.0
    rel_vx, rel_vy = ex - ax, ey - ay
    closing = (rel_vx * dx + rel_vy * dy) / distance
    if closing <= 0.1:
        return float("inf")
    return distance / closing


class InteractionRiskMetric(BaseMetric):
    name = "omnidirectional_interaction_risk"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        danger = float(config.get("interaction_danger_distance_m", 3.0))
        caution = float(config.get("interaction_caution_distance_m", 12.0))
        vals, min_d, min_ttc = [], float("inf"), float("inf")
        danger_frames = 0
        caution_frames = 0
        for frame in frames:
            e = ego(frame)
            pos = location_xy(e)
            actor_distances = []
            for actor in frame.get("actors", []):
                d_actor = distance2(pos, tuple(actor.get("location", [0.0, 0.0])[:2]))
                actor_distances.append((d_actor, actor))
                min_ttc = min(min_ttc, closing_ttc(e, actor, d_actor))
            d = min((item[0] for item in actor_distances), default=caution)
            min_d = min(min_d, d)
            if d <= danger:
                danger_frames += 1
            if d <= caution:
                caution_frames += 1
            vals.append(clamp((d - danger) / max(caution - danger, 0.1)))
        base_score = sum(vals) / len(vals)
        danger_penalty = danger_frames / len(frames)
        score = clamp(base_score * (1.0 - 0.5 * danger_penalty))
        return MetricResult.make(self.name, score, {
            "min_actor_distance_m": min_d,
            "min_time_to_collision_s": None if min_ttc == float("inf") else min_ttc,
            "danger_frame_ratio": danger_frames / len(frames),
            "caution_frame_ratio": caution_frames / len(frames),
            "danger_distance_m": danger,
            "caution_distance_m": caution,
        })
