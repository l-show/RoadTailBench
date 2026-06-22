from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy, speed_mps, velocity_xy, yaw_deg
from ..core.geometry import clamp, distance2, yaw_to_forward


def decompose_relative(ego_record, actor_record):
    px, py = location_xy(ego_record)
    aloc = actor_record.get("location", [0.0, 0.0, 0.0])
    dx, dy = float(aloc[0]) - px, float(aloc[1]) - py
    fx, fy = yaw_to_forward(yaw_deg(ego_record))
    lx, ly = -fy, fx
    longitudinal = dx * fx + dy * fy
    lateral = dx * lx + dy * ly
    return longitudinal, lateral


def closing_ttc(ego_record, actor_record, longitudinal_distance):
    if longitudinal_distance <= 0.0:
        return float("inf")
    ex, ey = velocity_xy(ego_record)
    av = actor_record.get("velocity", [0.0, 0.0, 0.0])
    ax, ay = float(av[0]), float(av[1])
    fx, fy = yaw_to_forward(yaw_deg(ego_record))
    closing = (ex - ax) * fx + (ey - ay) * fy
    if closing <= 0.1:
        return float("inf")
    return longitudinal_distance / closing


class InteractionRiskMetric(BaseMetric):
    name = "proximity_risk"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        danger = float(config.get("proximity_danger_distance_m", config.get("interaction_danger_distance_m", 3.0)))
        caution = float(config.get("proximity_caution_distance_m", config.get("interaction_caution_distance_m", 12.0)))
        time_headway_danger = float(config.get("proximity_time_headway_danger_s", 0.7))
        time_headway_caution = float(config.get("proximity_time_headway_caution_s", 1.5))
        env_danger = float(config.get("environment_clearance_danger_m", 0.75))
        env_caution = float(config.get("environment_clearance_caution_m", 2.5))
        lateral_danger = float(config.get("lateral_danger_distance_m", 1.5))
        lateral_caution = float(config.get("lateral_caution_distance_m", 4.0))
        lateral_relevance_longitudinal = float(config.get("lateral_relevance_longitudinal_m", 8.0))
        ttc_danger = float(config.get("ttc_danger_s", 2.0))
        ttc_caution = float(config.get("ttc_caution_s", 5.0))
        tlc_danger = float(config.get("tlc_danger_s", 1.0))
        tlc_caution = float(config.get("tlc_caution_s", 3.0))
        vals, min_d, min_ttc, min_tlc = [], float("inf"), float("inf"), float("inf")
        min_longitudinal, min_lateral = float("inf"), float("inf")
        danger_frames = 0
        caution_frames = 0
        dynamic_dangers, dynamic_cautions = [], []
        for frame in frames:
            e = ego(frame)
            pos = location_xy(e)
            ego_speed = speed_mps(e)
            dynamic_danger = max(danger, ego_speed * time_headway_danger)
            dynamic_caution = max(caution, ego_speed * time_headway_caution)
            dynamic_dangers.append(dynamic_danger)
            dynamic_cautions.append(dynamic_caution)
            frame_score = 1.0
            frame_danger = False
            frame_caution = False
            nearest_actor = None
            for actor in frame.get("actors", []):
                d_actor = distance2(pos, tuple(actor.get("location", [0.0, 0.0])[:2]))
                if nearest_actor is None or d_actor < nearest_actor[0]:
                    nearest_actor = (d_actor, actor)
                longitudinal, lateral = decompose_relative(e, actor)
                evx, evy = velocity_xy(e)
                av = actor.get("velocity", [0.0, 0.0, 0.0])
                rel_vx, rel_vy = evx - float(av[0]), evy - float(av[1])
                fx, fy = yaw_to_forward(yaw_deg(e))
                lx, ly = -fy, fx
                lateral_closing = abs(rel_vx * lx + rel_vy * ly)
                min_longitudinal = min(min_longitudinal, abs(longitudinal))
                min_lateral = min(min_lateral, abs(lateral))
                distance_score = clamp((d_actor - dynamic_danger) / max(dynamic_caution - dynamic_danger, 0.1))
                actor_score = distance_score
                if d_actor <= dynamic_danger:
                    frame_danger = True
                if d_actor <= dynamic_caution:
                    frame_caution = True

                if abs(longitudinal) <= lateral_relevance_longitudinal:
                    lateral_score = clamp((abs(lateral) - lateral_danger) / max(lateral_caution - lateral_danger, 0.1))
                    actor_score = min(actor_score, lateral_score)
                    if lateral_closing > 0.1:
                        tlc = max(0.0, abs(lateral) - lateral_danger) / lateral_closing
                        min_tlc = min(min_tlc, tlc)
                        tlc_score = clamp((tlc - tlc_danger) / max(tlc_caution - tlc_danger, 0.1))
                        actor_score = min(actor_score, tlc_score)
                        if tlc <= tlc_danger:
                            frame_danger = True
                        if tlc <= tlc_caution:
                            frame_caution = True
                    if abs(lateral) <= lateral_danger:
                        frame_danger = True
                    if abs(lateral) <= lateral_caution:
                        frame_caution = True

                if abs(lateral) <= lateral_caution:
                    ttc = closing_ttc(e, actor, longitudinal)
                    min_ttc = min(min_ttc, ttc)
                    if ttc != float("inf"):
                        ttc_score = clamp((ttc - ttc_danger) / max(ttc_caution - ttc_danger, 0.1))
                        actor_score = min(actor_score, ttc_score)
                        if ttc <= ttc_danger:
                            frame_danger = True
                        if ttc <= ttc_caution:
                            frame_caution = True
                frame_score = min(frame_score, actor_score)

            env_dist = frame.get("proximity", {}).get("nearest_environment_distance_m")
            if env_dist is not None:
                env_dist = float(env_dist)
                env_score = clamp((env_dist - env_danger) / max(env_caution - env_danger, 0.1))
                frame_score = min(frame_score, env_score)
                if env_dist <= env_danger:
                    frame_danger = True
                if env_dist <= env_caution:
                    frame_caution = True
            d = nearest_actor[0] if nearest_actor else (float(env_dist) if env_dist is not None else caution)
            min_d = min(min_d, d)
            if env_dist is not None:
                min_lateral = min(min_lateral, env_dist)
            if frame_danger:
                danger_frames += 1
            if frame_caution:
                caution_frames += 1
            vals.append(frame_score)
        base_score = sum(vals) / len(vals)
        danger_penalty = danger_frames / len(frames)
        score = clamp(base_score * (1.0 - 0.5 * danger_penalty))
        return MetricResult.make(self.name, score, {
            "min_proximity_distance_m": min_d,
            "min_longitudinal_distance_m": None if min_longitudinal == float("inf") else min_longitudinal,
            "min_lateral_distance_m": None if min_lateral == float("inf") else min_lateral,
            "min_time_to_collision_s": None if min_ttc == float("inf") else min_ttc,
            "min_time_to_lateral_conflict_s": None if min_tlc == float("inf") else min_tlc,
            "danger_frame_ratio": danger_frames / len(frames),
            "caution_frame_ratio": caution_frames / len(frames),
            "danger_distance_m": danger,
            "caution_distance_m": caution,
            "mean_dynamic_danger_distance_m": sum(dynamic_dangers) / len(dynamic_dangers),
            "mean_dynamic_caution_distance_m": sum(dynamic_cautions) / len(dynamic_cautions),
            "proximity_time_headway_danger_s": time_headway_danger,
            "proximity_time_headway_caution_s": time_headway_caution,
            "environment_clearance_danger_m": env_danger,
            "environment_clearance_caution_m": env_caution,
            "lateral_danger_distance_m": lateral_danger,
            "lateral_caution_distance_m": lateral_caution,
            "lateral_relevance_longitudinal_m": lateral_relevance_longitudinal,
            "ttc_danger_s": ttc_danger,
            "ttc_caution_s": ttc_caution,
            "tlc_danger_s": tlc_danger,
            "tlc_caution_s": tlc_caution,
            "ttc_note": "TTC is evaluated for forward longitudinal closing actor risk; TLC-like lateral conflict time is evaluated when lateral motion closes the gap.",
        })
