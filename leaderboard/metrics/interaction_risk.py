import math

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


def _score_time(value, danger, safe):
    if value == float("inf"):
        return 1.0
    return clamp((value - danger) / max(safe - danger, 0.1))


def _score_distance(value, danger, safe):
    return clamp((value - danger) / max(safe - danger, 0.1))


def _finite_min(values):
    finite = [v for v in values if v is not None and v != float("inf")]
    return min(finite) if finite else None


class InteractionRiskMetric(BaseMetric):
    name = "proximity_risk"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})

        distance_danger = float(config.get("proximity_danger_distance_m", config.get("interaction_danger_distance_m", 3.0)))
        distance_safe = float(config.get("proximity_caution_distance_m", config.get("interaction_caution_distance_m", 12.0)))
        time_headway_danger = float(config.get("proximity_time_headway_danger_s", 0.7))
        time_headway_safe = float(config.get("proximity_time_headway_caution_s", 1.5))
        long_time_danger = float(config.get("longitudinal_time_margin_danger_s", 0.7))
        long_time_safe = float(config.get("longitudinal_time_margin_safe_s", 1.5))
        lat_time_danger = float(config.get("lateral_time_margin_danger_s", 0.7))
        lat_time_safe = float(config.get("lateral_time_margin_safe_s", 2.0))
        lateral_distance_danger = float(config.get("lateral_clearance_danger_m", 1.0))
        lateral_distance_safe = float(config.get("lateral_clearance_safe_m", 3.0))
        env_danger = float(config.get("environment_clearance_danger_m", 0.75))
        env_safe = float(config.get("environment_clearance_safe_m", 2.5))
        min_speed = float(config.get("safety_margin_min_speed_mps", 0.5))
        env_min_hit = float(config.get("environment_raycast_min_hit_distance_m", 1.0))

        frame_scores = []
        min_distances, min_long_times, min_lat_times = [], [], []
        min_long_distances, min_lat_distances = [], []
        danger_frames = 0
        caution_frames = 0
        raycast_available = 0
        raycast_hit = 0
        censored = 0

        for frame in frames:
            e = ego(frame)
            pos = location_xy(e)
            ego_speed = speed_mps(e)
            evx, evy = velocity_xy(e)
            fx, fy = yaw_to_forward(yaw_deg(e))
            lx, ly = -fy, fx
            ego_long_speed = abs(evx * fx + evy * fy)
            ego_lat_speed = abs(evx * lx + evy * ly)

            dynamic_danger = max(distance_danger, ego_speed * time_headway_danger)
            dynamic_safe = max(distance_safe, ego_speed * time_headway_safe)

            candidate_scores = []
            frame_min_distance = None
            frame_min_long_time = None
            frame_min_lat_time = None
            frame_min_long_distance = None
            frame_min_lat_distance = None

            for actor in frame.get("actors", []):
                d_actor = distance2(pos, tuple(actor.get("location", [0.0, 0.0])[:2]))
                frame_min_distance = d_actor if frame_min_distance is None else min(frame_min_distance, d_actor)
                longitudinal, lateral = decompose_relative(e, actor)
                frame_min_long_distance = abs(longitudinal) if frame_min_long_distance is None else min(frame_min_long_distance, abs(longitudinal))
                frame_min_lat_distance = abs(lateral) if frame_min_lat_distance is None else min(frame_min_lat_distance, abs(lateral))

                av = actor.get("velocity", [0.0, 0.0, 0.0])
                rel_vx, rel_vy = evx - float(av[0]), evy - float(av[1])
                long_closing = abs(rel_vx * fx + rel_vy * fy)
                lat_closing = abs(rel_vx * lx + rel_vy * ly)

                distance_score = _score_distance(d_actor, dynamic_danger, dynamic_safe)
                long_time = abs(longitudinal) / max(long_closing, ego_long_speed, min_speed)
                lat_time = abs(lateral) / max(lat_closing, ego_lat_speed, min_speed)
                long_score = _score_time(long_time, long_time_danger, long_time_safe)
                lat_time_score = _score_time(lat_time, lat_time_danger, lat_time_safe)
                lat_dist_score = _score_distance(abs(lateral), lateral_distance_danger, lateral_distance_safe)

                frame_min_long_time = long_time if frame_min_long_time is None else min(frame_min_long_time, long_time)
                frame_min_lat_time = lat_time if frame_min_lat_time is None else min(frame_min_lat_time, lat_time)
                candidate_scores.append(min(distance_score, long_score, lat_time_score, lat_dist_score))

            proximity = frame.get("proximity", {})
            if proximity.get("raycast_available"):
                raycast_available += 1
            hits = proximity.get("environment_hits") or []
            if hits:
                raycast_hit += 1
            elif proximity.get("raycast_available"):
                censored += 1
            for hit in hits:
                distance = float(hit.get("distance_m", 0.0))
                if distance < env_min_hit:
                    continue
                angle = float(hit.get("relative_angle_deg", 0.0))
                angle_rad = math.radians(angle)
                long_distance = abs(distance * math.cos(angle_rad))
                lat_distance = abs(distance * math.sin(angle_rad))
                frame_min_distance = distance if frame_min_distance is None else min(frame_min_distance, distance)

                env_score = _score_distance(distance, env_danger, env_safe)
                if abs(math.cos(angle_rad)) >= 0.5:
                    frame_min_long_distance = long_distance if frame_min_long_distance is None else min(frame_min_long_distance, long_distance)
                    long_time = long_distance / max(ego_long_speed, min_speed)
                    frame_min_long_time = long_time if frame_min_long_time is None else min(frame_min_long_time, long_time)
                    env_score = min(env_score, _score_time(long_time, long_time_danger, long_time_safe))
                if abs(math.sin(angle_rad)) >= 0.5:
                    frame_min_lat_distance = lat_distance if frame_min_lat_distance is None else min(frame_min_lat_distance, lat_distance)
                    lat_time = lat_distance / max(ego_lat_speed, min_speed)
                    frame_min_lat_time = lat_time if frame_min_lat_time is None else min(frame_min_lat_time, lat_time)
                    env_score = min(
                        env_score,
                        _score_time(lat_time, lat_time_danger, lat_time_safe),
                        _score_distance(lat_distance, lateral_distance_danger, lateral_distance_safe),
                    )
                candidate_scores.append(env_score)

            if not candidate_scores:
                max_range = proximity.get("raycast_max_distance_m")
                if max_range is not None:
                    frame_min_distance = float(max_range)
                    censored += 1
                frame_score = 1.0
            else:
                frame_score = min(candidate_scores)

            if frame_min_distance is not None:
                min_distances.append(frame_min_distance)
            if frame_min_long_time is not None:
                min_long_times.append(frame_min_long_time)
            if frame_min_lat_time is not None:
                min_lat_times.append(frame_min_lat_time)
            if frame_min_long_distance is not None:
                min_long_distances.append(frame_min_long_distance)
            if frame_min_lat_distance is not None:
                min_lat_distances.append(frame_min_lat_distance)
            if frame_score <= 0.0:
                danger_frames += 1
            elif frame_score < 1.0:
                caution_frames += 1
            frame_scores.append(frame_score)

        base_score = sum(frame_scores) / len(frame_scores)
        danger_ratio = danger_frames / len(frames)
        score = clamp(base_score * (1.0 - 0.5 * danger_ratio))
        return MetricResult.make(self.name, score, {
            "mode": "longitudinal_lateral_safety_margin",
            "min_proximity_distance_m": _finite_min(min_distances),
            "min_longitudinal_distance_m": _finite_min(min_long_distances),
            "min_lateral_distance_m": _finite_min(min_lat_distances),
            "min_longitudinal_time_margin_s": _finite_min(min_long_times),
            "min_lateral_time_margin_s": _finite_min(min_lat_times),
            "danger_frame_ratio": danger_ratio,
            "caution_frame_ratio": caution_frames / len(frames),
            "raycast_available_ratio": raycast_available / len(frames),
            "raycast_hit_ratio": raycast_hit / len(frames),
            "sensor_range_censored_ratio": censored / len(frames),
            "distance_danger_m": distance_danger,
            "distance_safe_m": distance_safe,
            "proximity_time_headway_danger_s": time_headway_danger,
            "proximity_time_headway_safe_s": time_headway_safe,
            "longitudinal_time_margin_danger_s": long_time_danger,
            "longitudinal_time_margin_safe_s": long_time_safe,
            "lateral_time_margin_danger_s": lat_time_danger,
            "lateral_time_margin_safe_s": lat_time_safe,
            "lateral_clearance_danger_m": lateral_distance_danger,
            "lateral_clearance_safe_m": lateral_distance_safe,
            "environment_clearance_danger_m": env_danger,
            "environment_clearance_safe_m": env_safe,
            "environment_raycast_min_hit_distance_m": env_min_hit,
            "safety_margin_min_speed_mps": min_speed,
            "note": "Uses longitudinal and lateral safety time margins over actor and environment candidates.",
        })
