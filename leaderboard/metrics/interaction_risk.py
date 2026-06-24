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
    return dx * fx + dy * fy, dx * lx + dy * ly


def _distance_score(value, danger, safe):
    return clamp((value - danger) / max(safe - danger, 0.1))


def _time_score(value, danger, safe):
    if value is None or value == float("inf"):
        return 1.0
    return clamp((value - danger) / max(safe - danger, 0.1))


def _finite_min(values):
    vals = [v for v in values if v is not None and v != float("inf")]
    return min(vals) if vals else None


class InteractionRiskMetric(BaseMetric):
    name = "proximity_risk"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})

        long_ttc_danger = float(config.get("actor_longitudinal_ttc_danger_s", 1.0))
        long_ttc_safe = float(config.get("actor_longitudinal_ttc_safe_s", 3.0))
        long_distance_danger = float(config.get("actor_longitudinal_distance_danger_m", 3.0))
        long_headway_safe = float(config.get("actor_longitudinal_headway_safe_s", 1.5))
        lateral_tlc_danger = float(config.get("actor_lateral_tlc_danger_s", 1.0))
        lateral_tlc_safe = float(config.get("actor_lateral_tlc_safe_s", 3.0))
        lateral_clearance_danger = float(config.get("actor_lateral_clearance_danger_m", 1.0))
        lane_width_default = float(config.get("default_lane_width_m", 3.5))
        env_danger = float(config.get("environment_clearance_danger_m", 0.75))
        env_safe = float(config.get("environment_clearance_safe_m", 3.0))
        env_min_hit = float(config.get("environment_raycast_min_hit_distance_m", 1.0))
        w_long = float(config.get("proximity_weight_actor_longitudinal", 0.40))
        w_lat = float(config.get("proximity_weight_actor_lateral", 0.35))
        w_env = float(config.get("proximity_weight_environment", 0.25))
        weight_sum = max(w_long + w_lat + w_env, 0.1)
        w_long, w_lat, w_env = w_long / weight_sum, w_lat / weight_sum, w_env / weight_sum

        long_scores, lat_scores, env_scores, frame_scores = [], [], [], []
        min_long_ttc, min_lat_tlc, min_env_dist = [], [], []
        min_long_dist, min_lat_dist, min_actor_dist = [], [], []
        raycast_available = raycast_hit = raycast_censored = 0

        for frame in frames:
            e = ego(frame)
            pos = location_xy(e)
            evx, evy = velocity_xy(e)
            fx, fy = yaw_to_forward(yaw_deg(e))
            lx, ly = -fy, fx
            ego_speed = speed_mps(e)
            lane_width = float(e.get("lane_width_m") or config.get("lane_width_m") or lane_width_default)
            lane_boundary_distance = max(0.5 * lane_width, 0.1)

            frame_long_scores, frame_lat_scores = [], []
            for actor in frame.get("actors", []):
                d_actor = distance2(pos, tuple(actor.get("location", [0.0, 0.0])[:2]))
                longitudinal, lateral = decompose_relative(e, actor)
                min_actor_dist.append(d_actor)
                min_long_dist.append(abs(longitudinal))
                min_lat_dist.append(abs(lateral))

                av = actor.get("velocity", [0.0, 0.0, 0.0])
                rel_vx = evx - float(av[0])
                rel_vy = evy - float(av[1])
                long_closing = rel_vx * fx + rel_vy * fy
                if longitudinal > 0.0 and long_closing > 0.1:
                    ttc = longitudinal / long_closing
                else:
                    ttc = float("inf")
                min_long_ttc.append(ttc)
                safe_long_distance = max(long_distance_danger, ego_speed * long_headway_safe)
                long_score = min(
                    _time_score(ttc, long_ttc_danger, long_ttc_safe),
                    _distance_score(abs(longitudinal), long_distance_danger, safe_long_distance),
                )
                frame_long_scores.append(long_score)

                lateral_closing = abs(rel_vx * lx + rel_vy * ly)
                lateral_clearance = max(0.0, abs(lateral) - lateral_clearance_danger)
                if lateral_closing > 0.1:
                    tlc = lateral_clearance / lateral_closing
                else:
                    ego_lat_speed = abs(evx * lx + evy * ly)
                    tlc = lateral_clearance / ego_lat_speed if ego_lat_speed > 0.1 else float("inf")
                min_lat_tlc.append(tlc)
                lateral_safe = max(lane_boundary_distance, lateral_clearance_danger + 0.5)
                lat_score = min(
                    _time_score(tlc, lateral_tlc_danger, lateral_tlc_safe),
                    _distance_score(abs(lateral), lateral_clearance_danger, lateral_safe),
                )
                frame_lat_scores.append(lat_score)

            long_score = min(frame_long_scores) if frame_long_scores else 1.0
            lat_score = min(frame_lat_scores) if frame_lat_scores else 1.0

            proximity = frame.get("proximity", {})
            if proximity.get("raycast_available"):
                raycast_available += 1
            valid_env_distances = [
                float(hit.get("distance_m", 0.0))
                for hit in (proximity.get("environment_hits") or [])
                if float(hit.get("distance_m", 0.0)) >= env_min_hit
            ]
            if valid_env_distances:
                raycast_hit += 1
                env_distance = min(valid_env_distances)
                min_env_dist.append(env_distance)
                env_score = _distance_score(env_distance, env_danger, env_safe)
            else:
                if proximity.get("raycast_available"):
                    raycast_censored += 1
                env_score = 1.0

            long_scores.append(long_score)
            lat_scores.append(lat_score)
            env_scores.append(env_score)
            frame_scores.append(w_long * long_score + w_lat * lat_score + w_env * env_score)

        score = sum(frame_scores) / len(frame_scores)
        danger_frames = sum(1 for value in frame_scores if value <= 0.0)
        caution_frames = sum(1 for value in frame_scores if 0.0 < value < 1.0)
        return MetricResult.make(self.name, score, {
            "mode": "three_component_safety_margin",
            "component_scores": {
                "actor_longitudinal": sum(long_scores) / len(long_scores),
                "actor_lateral": sum(lat_scores) / len(lat_scores),
                "environment_clearance": sum(env_scores) / len(env_scores),
            },
            "component_weights": {
                "actor_longitudinal": w_long,
                "actor_lateral": w_lat,
                "environment_clearance": w_env,
            },
            "min_actor_distance_m": _finite_min(min_actor_dist),
            "min_longitudinal_distance_m": _finite_min(min_long_dist),
            "min_lateral_distance_m": _finite_min(min_lat_dist),
            "min_longitudinal_ttc_s": _finite_min(min_long_ttc),
            "min_lateral_tlc_s": _finite_min(min_lat_tlc),
            "min_environment_distance_m": _finite_min(min_env_dist),
            "danger_frame_ratio": danger_frames / len(frames),
            "caution_frame_ratio": caution_frames / len(frames),
            "raycast_available_ratio": raycast_available / len(frames),
            "raycast_hit_ratio": raycast_hit / len(frames),
            "sensor_range_censored_ratio": raycast_censored / len(frames),
            "actor_longitudinal_ttc_danger_s": long_ttc_danger,
            "actor_longitudinal_ttc_safe_s": long_ttc_safe,
            "actor_longitudinal_distance_danger_m": long_distance_danger,
            "actor_longitudinal_headway_safe_s": long_headway_safe,
            "actor_lateral_tlc_danger_s": lateral_tlc_danger,
            "actor_lateral_tlc_safe_s": lateral_tlc_safe,
            "actor_lateral_clearance_danger_m": lateral_clearance_danger,
            "default_lane_width_m": lane_width_default,
            "environment_clearance_danger_m": env_danger,
            "environment_clearance_safe_m": env_safe,
            "environment_raycast_min_hit_distance_m": env_min_hit,
            "note": "Score is a weighted mean of actor longitudinal TTC/distance, actor lateral TLC/clearance, and omnidirectional environment raycast clearance.",
        })
