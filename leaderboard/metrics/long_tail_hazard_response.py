import math

from .base import BaseMetric, MetricResult
from ..core.extractors import control, ego, location_xy, speed_mps
from ..core.geometry import clamp, distance2


class LongTailHazardResponseMetric(BaseMetric):
    name = "long_tail_hazard_response"

    def compute(self, frames, config, context=None):
        hazards = config.get("hazards", [])
        if not hazards:
            return MetricResult.make(self.name, 1.0, {"reason": "no_hazard_events"})
        tau = float(config.get("response_tau_s", 2.0))
        results = []
        for hazard in hazards:
            center = tuple(hazard.get("center", [0.0, 0.0])[:2])
            perception = float(hazard.get("perception_radius_m", hazard.get("radius_m", 10.0) + 15.0))
            danger = float(hazard.get("danger_radius_m", hazard.get("radius_m", 5.0)))
            expected = str(hazard.get("expected_behavior", "slow_or_avoid"))
            target_speed = float(hazard.get("target_speed_kmh", config.get("hazard_target_speed_kmh", 40.0))) / 3.6
            enter_time, response_time, min_distance, min_speed = None, None, float("inf"), float("inf")
            collision_violation, prev_speed = False, None
            danger_frames = 0
            for frame in frames:
                t = float(frame.get("time", frame.get("frame", 0) * 0.05))
                e = ego(frame)
                d = distance2(location_xy(e), center)
                cur_speed = speed_mps(e)
                min_distance = min(min_distance, d)
                min_speed = min(min_speed, cur_speed)
                if enter_time is None and d <= perception:
                    enter_time = t
                if enter_time is not None and d <= danger:
                    danger_frames += 1
                if frame.get("collisions"):
                    collision_violation = True
                if enter_time is not None and response_time is None:
                    c = control(e)
                    braking = float(c.get("brake", 0.0)) > float(config.get("response_brake_threshold", 0.15))
                    steering = abs(float(c.get("steer", 0.0))) > float(config.get("response_steer_threshold", 0.20))
                    slowing = prev_speed is not None and (prev_speed - cur_speed) > float(config.get("response_speed_drop_mps", 0.5))
                    speed_compliant = cur_speed <= target_speed
                    if braking or steering or slowing or speed_compliant:
                        response_time = t
                prev_speed = cur_speed
            if enter_time is None:
                score, rt, reason = 1.0, None, "not_encountered"
            elif response_time is None:
                score, rt, reason = 0.0, None, "no_response"
            else:
                rt = max(0.0, response_time - enter_time)
                response_score = math.exp(-rt / tau)
                speed_score = 1.0 - clamp(max(0.0, min_speed - target_speed) / max(target_speed, 0.1))
                score = 0.55 * response_score + 0.45 * speed_score
                reason = "responded"
            if collision_violation:
                score *= 0.25
                reason = "collision"
            elif danger_frames and not hazard.get("allow_enter_danger_zone", False) and not expected.startswith("yield"):
                score *= 0.7
                reason = "entered_danger_zone"
            results.append({
                "id": hazard.get("id"),
                "type": hazard.get("type"),
                "expected_behavior": expected,
                "reaction_time_s": rt,
                "min_distance_m": min_distance,
                "min_speed_kmh": min_speed * 3.6 if min_speed < float("inf") else None,
                "danger_frames": danger_frames,
                "reason": reason,
                "score": score,
            })
        return MetricResult.make(self.name, sum(r["score"] for r in results) / len(results), {"hazard_responses": results})
