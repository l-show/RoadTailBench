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
            enter_time, response_time, min_distance, min_speed = None, None, float("inf"), float("inf")
            collision_violation, prev_speed = False, None
            entry_speed = None
            response_reason = None
            response_streak = 0
            for frame in frames:
                t = float(frame.get("time", frame.get("frame", 0) * 0.05))
                e = ego(frame)
                d = distance2(location_xy(e), center)
                cur_speed = speed_mps(e)
                min_distance = min(min_distance, d)
                min_speed = min(min_speed, cur_speed)
                if enter_time is None and d <= perception:
                    enter_time = t
                    entry_speed = cur_speed
                if frame.get("collisions"):
                    collision_violation = True
                if enter_time is not None and response_time is None:
                    c = control(e)
                    braking = float(c.get("brake", 0.0)) >= float(config.get("response_brake_threshold", 0.20))
                    steering = abs(float(c.get("steer", 0.0))) >= float(config.get("response_steer_threshold", 0.25))
                    throttle_release = float(c.get("throttle", 0.0)) <= float(config.get("response_throttle_release_threshold", 0.05))
                    slowing = prev_speed is not None and (prev_speed - cur_speed) >= float(config.get("response_speed_drop_mps", 0.4))
                    cumulative_drop = entry_speed is not None and (entry_speed - cur_speed) >= float(config.get("response_cumulative_speed_drop_mps", 1.5))
                    triggered = slowing or cumulative_drop or braking or steering or throttle_release
                    response_streak = response_streak + 1 if triggered else 0
                    if response_streak >= int(config.get("response_min_consecutive_frames", 2)):
                        response_time = t
                        if cumulative_drop or slowing:
                            response_reason = "speed_drop"
                        elif braking:
                            response_reason = "brake"
                        elif throttle_release:
                            response_reason = "throttle_release"
                        else:
                            response_reason = "steer"
                prev_speed = cur_speed
            if enter_time is None:
                score, rt, reason = 1.0, None, "not_encountered"
            elif response_time is None:
                score, rt, reason = 0.0, None, "no_response"
            else:
                rt = max(0.0, response_time - enter_time)
                score = math.exp(-rt / tau)
                reason = "responded"
            if collision_violation:
                score *= 0.25
                reason = "collision"
            results.append({
                "id": hazard.get("id"),
                "type": hazard.get("type"),
                "reaction_time_s": rt,
                "enter_time_s": enter_time,
                "response_time_s": response_time,
                "min_distance_m": min_distance,
                "min_speed_kmh": min_speed * 3.6 if min_speed < float("inf") else None,
                "entry_speed_kmh": entry_speed * 3.6 if entry_speed is not None else None,
                "speed_drop_kmh": (entry_speed - min_speed) * 3.6 if entry_speed is not None and min_speed < float("inf") else None,
                "reason": reason,
                "response_reason": response_reason,
                "score": score,
            })
        return MetricResult.make(self.name, sum(r["score"] for r in results) / len(results), {"hazard_responses": results})
