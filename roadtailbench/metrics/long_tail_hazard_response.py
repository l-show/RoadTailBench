import math

from .base import BaseMetric, MetricResult
from ..core.extractors import control, ego, location_xy, speed_mps
from ..core.geometry import distance2


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
            enter_time, response_time, violation, prev_speed = None, None, False, None
            for frame in frames:
                t = float(frame.get("time", frame.get("frame", 0) * 0.05))
                e = ego(frame)
                d = distance2(location_xy(e), center)
                if enter_time is None and d <= perception:
                    enter_time = t
                if enter_time is not None and d <= danger:
                    violation = True
                if frame.get("collisions"):
                    violation = True
                if enter_time is not None and response_time is None:
                    c = control(e)
                    cur_speed = speed_mps(e)
                    braking = float(c.get("brake", 0.0)) > float(config.get("response_brake_threshold", 0.15))
                    steering = abs(float(c.get("steer", 0.0))) > float(config.get("response_steer_threshold", 0.20))
                    slowing = prev_speed is not None and (prev_speed - cur_speed) > float(config.get("response_speed_drop_mps", 0.5))
                    if braking or steering or slowing:
                        response_time = t
                prev_speed = speed_mps(e)
            if enter_time is None:
                score, rt = 1.0, None
            elif response_time is None:
                score, rt = 0.0, None
            else:
                rt = max(0.0, response_time - enter_time)
                score = math.exp(-rt / tau)
            if violation and not hazard.get("allow_enter_danger_zone", False):
                score *= 0.2
            results.append({"id": hazard.get("id"), "type": hazard.get("type"), "reaction_time_s": rt, "score": score})
        return MetricResult.make(self.name, sum(r["score"] for r in results) / len(results), {"hazard_responses": results})
