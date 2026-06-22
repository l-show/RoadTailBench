from .base import BaseMetric, MetricResult
from ..core.extractors import acceleration_xy, ego, velocity_xy, yaw_deg
from ..core.geometry import angle_delta_deg, clamp, yaw_to_forward


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def _rms(values):
    return (_mean([v * v for v in values])) ** 0.5 if values else 0.0


def _score_threshold(rms_value, max_value, rms_limit, hard_limit):
    rms_score = 1.0 - clamp(max(0.0, rms_value - rms_limit) / max(rms_limit, 0.1))
    max_score = 1.0 - clamp(max(0.0, max_value - hard_limit) / max(hard_limit, 0.1))
    return 0.65 * rms_score + 0.35 * max_score


def _smooth(values, radius=2):
    if len(values) <= 2 or radius <= 0:
        return values
    out = []
    for index in range(len(values)):
        lo = max(0, index - radius)
        hi = min(len(values), index + radius + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out


class ComfortMetric(BaseMetric):
    name = "comfort"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})
        long_accel_limit = float(config.get("comfort_longitudinal_accel_rms_limit_mps2", 2.5))
        lateral_accel_limit = float(config.get("comfort_lateral_accel_rms_limit_mps2", 2.0))
        long_accel_hard = float(config.get("comfort_longitudinal_accel_hard_mps2", 5.0))
        lateral_accel_hard = float(config.get("comfort_lateral_accel_hard_mps2", 4.0))
        long_jerk_limit = float(config.get("comfort_longitudinal_jerk_rms_limit_mps3", 3.0))
        lateral_jerk_limit = float(config.get("comfort_lateral_jerk_rms_limit_mps3", 3.0))
        jerk_hard = float(config.get("comfort_jerk_hard_mps3", 12.0))
        yaw_rate_limit = float(config.get("comfort_yaw_rate_rms_limit_deg_s", 20.0))
        yaw_rate_hard = float(config.get("comfort_yaw_rate_hard_deg_s", 45.0))

        long_velocities, lateral_velocities = [], []
        fallback_long_accels, fallback_lateral_accels = [], []
        for frame in frames:
            e = ego(frame)
            ax, ay = acceleration_xy(e)
            vx, vy = velocity_xy(e)
            fx, fy = yaw_to_forward(yaw_deg(e))
            lx, ly = -fy, fx
            long_velocities.append(vx * fx + vy * fy)
            lateral_velocities.append(vx * lx + vy * ly)
            fallback_long_accels.append(ax * fx + ay * fy)
            fallback_lateral_accels.append(ax * lx + ay * ly)
        times = [float(frame.get("time", 0.0)) for frame in frames]
        long_accels, lateral_accels = [], []
        for index in range(len(frames)):
            if index == 0:
                long_accels.append(fallback_long_accels[0])
                lateral_accels.append(fallback_lateral_accels[0])
                continue
            dt = max(times[index] - times[index - 1], 1e-3)
            long_from_velocity = (long_velocities[index] - long_velocities[index - 1]) / dt
            lateral_from_velocity = (lateral_velocities[index] - lateral_velocities[index - 1]) / dt
            long_accels.append(long_from_velocity if abs(long_from_velocity) > 1e-6 else fallback_long_accels[index])
            lateral_accels.append(lateral_from_velocity if abs(lateral_from_velocity) > 1e-6 else fallback_lateral_accels[index])
        long_accels = _smooth(long_accels)
        lateral_accels = _smooth(lateral_accels)
        long_jerks, lateral_jerks, yaw_rates = [], [], []
        for index in range(1, len(frames)):
            dt = max(times[index] - times[index - 1], 1e-3)
            long_jerks.append((long_accels[index] - long_accels[index - 1]) / dt)
            lateral_jerks.append((lateral_accels[index] - lateral_accels[index - 1]) / dt)
            yaw_rates.append(angle_delta_deg(yaw_deg(ego(frames[index])), yaw_deg(ego(frames[index - 1]))) / dt)

        long_accel_abs = [abs(v) for v in long_accels]
        lateral_accel_abs = [abs(v) for v in lateral_accels]
        long_jerk_abs = [abs(v) for v in long_jerks]
        lateral_jerk_abs = [abs(v) for v in lateral_jerks]
        yaw_rate_abs = [abs(v) for v in yaw_rates]

        long_accel_score = _score_threshold(_rms(long_accel_abs), max(long_accel_abs or [0.0]), long_accel_limit, long_accel_hard)
        lateral_accel_score = _score_threshold(_rms(lateral_accel_abs), max(lateral_accel_abs or [0.0]), lateral_accel_limit, lateral_accel_hard)
        long_jerk_score = _score_threshold(_rms(long_jerk_abs), max(long_jerk_abs or [0.0]), long_jerk_limit, jerk_hard)
        lateral_jerk_score = _score_threshold(_rms(lateral_jerk_abs), max(lateral_jerk_abs or [0.0]), lateral_jerk_limit, jerk_hard)
        yaw_score = _score_threshold(_rms(yaw_rate_abs), max(yaw_rate_abs or [0.0]), yaw_rate_limit, yaw_rate_hard)

        score = (
            0.25 * long_accel_score
            + 0.25 * lateral_accel_score
            + 0.20 * long_jerk_score
            + 0.20 * lateral_jerk_score
            + 0.10 * yaw_score
        )
        return MetricResult.make(self.name, score, {
            "mode": "body_frame_accel_jerk_yaw_rate",
            "longitudinal_accel_rms_mps2": _rms(long_accel_abs),
            "lateral_accel_rms_mps2": _rms(lateral_accel_abs),
            "longitudinal_jerk_rms_mps3": _rms(long_jerk_abs),
            "lateral_jerk_rms_mps3": _rms(lateral_jerk_abs),
            "yaw_rate_rms_deg_s": _rms(yaw_rate_abs),
            "max_longitudinal_accel_mps2": max(long_accel_abs or [0.0]),
            "max_lateral_accel_mps2": max(lateral_accel_abs or [0.0]),
            "max_longitudinal_jerk_mps3": max(long_jerk_abs or [0.0]),
            "max_lateral_jerk_mps3": max(lateral_jerk_abs or [0.0]),
            "max_yaw_rate_deg_s": max(yaw_rate_abs or [0.0]),
            "component_scores": {
                "longitudinal_acceleration": long_accel_score,
                "lateral_acceleration": lateral_accel_score,
                "longitudinal_jerk": long_jerk_score,
                "lateral_jerk": lateral_jerk_score,
                "yaw_rate": yaw_score,
            },
        })
