from .base import BaseMetric, MetricResult
from ..core.extractors import ego, location_xy, yaw_deg
from ..core.geometry import clamp, polyline_lengths, project_point_to_polyline
from ..core.trajectory import heading_error_deg, normalize_reference_trajectory, reference_yaw_at


def _relative_times(frames):
    times = [float(frame.get("time", 0.0)) for frame in frames]
    start = times[0] if times else 0.0
    return [time - start for time in times]


def _score_error(value, allowed, hard):
    if value <= allowed:
        return 1.0
    return 1.0 - clamp((value - allowed) / max(hard - allowed, 0.1))


class DrivableAreaMetric(BaseMetric):
    name = "drivable_area"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})

        reference = normalize_reference_trajectory(config)
        route = [(p["x"], p["y"]) for p in reference]
        if len(route) < 2:
            return MetricResult.make(self.name, 1.0, {
                "mode": "spatiotemporal_reference_deviation",
                "reason": "missing_reference_trajectory",
            })

        distances = polyline_lengths(route)
        route_length = distances[-1]
        ref_speed_mps = max(float(config.get("reference_speed_kmh", 50.0)) / 3.6, 0.1)
        allowed_lat = float(config.get("allowed_lateral_error_m", 4.0))
        hard_lat = float(config.get("hard_lateral_error_m", 12.0))
        allowed_progress = float(config.get("allowed_progress_error_m", 20.0))
        hard_progress = float(config.get("hard_progress_error_m", 60.0))
        allowed_time = float(config.get("allowed_time_error_s", 3.0))
        hard_time = float(config.get("hard_time_error_s", 8.0))
        allowed_heading = float(config.get("allowed_heading_error_deg", 45.0))
        hard_heading = float(config.get("hard_heading_error_deg", 120.0))

        times = _relative_times(frames)
        frame_scores, lateral_errors, progress_errors, heading_errors = [], [], [], []
        progress_values = []
        for index, frame in enumerate(frames):
            pos = location_xy(ego(frame))
            actual_s, lateral, seg_index = project_point_to_polyline(pos, route)
            expected_s = min(route_length, times[index] * ref_speed_mps)
            progress_error = abs(actual_s - expected_s)
            time_error_m = min(hard_progress, abs(actual_s - expected_s))
            actual_yaw = yaw_deg(ego(frame)) if ego(frame).get("rotation") else None
            ref_yaw = reference_yaw_at(reference, seg_index)
            heading_error = heading_error_deg(actual_yaw, ref_yaw)

            lateral_score = _score_error(lateral, allowed_lat, hard_lat)
            progress_score = min(
                _score_error(progress_error, allowed_progress, hard_progress),
                _score_error(time_error_m / ref_speed_mps, allowed_time, hard_time),
            )
            if heading_error is None:
                heading_score = 1.0
            else:
                heading_score = _score_error(heading_error, allowed_heading, hard_heading)
                heading_errors.append(heading_error)

            frame_scores.append(0.45 * lateral_score + 0.35 * progress_score + 0.20 * heading_score)
            lateral_errors.append(lateral)
            progress_errors.append(progress_error)
            progress_values.append(actual_s)

        details = {
            "mode": "spatiotemporal_reference_deviation",
            "route_length_m": route_length,
            "reference_speed_kmh": ref_speed_mps * 3.6,
            "max_lateral_deviation_m": max(lateral_errors),
            "mean_lateral_deviation_m": sum(lateral_errors) / len(lateral_errors),
            "max_progress_error_m": max(progress_errors),
            "mean_progress_error_m": sum(progress_errors) / len(progress_errors),
            "final_progress_m": max(progress_values) if progress_values else 0.0,
            "allowed_lateral_error_m": allowed_lat,
            "hard_lateral_error_m": hard_lat,
            "allowed_progress_error_m": allowed_progress,
            "hard_progress_error_m": hard_progress,
            "allowed_time_error_s": allowed_time,
            "hard_time_error_s": hard_time,
        }
        if heading_errors:
            details.update({
                "max_heading_error_deg": max(heading_errors),
                "mean_heading_error_deg": sum(heading_errors) / len(heading_errors),
                "allowed_heading_error_deg": allowed_heading,
                "hard_heading_error_deg": hard_heading,
            })
        return MetricResult.make(self.name, sum(frame_scores) / len(frame_scores), details)
