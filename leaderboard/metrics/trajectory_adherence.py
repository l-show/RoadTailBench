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


class TrajectoryAdherenceMetric(BaseMetric):
    name = "trajectory_adherence"

    def compute(self, frames, config, context=None):
        if not frames:
            return MetricResult.make(self.name, 0.0, {"reason": "missing_frames"})

        reference = normalize_reference_trajectory(config)
        route = [(p["x"], p["y"]) for p in reference]
        requested_mode = str(config.get("trajectory_adherence_mode", "spatial")).lower()
        spatiotemporal = requested_mode in ("spatiotemporal", "spatiotemporal_reference_deviation", "time_progress")
        mode = "spatiotemporal_reference_deviation" if spatiotemporal else "spatial_reference_deviation"
        if len(route) < 2:
            return MetricResult.make(self.name, 0.0, {
                "mode": mode,
                "reason": "invalid_missing_reference_trajectory",
            })

        distances = polyline_lengths(route)
        route_length = distances[-1]
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
        ref_speed_mps = max(float(config.get("reference_speed_kmh", 50.0)) / 3.6, 0.1)
        for index, frame in enumerate(frames):
            e = ego(frame)
            pos = location_xy(e)
            actual_s, lateral, seg_index = project_point_to_polyline(pos, route)
            expected_s = min(route_length, times[index] * ref_speed_mps)
            progress_error = abs(actual_s - expected_s)
            actual_yaw = yaw_deg(e) if e.get("rotation") else None
            ref_yaw = reference_yaw_at(reference, seg_index)
            heading_error = heading_error_deg(actual_yaw, ref_yaw)

            lateral_score = _score_error(lateral, allowed_lat, hard_lat)
            if spatiotemporal:
                progress_score = min(
                    _score_error(progress_error, allowed_progress, hard_progress),
                    _score_error(progress_error / ref_speed_mps, allowed_time, hard_time),
                )
            else:
                progress_score = 1.0
            if heading_error is None:
                heading_score = 1.0
            else:
                heading_score = _score_error(heading_error, allowed_heading, hard_heading)
                heading_errors.append(heading_error)

            if spatiotemporal:
                frame_scores.append(0.45 * lateral_score + 0.35 * progress_score + 0.20 * heading_score)
            else:
                frame_scores.append(0.75 * lateral_score + 0.25 * heading_score)
            lateral_errors.append(lateral)
            progress_errors.append(progress_error)
            progress_values.append(actual_s)

        details = {
            "mode": mode,
            "route_length_m": route_length,
            "max_lateral_deviation_m": max(lateral_errors),
            "mean_lateral_deviation_m": sum(lateral_errors) / len(lateral_errors),
            "final_progress_m": max(progress_values) if progress_values else 0.0,
            "allowed_lateral_error_m": allowed_lat,
            "hard_lateral_error_m": hard_lat,
        }
        if spatiotemporal:
            details.update({
                "reference_speed_kmh": ref_speed_mps * 3.6,
                "max_progress_error_m": max(progress_errors),
                "mean_progress_error_m": sum(progress_errors) / len(progress_errors),
                "allowed_progress_error_m": allowed_progress,
                "hard_progress_error_m": hard_progress,
                "allowed_time_error_s": allowed_time,
                "hard_time_error_s": hard_time,
            })
        if heading_errors:
            details.update({
                "max_heading_error_deg": max(heading_errors),
                "mean_heading_error_deg": sum(heading_errors) / len(heading_errors),
                "allowed_heading_error_deg": allowed_heading,
                "hard_heading_error_deg": hard_heading,
            })
        return MetricResult.make(self.name, sum(frame_scores) / len(frame_scores), details)
