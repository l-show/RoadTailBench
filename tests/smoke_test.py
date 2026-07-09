import json
import time
from pathlib import Path
from types import SimpleNamespace

from leaderboard.cli.plot_run import main as plot_main
from leaderboard.cli.run import build_argparser
from leaderboard.runtime.carla_utils import actor_to_record
from leaderboard.core.trajectory import normalize_reference_trajectory
from leaderboard.runtime.frame_logger import RuntimeFrameLogger
from leaderboard.metrics.collision_penalty import CollisionPenaltyMetric
from leaderboard.metrics.control_stability import ControlStabilityMetric
from leaderboard.metrics.driving_efficiency import DrivingEfficiencyMetric
from leaderboard.metrics.energy_efficiency import EnergyEfficiencyMetric
from leaderboard.metrics.evaluator import evaluate_leaderboard
from leaderboard.metrics.interaction_risk import InteractionRiskMetric
from leaderboard.metrics.long_tail_hazard_response import LongTailHazardResponseMetric
from leaderboard.metrics.route_completion import RouteCompletionMetric
from leaderboard.metrics.speed_appropriateness import SpeedAppropriatenessMetric
from leaderboard.metrics.trajectory_adherence import TrajectoryAdherenceMetric
from leaderboard.runtime.runner import CodeScenarioRunner
from leaderboard.scenarios.discovery import discover_scenarios


ROOT = Path(__file__).resolve().parents[1]


def test_discovery():
    scenarios = discover_scenarios(ROOT / "scenes", ROOT / "metadata", "RTB116-RTB125")
    assert len(scenarios) == 10
    assert all(s.metadata_path for s in scenarios)


def test_runner_cli_args():
    args = build_argparser().parse_args([
        "--scene-root", "scenes",
        "--metadata-root", "metadata",
        "--scenes", "RTB116-RTB125",
        "--limit", "2",
        "--carla-timeout", "30",
        "--map-load-mode", "helper",
        "--map-load-timeout", "45",
        "--map-load-sleep", "1",
        "--spectator-mode", "none",
        "--abort-on-carla-crash",
        "--record-video",
        "--record-video-mode", "ego_6cam",
        "--video-fps", "8",
        "--restore-world-settings",
        "--scenario-timeout", "120",
        "--environment-raycast-interval-frames", "4",
        "--environment-raycast-distance-m", "25",
        "--environment-raycast-min-hit-distance-m", "1.2",
        "--environment-raycast-angles-deg=-90,0,90",
        "--dry-run",
    ])
    assert args.limit == 2
    assert args.carla_timeout == 30
    assert args.map_load_mode == "helper"
    assert args.map_load_timeout == 45
    assert args.map_load_sleep == 1
    assert args.spectator_mode == "none"
    assert args.abort_on_carla_crash is True
    assert args.record_video is True
    assert args.record_video_mode == "ego_6cam"
    assert args.video_fps == 8
    assert args.restore_world_settings is True
    assert args.scenario_timeout == 120
    assert args.environment_raycast_interval_frames == 4
    assert args.environment_raycast_distance_m == 25.0
    assert args.environment_raycast_min_hit_distance_m == 1.2
    assert args.environment_raycast_angles_deg == "-90,0,90"


def test_runner_cli_defaults():
    args = build_argparser().parse_args(["--scene-root", "scenes"])
    assert args.limit == 0
    assert args.carla_timeout == 180.0
    assert args.map_load_mode == "api"
    assert args.map_load_timeout == 300.0
    assert args.spectator_mode == "ego_start"
    assert args.abort_on_carla_crash is True
    assert args.process_exit_timeout == 2.0
    assert args.restore_world_settings is False
    assert args.scenario_timeout == 0.0
    assert args.natural_end_distance_m == 5.0
    assert args.natural_end_min_ticks == 5
    assert args.disable_natural_end is False
    assert args.environment_raycast_interval_frames == 5
    assert args.environment_raycast_distance_m == 30.0
    assert args.environment_raycast_min_hit_distance_m == 1.0


def test_metadata_json():
    for path in (ROOT / "metadata").glob("RTB*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["scenario_id"] == path.stem
        assert data["ego_type_id"]
        assert "excel_metadata" not in data
        assert "route_waypoints" not in data
        assert "centerline_route" not in data
        if data.get("reference_trajectory_source") == "not_static_ego_reference_detected":
            assert data.get("notes")
        else:
            assert len(data["reference_trajectory"]) >= 2
            assert data["reference_trajectory_format"] in ("x_y", "x_y_yaw", "x_y_z_yaw")


def test_reference_trajectory_deviation_is_loose_for_reasonable_path():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.5, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 0.5, "ego": {"location": [5.0, 0.5, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 1.0, "ego": {"location": [10.0, 0.8, 0.0], "rotation": [0.0, 0.0, 5.0]}},
        {"time": 1.5, "ego": {"location": [15.0, 1.0, 0.0], "rotation": [0.0, 0.0, 5.0]}},
    ]
    config = {
        "reference_speed_kmh": 36.0,
        "reference_trajectory": [[0, 0, 0], [20, 0, 0]],
        "allowed_lateral_error_m": 4.0,
        "hard_lateral_error_m": 12.0,
    }
    result = TrajectoryAdherenceMetric().compute(frames, config)
    assert result["score"] > 0.95
    assert result["details"]["mode"] == "spatial_reference_deviation"
    assert result["details"]["max_lateral_deviation_m"] <= 1.0


def test_spatial_trajectory_adherence_does_not_penalize_timing():
    slow_frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 10.0, "ego": {"location": [20.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
    ]
    fast_frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 0.5, "ego": {"location": [20.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
    ]
    config = {"reference_speed_kmh": 10.0, "reference_trajectory": [[0, 0, 0], [20, 0, 0]]}
    metric = TrajectoryAdherenceMetric()
    assert metric.compute(slow_frames, config)["score"] == metric.compute(fast_frames, config)["score"] == 1.0


def test_spatiotemporal_trajectory_adherence_remains_opt_in():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 0.5, "ego": {"location": [20.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
    ]
    config = {
        "trajectory_adherence_mode": "spatiotemporal",
        "reference_speed_kmh": 10.0,
        "reference_trajectory": [[0, 0, 0], [20, 0, 0]],
    }
    result = TrajectoryAdherenceMetric().compute(frames, config)
    assert result["score"] < 1.0
    assert result["details"]["mode"] == "spatiotemporal_reference_deviation"
    assert result["details"]["max_progress_error_m"] > 0.0


def test_reference_trajectory_deviation_penalizes_large_offset():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 20.0, 0.0], "rotation": [0.0, 0.0, 180.0]}},
        {"time": 1.0, "ego": {"location": [1.0, 20.0, 0.0], "rotation": [0.0, 0.0, 180.0]}},
    ]
    config = {
        "reference_speed_kmh": 36.0,
        "reference_trajectory": [[0, 0, 0], [20, 0, 0]],
    }
    result = TrajectoryAdherenceMetric().compute(frames, config)
    assert result["score"] < 0.5
    assert result["details"]["max_heading_error_deg"] >= 120.0


def test_legacy_route_third_column_is_not_treated_as_yaw():
    points = normalize_reference_trajectory({"route": [[0, 0, 0.5], [10, 0, 0.5]]})
    assert points == [{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}]


def test_reference_trajectory_can_be_multiline_text_with_yaw():
    raw = """
    -10.453 -137.365 84.618
    -10.453 -127.365 84.618
    -10.453 -117.365 84.618
    """
    points = normalize_reference_trajectory({
        "reference_trajectory": raw,
        "reference_trajectory_format": "x_y_yaw",
    })
    assert points == [
        {"x": -10.453, "y": -137.365, "yaw": 84.618},
        {"x": -10.453, "y": -127.365, "yaw": 84.618},
        {"x": -10.453, "y": -117.365, "yaw": 84.618},
    ]


def test_reference_trajectory_text_supports_x_y_z_yaw_format():
    raw = """
    1.0 2.0 0.5 90.0
    3.0 4.0 0.5 91.0
    """
    points = normalize_reference_trajectory({
        "reference_trajectory": raw,
        "reference_trajectory_format": "x_y_z_yaw",
    })
    assert points == [
        {"x": 1.0, "y": 2.0, "yaw": 90.0},
        {"x": 3.0, "y": 4.0, "yaw": 91.0},
    ]


def test_route_completion_missing_route_without_goal_is_zero():
    frames = [{"ego": {"location": [0.0, 0.0, 0.0]}}]
    result = RouteCompletionMetric().compute(frames, {})
    assert result["score"] == 0.0
    assert result["details"]["reason"] == "invalid_missing_route_and_goal"


def test_route_completion_uses_start_goal_fallback():
    frames = [
        {"ego": {"location": [0.0, 0.0, 0.0]}},
        {"ego": {"location": [9.8, 0.0, 0.0]}},
    ]
    config = {"ego_start": {"location": {"x": 0.0, "y": 0.0}}, "ego_end": {"location": {"x": 10.0, "y": 0.0}}}
    result = RouteCompletionMetric().compute(frames, config)
    assert result["score"] == 1.0
    assert result["details"]["mode"] == "start_goal_fallback"


def test_route_completion_goal_can_succeed_off_reference_trajectory():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 20.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 1.0, "ego": {"location": [10.0, 20.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
    ]
    config = {
        "reference_trajectory": [[0, 0, 0], [10, 0, 0]],
        "ego_end": {"location": {"x": 10.0, "y": 20.0}},
        "route_goal_tolerance_m": 1.0,
    }
    assert RouteCompletionMetric().compute(frames, config)["score"] == 1.0
    assert TrajectoryAdherenceMetric().compute(frames, config)["score"] < 0.6


def test_route_completion_completed_ego_destroyed_is_complete():
    frames = [
        {"ego": {"location": [0.0, 0.0, 0.0]}},
        {"ego": {"location": [2.0, 0.0, 0.0]}},
    ]
    config = {
        "reference_trajectory": [[0, 0, 0], [100, 0, 0]],
        "ego_end": {"location": {"x": 100.0, "y": 0.0}},
        "runtime_summary": {"status": "completed", "termination_reason": "ego_destroyed"},
    }
    result = RouteCompletionMetric().compute(frames, config)
    assert result["score"] == 1.0
    assert result["details"]["mode"] == "runtime_terminal_event"
    assert result["details"]["termination_reason"] == "ego_destroyed"


def test_route_completion_completed_goal_terminal_event_is_complete():
    frames = [{"ego": {"location": [0.0, 0.0, 0.0]}}]
    config = {
        "reference_trajectory": [[0, 0, 0], [100, 0, 0]],
        "runtime_summary": {"status": "completed", "termination_reason": "ego_reached_goal"},
    }
    result = RouteCompletionMetric().compute(frames, config)
    assert result["score"] == 1.0
    assert result["details"]["termination_reason"] == "ego_reached_goal"


def test_route_completion_failed_ego_destroyed_uses_geometry():
    frames = [
        {"ego": {"location": [0.0, 0.0, 0.0]}},
        {"ego": {"location": [2.0, 0.0, 0.0]}},
    ]
    config = {
        "reference_trajectory": [[0, 0, 0], [100, 0, 0]],
        "ego_end": {"location": {"x": 100.0, "y": 0.0}},
        "runtime_summary": {"status": "failed", "termination_reason": "ego_destroyed"},
    }
    result = RouteCompletionMetric().compute(frames, config)
    assert result["score"] < 1.0
    assert result["details"]["mode"] == "reference_trajectory_projection"


def test_trajectory_adherence_missing_reference_is_zero():
    frames = [{"ego": {"location": [0.0, 0.0, 0.0]}}]
    result = TrajectoryAdherenceMetric().compute(frames, {})
    assert result["score"] == 0.0
    assert result["details"]["reason"] == "invalid_missing_reference_trajectory"


def test_evaluator_uses_leaderboard_score_name():
    result = evaluate_leaderboard([], {"scenario_id": "RTB_TEST"})
    assert result["scenario_id"] == "RTB_TEST"
    assert "leaderboard_driving_score" in result["metrics"]
    assert "proximity_risk" in result["metrics"]
    assert "behavior_capability_score" in result["metrics"]
    assert "hazard_capability_score" in result["metrics"]
    assert "road_engineering_hazard_adaptation" not in result["metrics"]


def test_capability_scores_use_binary_vector():
    frames = [{"ego": {"location": [0.0, 0.0, 0.0]}}]
    config = {
        "scenario_id": "RTB_CAP",
        "capability_vector": {
            "ego_action": {
                "names": ["Overtaking", "Following", "Yielding", "Merging", "Crossing", "Braking", "Keeping"],
                "values": [1, 0, 0, 0, 0, 0, 1],
            },
            "hazard_type": {
                "names": [
                    "traffic_signs_markings",
                    "separation_protection",
                    "speed_control_facilities",
                    "lighting_facilities",
                    "road_intersection",
                    "road_surface_condition",
                    "road_alignment",
                    "limited_sight_distance",
                    "clearance_intrusion",
                    "adverse_weather",
                ],
                "values": [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            },
        },
    }
    result = evaluate_leaderboard(frames, config)["metrics"]
    behavior = result["behavior_capability_score"]
    hazard = result["hazard_capability_score"]
    assert "Overtaking" in behavior["details"]["selected_capabilities"]
    assert "Keeping" in behavior["details"]["selected_capabilities"]
    assert behavior["details"]["per_capability_scores"]["Following"] is None
    assert behavior["details"]["capability_names"] == ["Overtaking", "Following", "Yielding", "Merging", "Crossing", "Braking", "Keeping"]
    assert "limited_sight_distance" in hazard["details"]["selected_capabilities"]
    assert behavior["details"]["group"] == "ego_action"
    assert hazard["details"]["group"] == "hazard_type"
    assert result["ability_score"]["details"]["mode"] == "compatibility_aggregate_of_two_capability_scores"


def test_capability_scores_accept_legacy_dict_vector():
    frames = [{"ego": {"location": [0.0, 0.0, 0.0]}}]
    config = {
        "scenario_id": "RTB_CAP_LEGACY",
        "capability_vector": {
            "behavior": {"overtaking": 1, "lane_change": 0},
            "hazard": {"road_surface_low_friction": 1},
        },
    }
    result = evaluate_leaderboard(frames, config)["metrics"]
    behavior = result["behavior_capability_score"]
    hazard = result["hazard_capability_score"]
    assert "Overtaking" in behavior["details"]["selected_capabilities"]
    assert "Keeping" not in behavior["details"]["selected_capabilities"]
    assert "road_surface_condition" in hazard["details"]["selected_capabilities"]


def test_collision_penalty_is_weighted_not_binary():
    frames = [
        {"time": 0.0, "collisions": [{"other_actor_id": 1, "other_actor_type": "static.prop.mesh", "type": "collision"}]},
        {"time": 0.1, "collisions": [{"other_actor_id": 1, "other_actor_type": "static.prop.mesh", "type": "collision"}]},
    ]
    result = CollisionPenaltyMetric().compute(frames, {})
    assert result["score"] > 0.0
    assert result["details"]["collision_count"] == 1
    assert result["details"]["weighted_collision_count"] == 0.25


def test_hazard_response_uses_risk_radius_when_responding():
    frames = [
        {"time": 0.0, "ego": {"location": [10.0, 0.0, 0.0], "speed_mps": 12.0, "control": {"brake": 0.0}}},
        {"time": 0.5, "ego": {"location": [2.0, 0.0, 0.0], "speed_mps": 3.0, "control": {"brake": 1.0}}},
        {"time": 1.0, "ego": {"location": [0.5, 0.0, 0.0], "speed_mps": 0.5, "control": {"brake": 1.0}}},
    ]
    config = {
        "hazards": [{
            "id": "yield_test",
            "center": [0.0, 0.0, 0.0],
            "radius_m": 3.0,
            "reference_speed_kmh": 15.0,
            "expected_behavior": "yield_or_stop_for_priority_conflict",
        }]
    }
    result = LongTailHazardResponseMetric().compute(frames, config)
    assert result["score"] > 0.5
    response = result["details"]["hazard_responses"][0]
    assert "danger_frames" not in response
    assert response["risk_radius_m"] == 3.0
    assert response["reason"] == "responded"


def test_hazard_response_ignores_legacy_perception_radius():
    frames = [
        {"time": 0.0, "ego": {"location": [9.0, 0.0, 0.0], "speed_mps": 12.0, "control": {"brake": 1.0}}},
        {"time": 0.5, "ego": {"location": [8.0, 0.0, 0.0], "speed_mps": 6.0, "control": {"brake": 1.0}}},
    ]
    config = {
        "hazards": [{
            "id": "legacy_field_test",
            "center": [0.0, 0.0],
            "radius_m": 3.0,
            "perception_radius_m": 20.0,
        }]
    }
    result = LongTailHazardResponseMetric().compute(frames, config)
    response = result["details"]["hazard_responses"][0]
    assert response["reason"] == "not_encountered"
    assert response["risk_radius_m"] == 3.0


def test_driving_efficiency_uses_sim_time_not_speed_average():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.0, 0.0], "speed_mps": 20.0}},
        {"time": 10.0, "ego": {"location": [100.0, 0.0, 0.0], "speed_mps": 0.0}},
    ]
    config = {"reference_trajectory": [[0, 0], [100, 0]], "speed_limit_kmh": 36.0}
    result = DrivingEfficiencyMetric().compute(frames, config, {"route_completion": {"score": 1.0}})
    assert result["score"] == 1.0
    assert result["details"]["elapsed_sim_seconds"] == 10.0


def test_speed_appropriateness_separates_limit_and_hazard_reference():
    frames = [
        {"ego": {"location": [0.0, 0.0, 0.0], "speed_mps": 10.0}},
        {"ego": {"location": [10.0, 0.0, 0.0], "speed_mps": 25.0}},
        {"ego": {"location": [50.0, 0.0, 0.0], "speed_mps": 8.0}},
    ]
    config = {
        "speed_limit_kmh": 72.0,
        "hazards": [{"id": "h1", "center": [0.0, 0.0], "radius_m": 5.0, "reference_speed_kmh": 18.0}],
    }
    result = SpeedAppropriatenessMetric().compute(frames, config)
    assert result["score"] < 1.0
    assert result["details"]["hazard_frame_ratio"] > 0.0


def test_proximity_risk_uses_three_component_scores():
    frames = [{
        "ego": {
            "location": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0],
            "velocity": [10.0, 0.0, 0.0],
        },
        "actors": [{
            "id": 2,
            "type_id": "vehicle.test",
            "location": [10.0, 0.5, 0.0],
            "velocity": [0.0, 0.0, 0.0],
        }],
        "proximity": {"nearest_environment_distance_m": 20.0},
    }]
    result = InteractionRiskMetric().compute(frames, {})
    assert result["score"] < 1.0
    assert result["details"]["mode"] == "three_component_safety_margin"
    assert result["details"]["min_longitudinal_ttc_s"] is not None
    assert result["details"]["min_lateral_distance_m"] <= 0.5
    assert "component_scores" in result["details"]


def test_proximity_risk_uses_lateral_tlc_margin():
    frames = [{
        "ego": {
            "location": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0],
            "velocity": [0.0, 2.0, 0.0],
        },
        "actors": [{
            "id": 2,
            "type_id": "vehicle.test",
            "location": [1.0, 2.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
        }],
    }]
    result = InteractionRiskMetric().compute(frames, {})
    assert result["details"]["min_lateral_tlc_s"] is not None
    assert result["score"] < 1.0


def test_proximity_risk_uses_environment_hits_as_candidates():
    frames = [{
        "ego": {
            "location": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0],
            "velocity": [4.0, 0.0, 0.0],
        },
        "actors": [],
        "proximity": {
            "raycast_available": True,
            "raycast_max_distance_m": 30.0,
            "environment_hits": [{"relative_angle_deg": 0.0, "distance_m": 1.0}],
            "nearest_environment_distance_m": 1.0,
        },
    }]
    result = InteractionRiskMetric().compute(frames, {})
    assert result["score"] < 1.0
    assert result["details"]["raycast_hit_ratio"] == 1.0
    assert result["details"]["min_environment_distance_m"] == 1.0


def test_proximity_risk_ignores_too_close_environment_hits():
    frames = [{
        "ego": {
            "location": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0],
            "velocity": [4.0, 0.0, 0.0],
        },
        "actors": [],
        "proximity": {
            "raycast_available": True,
            "raycast_max_distance_m": 30.0,
            "environment_hits": [{"relative_angle_deg": 0.0, "distance_m": 0.3}],
            "nearest_environment_distance_m": 0.3,
        },
    }]
    result = InteractionRiskMetric().compute(frames, {})
    assert result["score"] == 1.0
    assert result["details"]["min_environment_distance_m"] is None
    assert result["details"]["sensor_range_censored_ratio"] == 1.0


def test_hazard_response_requires_control_or_speed_change_not_speed_compliance_only():
    frames = [
        {"time": 0.0, "ego": {"location": [3.0, 0.0, 0.0], "speed_mps": 2.0, "control": {"throttle": 0.2, "brake": 0.0, "steer": 0.0}}},
        {"time": 0.1, "ego": {"location": [2.0, 0.0, 0.0], "speed_mps": 2.0, "control": {"throttle": 0.2, "brake": 0.0, "steer": 0.0}}},
    ]
    config = {"hazards": [{"center": [0.0, 0.0], "radius_m": 3.0, "reference_speed_kmh": 40.0}]}
    result = LongTailHazardResponseMetric().compute(frames, config)
    assert result["score"] == 0.0
    assert result["details"]["hazard_responses"][0]["reason"] == "no_response"


def test_control_stability_includes_throttle_and_brake():
    frames = [
        {"ego": {"control": {"steer": 0.0, "throttle": 0.0, "brake": 0.0}}},
        {"ego": {"control": {"steer": 0.0, "throttle": 1.0, "brake": 0.0}}},
        {"ego": {"control": {"steer": 0.0, "throttle": 1.0, "brake": 1.0}}},
    ]
    result = ControlStabilityMetric().compute(frames, {})
    assert result["score"] < 1.0
    assert result["details"]["max_throttle_delta"] == 1.0
    assert result["details"]["max_brake_delta"] == 1.0


def test_energy_efficiency_penalizes_hard_acceleration():
    steady = [
        {"time": 0.0, "ego": {"speed_mps": 10.0}},
        {"time": 1.0, "ego": {"speed_mps": 10.0}},
    ]
    hard = [
        {"time": 0.0, "ego": {"speed_mps": 0.0}},
        {"time": 1.0, "ego": {"speed_mps": 20.0}},
    ]
    metric = EnergyEfficiencyMetric()
    assert metric.compute(hard, {})["score"] < metric.compute(steady, {})["score"]


def test_energy_efficiency_reports_regen_energy():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 0.0, 0.0], "speed_mps": 20.0}},
        {"time": 1.0, "ego": {"location": [15.0, 0.0, 0.0], "speed_mps": 10.0}},
    ]
    result = EnergyEfficiencyMetric().compute(frames, {})
    assert result["details"]["mode"] == "longitudinal_dynamics_with_regen"
    assert result["details"]["regenerated_energy_kwh"] >= 0.0


def test_comfort_uses_body_frame_components():
    frames = [
        {"time": 0.0, "ego": {"acceleration": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0]}},
        {"time": 1.0, "ego": {"acceleration": [0.0, 3.0, 0.0], "rotation": [0.0, 0.0, 30.0]}},
    ]
    result = __import__("leaderboard.metrics.comfort", fromlist=["ComfortMetric"]).ComfortMetric().compute(frames, {})
    assert result["score"] < 1.0
    assert result["details"]["mode"] == "body_frame_accel_jerk_yaw_rate"
    assert result["details"]["max_lateral_accel_mps2"] > 0.0


def test_actor_to_record_without_control():
    class Vec:
        x = 1.0
        y = 2.0
        z = 3.0

    class Rot:
        roll = 0.0
        pitch = 0.0
        yaw = 90.0

    class Transform:
        location = Vec()
        rotation = Rot()

    class StaticActor:
        id = 10
        type_id = "static.prop.box"
        attributes = {}

        def get_transform(self):
            return Transform()

        def get_velocity(self):
            return Vec()

        def get_acceleration(self):
            return Vec()

    record = actor_to_record(StaticActor())
    assert record["type_id"] == "static.prop.box"
    assert "control" not in record


def test_scenario_timeout_writes_summary_and_continues_shape(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    restored = {"called": False}
    runner.args = SimpleNamespace(
        ego_mode="scene_ego",
        output_root=str(tmp_path),
        scenario_timeout=0.001,
        ego_wait_timeout=1.0,
        tick_wait_timeout=1.0,
        cleanup_ego=False,
        restore_world_settings=False,
        disable_natural_end=False,
        natural_end_distance_m=5.0,
        natural_end_min_ticks=5,
        carla_health_timeout=0.1,
        process_exit_timeout=0.1,
        record_video=False,
        spectator_mode="none",
    )
    runner._carla_alive = True
    runner._last_rpc = ""
    runner.probe_carla_alive = lambda: True
    runner.connect_world = lambda scenario: object()
    runner.restore_world = lambda: restored.__setitem__("called", True)
    runner.find_scene_ego = lambda scenario: SimpleNamespace(is_alive=True)
    runner.advance_world_for_collection = lambda world, wait_timeout=None: time.sleep(0.01)

    class FakeProc:
        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

        def kill(self):
            pass

    runner.start_scene_process = lambda scenario, output_dir: FakeProc()
    scenario = SimpleNamespace(scene_id="RTB_TIMEOUT", script_path=ROOT / "scenes" / "RTB116.py", metadata={}, metadata_path=None)

    summary = runner.run_scenario(scenario)

    assert summary["status"] == "completed_timeout"
    assert "scenario-timeout" in summary["error"]
    assert summary["termination_reason"] == "scenario_timeout"
    assert summary["elapsed_wall_seconds"] >= 0.0
    assert restored["called"] is True
    assert (Path(summary["output_dir"]) / "leaderboard_run_summary.json").exists()


def test_natural_end_finishes_before_timeout(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    restored = {"called": False}
    runner.args = SimpleNamespace(
        ego_mode="scene_ego",
        output_root=str(tmp_path),
        scenario_timeout=30.0,
        ego_wait_timeout=1.0,
        tick_wait_timeout=1.0,
        max_ticks=100,
        min_ticks_after_script_exit=20,
        actor_log_radius_m=120.0,
        cleanup_ego=False,
        restore_world_settings=False,
        disable_natural_end=False,
        natural_end_distance_m=5.0,
        natural_end_min_ticks=1,
        carla_health_timeout=0.1,
        process_exit_timeout=0.1,
        record_video=False,
        spectator_mode="none",
    )
    runner._carla_alive = True
    runner._last_rpc = ""
    runner.probe_carla_alive = lambda: True
    runner.connect_world = lambda scenario: object()
    runner.restore_world = lambda: restored.__setitem__("called", True)
    runner.advance_world_for_collection = lambda world, wait_timeout=None: None
    runner.carla = object()

    ego = SimpleNamespace(id=1, is_alive=True)
    runner.find_scene_ego = lambda scenario: ego

    class FakeLogger:
        def __init__(self, output_dir, scenario, config, carla_module=None):
            self.output_dir = Path(output_dir)
            self._frames = []

        def attach_collision_sensor(self, carla, world, ego_actor):
            pass

        def log_tick(self, world, ego_actor, control, actor_radius_m=120.0):
            self._frames.append({
                "frame": 1,
                "time": 1.0,
                "ego": {"location": [10.0, 0.0, 0.0], "speed_mps": 0.0},
                "actors": [],
                "collisions": [],
            })

        def close(self, summary, carla_alive=True):
            summary_path = self.output_dir / "leaderboard_run_summary.json"
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            return {"summary": str(summary_path)}

    class FakeProc:
        def poll(self):
            return None

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

        def kill(self):
            pass

    original_logger = __import__("leaderboard.runtime.runner", fromlist=["RuntimeFrameLogger"]).RuntimeFrameLogger
    __import__("leaderboard.runtime.runner", fromlist=["RuntimeFrameLogger"]).RuntimeFrameLogger = FakeLogger
    try:
        runner.start_scene_process = lambda scenario, output_dir: FakeProc()
        scenario = SimpleNamespace(
            scene_id="RTB_NATURAL",
            script_path=ROOT / "scenes" / "RTB116.py",
            metadata={"town": "TownTest", "ego_end": {"location": {"x": 10.0, "y": 0.0, "z": 0.0}}},
            metadata_path=None,
        )
        runner.world = SimpleNamespace(get_map=lambda: SimpleNamespace(name="TownTest"))
        summary = runner.run_scenario(scenario)
    finally:
        __import__("leaderboard.runtime.runner", fromlist=["RuntimeFrameLogger"]).RuntimeFrameLogger = original_logger

    assert summary["status"] == "completed"
    assert summary["termination_reason"] == "ego_reached_goal"
    assert summary["ticks"] == 1
    assert restored["called"] is True


def test_natural_end_ignores_non_ego_actor_destroyed():
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(
        disable_natural_end=False,
        natural_end_distance_m=5.0,
        natural_end_min_ticks=1,
        spectator_mode="none",
    )
    ego = SimpleNamespace(id=1, is_alive=True)
    frame = {
        "ego": {"location": [0.0, 0.0, 0.0]},
        "actors": [],
    }

    reason, goal_ticks = runner.check_natural_termination(
        ego,
        frame,
        (100.0, 0.0),
        0,
    )

    assert reason is None
    assert goal_ticks == 0


def test_collision_logger_records_location(tmp_path):
    config = {"scenario_id": "RTB_COLLISION", "reference_trajectory": [[0, 0, 0], [1, 0, 0]]}
    logger = RuntimeFrameLogger(tmp_path, SimpleNamespace(scene_id="RTB_COLLISION"), config)

    class FakeSensor:
        def listen(self, callback):
            self.callback = callback

        def stop(self):
            pass

        def destroy(self):
            pass

    sensor = FakeSensor()

    class FakeWorld:
        def get_blueprint_library(self):
            return SimpleNamespace(find=lambda name: object())

        def spawn_actor(self, bp, transform, attach_to=None):
            return sensor

    class FakeCarla:
        Transform = object

    other = SimpleNamespace(id=99, type_id="vehicle.test", attributes={"role_name": "npc"})
    event = SimpleNamespace(
        frame=123,
        other_actor=other,
        transform=SimpleNamespace(location=SimpleNamespace(x=1.0, y=2.0, z=3.0)),
    )

    logger.attach_collision_sensor(FakeCarla, FakeWorld(), SimpleNamespace(get_location=lambda: None))
    sensor.callback(event)
    try:
        assert logger._collisions[0]["location"] == [1.0, 2.0, 3.0]
        assert logger._collisions[0]["other_actor_id"] == 99
    finally:
        logger.close(carla_alive=False)


def test_environment_raycast_is_downsampled_and_reused(tmp_path):
    config = {"scenario_id": "RTB_RAY", "environment_raycast_interval_frames": 5}
    logger = RuntimeFrameLogger(tmp_path, SimpleNamespace(scene_id="RTB_RAY"), config)

    class Loc:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z

        def __add__(self, other):
            return Loc(self.x + other.x, self.y + other.y, self.z + other.z)

        def __mul__(self, scalar):
            return Loc(self.x * scalar, self.y * scalar, self.z * scalar)

        def distance(self, other):
            return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    class FakeCarla:
        Location = Loc
        Vector3D = Loc

    class FakeWorld:
        def __init__(self):
            self.calls = 0

        def cast_ray(self, origin, target):
            self.calls += 1
            return [SimpleNamespace(location=Loc(3.0, 0.0, 1.0), label="test")]

    ego_actor = SimpleNamespace(
        get_transform=lambda: SimpleNamespace(location=Loc(), rotation=SimpleNamespace(yaw=0.0)),
    )
    world = FakeWorld()
    try:
        first = logger.collect_environment_proximity(FakeCarla, world, ego_actor, frame_id=10)
        second = logger.collect_environment_proximity(FakeCarla, world, ego_actor, frame_id=11)
        assert first["raycast_reused"] is False
        assert second["raycast_reused"] is True
        assert world.calls == 7
    finally:
        logger.close(carla_alive=False)


def test_batch_aborts_on_carla_crash(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(output_root=str(tmp_path), abort_on_carla_crash=True)
    calls = []

    def fake_run(scenario):
        calls.append(scenario.scene_id)
        return {
            "scene_id": scenario.scene_id,
            "status": "carla_crashed",
            "error": "simulator unavailable",
            "ticks": 0,
        }

    runner.run_scenario = fake_run
    scenarios = [SimpleNamespace(scene_id="RTB_A", script_path="a.py"), SimpleNamespace(scene_id="RTB_B", script_path="b.py")]
    summaries = runner.run(scenarios)

    assert [s["scene_id"] for s in summaries] == ["RTB_A"]
    assert calls == ["RTB_A"]
    assert (tmp_path / "leaderboard_batch_status.json").exists()


def test_scene_process_forces_utf8_output(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(
        ego_mode="scene_ego",
        host="localhost",
        port=2000,
    )
    scenario = SimpleNamespace(scene_id="RTB_UTF8", script_path=ROOT / "scenes" / "RTB116.py")

    captured = {}

    class FakeProc:
        def __init__(self):
            self._leaderboard_stdout_file = None
            self._leaderboard_stdout_path = None

    def fake_popen(cmd, cwd=None, env=None, stdout=None, stderr=None, text=None, encoding=None, errors=None):
        captured["env"] = env
        captured["encoding"] = encoding
        captured["errors"] = errors
        proc = FakeProc()
        proc._leaderboard_stdout_file = stdout
        return proc

    import leaderboard.runtime.runner as runner_module

    original_popen = runner_module.subprocess.Popen
    runner_module.subprocess.Popen = fake_popen
    try:
        proc = runner.start_scene_process(scenario, tmp_path)
        proc._leaderboard_stdout_file.close()
    finally:
        runner_module.subprocess.Popen = original_popen

    assert captured["env"]["PYTHONIOENCODING"] == "utf-8:replace"
    assert captured["env"]["PYTHONUTF8"] == "1"
    assert captured["encoding"] == "utf-8"
    assert captured["errors"] == "replace"


def test_find_scene_ego_prefers_role_name_over_type_and_start():
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(ego_role_name="ego,hero", ego_type_id="")
    runner._last_rpc = ""

    class Loc:
        def __init__(self, x, y, z=0.0):
            self.x, self.y, self.z = x, y, z

        def distance(self, other):
            return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    class FakeCarla:
        Location = Loc

    class ActorList(list):
        def filter(self, pattern):
            return self

    role_actor = SimpleNamespace(
        id=1,
        type_id="vehicle.other",
        attributes={"role_name": "ego"},
        get_location=lambda: Loc(100.0, 0.0),
    )
    nearby_same_type = SimpleNamespace(
        id=2,
        type_id="vehicle.test",
        attributes={"role_name": "npc"},
        get_location=lambda: Loc(0.5, 0.0),
    )
    runner.carla = FakeCarla
    runner.world = SimpleNamespace(get_actors=lambda: ActorList([nearby_same_type, role_actor]))
    scenario = SimpleNamespace(metadata={
        "ego_role_names": ["ego"],
        "ego_type_id": "vehicle.test",
        "ego_start": {"location": {"x": 0.0, "y": 0.0}},
    })

    assert runner.find_scene_ego(scenario) is role_actor


def test_find_scene_ego_reads_nested_metadata_role_names():
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(ego_role_name="", ego_type_id="")
    runner._last_rpc = ""

    class ActorList(list):
        def filter(self, pattern):
            return self

    nested_role_actor = SimpleNamespace(
        id=1,
        type_id="vehicle.test",
        attributes={"role_name": "ego"},
        get_location=lambda: None,
    )
    runner.world = SimpleNamespace(get_actors=lambda: ActorList([nested_role_actor]))
    scenario = SimpleNamespace(metadata={"ego": {"role_names": ["ego"]}})

    assert runner.find_scene_ego(scenario) is nested_role_actor


def test_find_scene_ego_uses_start_when_type_has_multiple_matches():
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(ego_role_name="", ego_type_id="")
    runner._last_rpc = ""

    class Loc:
        def __init__(self, x, y, z=0.0):
            self.x, self.y, self.z = x, y, z

        def distance(self, other):
            return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    class FakeCarla:
        Location = Loc

    class ActorList(list):
        def filter(self, pattern):
            return self

    near = SimpleNamespace(id=1, type_id="vehicle.test", attributes={}, get_location=lambda: Loc(1.0, 0.0))
    far = SimpleNamespace(id=2, type_id="vehicle.test", attributes={}, get_location=lambda: Loc(30.0, 0.0))
    runner.carla = FakeCarla
    runner.world = SimpleNamespace(get_actors=lambda: ActorList([far, near]))
    scenario = SimpleNamespace(metadata={
        "ego_type_id": "vehicle.test",
        "ego_start": {"location": {"x": 0.0, "y": 0.0}},
        "ego_start_match_radius_m": 8.0,
    })

    assert runner.find_scene_ego(scenario) is near


def test_spawn_agent_ego_uses_ego_role_name():
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(ego_blueprint="vehicle.default")

    class FakeBlueprint:
        def __init__(self):
            self.attributes = {}

        def set_attribute(self, key, value):
            self.attributes[key] = value

    bp = FakeBlueprint()
    captured = {}
    runner.world = SimpleNamespace(
        get_blueprint_library=lambda: SimpleNamespace(find=lambda bp_id: bp),
        try_spawn_actor=lambda blueprint, transform: captured.setdefault("blueprint", blueprint) or SimpleNamespace(id=1),
    )

    class FakeCarla:
        class Transform:
            def __init__(self, location, rotation):
                self.location = location
                self.rotation = rotation

        class Location:
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x, self.y, self.z = x, y, z

        class Rotation:
            def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
                self.pitch, self.yaw, self.roll = pitch, yaw, roll

    runner.carla = FakeCarla
    scenario = SimpleNamespace(metadata={"ego_start": {"location": {"x": 1.0, "y": 2.0}}, "ego_blueprint": "vehicle.test"})

    runner.spawn_agent_ego(scenario)

    assert captured["blueprint"].attributes["role_name"] == "ego"


def test_scenario_marks_carla_crashed_before_start(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(
        output_root=str(tmp_path),
        scenario_timeout=30.0,
        cleanup_ego=False,
        ego_mode="scene_ego",
        process_exit_timeout=0.1,
    )
    runner._carla_alive = False
    runner._last_rpc = ""
    runner.probe_carla_alive = lambda: False
    scenario = SimpleNamespace(scene_id="RTB_DEAD", script_path=ROOT / "scenes" / "RTB116.py", metadata={}, metadata_path=None)

    summary = runner.run_scenario(scenario)

    assert summary["status"] == "carla_crashed"
    assert summary["termination_reason"] == "carla_unavailable"
    assert summary["ticks"] == 0
    assert (Path(summary["output_dir"]) / "leaderboard_run_summary.json").exists()


def test_launch_script_uses_carla_control_for_map():
    text = (ROOT / "scripts" / "launch_carla_editor.ps1").read_text(encoding="utf-8")
    assert "carla_control.py" in text
    assert "--map $MapName" in text
    assert "SleepAfterLoad" in text


def test_plot_run_writes_detailed_pngs(tmp_path, monkeypatch):
    run_dir = tmp_path / "RTB_PLOT"
    run_dir.mkdir()
    frames = [
        {
            "frame": 1,
            "time": 0.0,
            "ego": {
                "id": 1,
                "type_id": "vehicle.test",
                "role_name": "ego",
                "location": [0.0, 0.0, 0.0],
                "velocity": [1.0, 0.0, 0.0],
                "acceleration": [0.1, 0.0, 0.0],
                "speed_mps": 1.0,
                "control": {"throttle": 0.2, "brake": 0.0, "steer": 0.1},
            },
            "actors": [{"id": 2, "type_id": "vehicle.npc", "location": [1.0, 0.0, 0.0]}],
            "collisions": [],
        },
        {
            "frame": 2,
            "time": 0.05,
            "ego": {
                "id": 1,
                "type_id": "vehicle.test",
                "role_name": "ego",
                "location": [1.0, 0.0, 0.0],
                "velocity": [1.0, 0.0, 0.0],
                "acceleration": [0.1, 0.0, 0.0],
                "speed_mps": 1.0,
                "control": {"throttle": 0.0, "brake": 0.2, "steer": -0.1},
            },
            "actors": [{"id": 2, "type_id": "vehicle.npc", "location": [2.0, 0.0, 0.0]}],
            "collisions": [{"location": [1.0, 0.0, 0.0], "other_actor_id": 2, "other_actor_type": "vehicle.npc"}],
        },
    ]
    with (run_dir / "leaderboard_frame_log.jsonl").open("w", encoding="utf-8") as f:
        for frame in frames:
            f.write(json.dumps(frame) + "\n")
    (run_dir / "leaderboard_scenario_config.json").write_text(json.dumps({
        "scenario_id": "RTB_PLOT",
        "reference_trajectory": [[0, 0, 0], [1, 0, 0]],
        "hazards": [{"center": [0.5, 0.0, 0.0]}],
    }), encoding="utf-8")
    metrics = evaluate_leaderboard(frames, {"scenario_id": "RTB_PLOT", "reference_trajectory": [[0, 0, 0], [1, 0, 0]], "scenario_tags": ["A.test", "B.test", "C.test"]})
    (run_dir / "leaderboard_metrics.json").write_text(json.dumps(metrics), encoding="utf-8")

    monkeypatch.setattr("sys.argv", ["leaderboard-plot", "--run-dir", str(run_dir)])
    assert plot_main() == 0

    for name in (
        "leaderboard_trajectory.png",
        "leaderboard_ego_timeseries.png",
        "leaderboard_metric_scores.png",
        "leaderboard_ability_breakdown.png",
        "leaderboard_proximity_timeseries.png",
    ):
        assert (run_dir / name).exists()
