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
from leaderboard.metrics.drivable_area import DrivableAreaMetric
from leaderboard.metrics.evaluator import evaluate_leaderboard
from leaderboard.metrics.long_tail_hazard_response import LongTailHazardResponseMetric
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
    result = DrivableAreaMetric().compute(frames, config)
    assert result["score"] > 0.95
    assert result["details"]["mode"] == "spatiotemporal_reference_deviation"
    assert result["details"]["max_lateral_deviation_m"] <= 1.0


def test_reference_trajectory_deviation_penalizes_large_offset():
    frames = [
        {"time": 0.0, "ego": {"location": [0.0, 20.0, 0.0], "rotation": [0.0, 0.0, 180.0]}},
        {"time": 1.0, "ego": {"location": [1.0, 20.0, 0.0], "rotation": [0.0, 0.0, 180.0]}},
    ]
    config = {
        "reference_speed_kmh": 36.0,
        "reference_trajectory": [[0, 0, 0], [20, 0, 0]],
    }
    result = DrivableAreaMetric().compute(frames, config)
    assert result["score"] < 0.5
    assert result["details"]["max_heading_error_deg"] >= 120.0


def test_legacy_route_third_column_is_not_treated_as_yaw():
    points = normalize_reference_trajectory({"route": [[0, 0, 0.5], [10, 0, 0.5]]})
    assert points == [{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}]


def test_evaluator_uses_leaderboard_score_name():
    result = evaluate_leaderboard([], {"scenario_id": "RTB_TEST"})
    assert result["scenario_id"] == "RTB_TEST"
    assert "leaderboard_driving_score" in result["metrics"]


def test_collision_penalty_is_weighted_not_binary():
    frames = [
        {"time": 0.0, "collisions": [{"other_actor_id": 1, "other_actor_type": "static.prop.mesh", "type": "collision"}]},
        {"time": 0.1, "collisions": [{"other_actor_id": 1, "other_actor_type": "static.prop.mesh", "type": "collision"}]},
    ]
    result = CollisionPenaltyMetric().compute(frames, {})
    assert result["score"] > 0.0
    assert result["details"]["collision_count"] == 1
    assert result["details"]["weighted_collision_count"] == 0.25


def test_yield_hazard_allows_danger_entry_when_responding():
    frames = [
        {"time": 0.0, "ego": {"location": [10.0, 0.0, 0.0], "speed_mps": 12.0, "control": {"brake": 0.0}}},
        {"time": 0.5, "ego": {"location": [2.0, 0.0, 0.0], "speed_mps": 3.0, "control": {"brake": 1.0}}},
        {"time": 1.0, "ego": {"location": [0.5, 0.0, 0.0], "speed_mps": 0.5, "control": {"brake": 1.0}}},
    ]
    config = {
        "hazards": [{
            "id": "yield_test",
            "center": [0.0, 0.0, 0.0],
            "perception_radius_m": 15.0,
            "danger_radius_m": 4.0,
            "target_speed_kmh": 15.0,
            "expected_behavior": "yield_or_stop_for_priority_conflict",
            "allow_enter_danger_zone": True,
        }]
    }
    result = LongTailHazardResponseMetric().compute(frames, config)
    assert result["score"] > 0.5
    assert result["details"]["hazard_responses"][0]["danger_frames"] > 0


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
        def __init__(self, output_dir, scenario, config):
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
    ):
        assert (run_dir / name).exists()
