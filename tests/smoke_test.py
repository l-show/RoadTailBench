import json
import time
from pathlib import Path
from types import SimpleNamespace

from leaderboard.cli.run import build_argparser
from leaderboard.metrics.drivable_area import DrivableAreaMetric
from leaderboard.metrics.evaluator import evaluate_leaderboard
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
        "--scenario-timeout", "120",
        "--dry-run",
    ])
    assert args.limit == 2
    assert args.carla_timeout == 30
    assert args.scenario_timeout == 120


def test_runner_cli_defaults():
    args = build_argparser().parse_args(["--scene-root", "scenes"])
    assert args.limit == 0
    assert args.carla_timeout == 180.0
    assert args.scenario_timeout == 0.0


def test_metadata_json():
    for path in (ROOT / "metadata").glob("RTB*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["scenario_id"] == path.stem
        assert data["ego_type_id"]
        assert len(data["centerline_route"]) >= 2


def test_centerline_lane_change():
    frames = [
        {"ego": {"location": [0.0, 0.5, 0.0]}},
        {"ego": {"location": [5.0, 0.5, 0.0]}},
        {"ego": {"location": [10.0, 3.1, 0.0]}},
        {"ego": {"location": [15.0, 3.2, 0.0]}},
    ]
    config = {
        "allowed_lateral_error_m": 1.0,
        "hard_lateral_error_m": 3.0,
        "centerline_segments": [
            {"id": "lane_0", "points": [[0, 0, 0], [10, 0, 0]]},
            {"id": "lane_1", "points": [[10, 3, 0], [20, 3, 0]]},
        ],
    }
    result = DrivableAreaMetric().compute(frames, config)
    assert result["score"] == 1.0
    assert result["details"]["selected_segment_counts"] == {"lane_0": 2, "lane_1": 2}


def test_evaluator_uses_leaderboard_score_name():
    result = evaluate_leaderboard([], {"scenario_id": "RTB_TEST"})
    assert result["scenario_id"] == "RTB_TEST"
    assert "leaderboard_driving_score" in result["metrics"]


def test_scenario_timeout_writes_summary_and_continues_shape(tmp_path):
    runner = object.__new__(CodeScenarioRunner)
    runner.args = SimpleNamespace(
        ego_mode="scene_ego",
        output_root=str(tmp_path),
        scenario_timeout=0.001,
        ego_wait_timeout=1.0,
        tick_wait_timeout=1.0,
        cleanup_ego=False,
    )
    runner.connect_world = lambda scenario: object()
    runner.restore_world = lambda: None
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

    assert summary["status"] == "timeout"
    assert "scenario-timeout" in summary["error"]
    assert (Path(summary["output_dir"]) / "leaderboard_run_summary.json").exists()
