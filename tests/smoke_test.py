import json
from pathlib import Path

from roadtailbench.metrics.drivable_area import DrivableAreaMetric
from roadtailbench.scenarios.discovery import discover_scenarios


ROOT = Path(__file__).resolve().parents[1]


def test_discovery():
    scenarios = discover_scenarios(ROOT / "scenes" / "rtb116_125", ROOT / "metadata" / "rtb116_125", "RTB116-RTB125")
    assert len(scenarios) == 10
    assert all(s.metadata_path for s in scenarios)


def test_metadata_json():
    for path in (ROOT / "metadata" / "rtb116_125").glob("RTB*.json"):
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
