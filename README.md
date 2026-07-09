# RoadTailBench Leaderboard

RoadTailBench Leaderboard is a standalone closed-loop evaluation layer for CARLA code scenarios.

This repository intentionally does not use Bench2Drive XML/XOSC route loading. A scene is a Python script such as `RTB116.py`; the script owns its weather, actors, hazards, and scene-owned ego behavior in `scene_ego` mode. The leaderboard runner loads the CARLA map from metadata, starts the scene script, finds the ego vehicle, records frames, and evaluates metrics.

## License Boundary

This project is Apache-2.0. Do not copy code from repositories whose top-level license forbids derivative redistribution. Bench2Drive and Bench2DriveZoo top-level repositories are not vendored here. CARLA MIT-licensed interfaces may be used as external dependencies or referenced with their notices.

## Quick Start

Install the package inside the conda environment you will use to run CARLA:

```powershell
conda activate Carla-0915
cd G:\Codex\RoadTailBench
python -m pip install -e . --no-build-isolation

leaderboard-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --dry-run
```

If the console script is not available yet, use the repository script directly:

```powershell
python G:\Codex\RoadTailBench\run_leaderboard.py `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --dry-run
```

Avoid running `python -m leaderboard...` from `G:\Bench2Drive`, because that directory also contains an older `leaderboard` package.

For automated `scene_ego` CARLA execution:

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --ego-mode scene_ego `
  --scenario-timeout 300 `
  --output-root G:\Codex\RoadTailBench\outputs
```

`--limit 3` runs only the first three discovered scenarios from the selected range. Remove it, or set `--limit 0`, to run the full selection.

If CARLA map loading stalls, test the CARLA side directly:

```powershell
python G:\Codex\RoadTailBench\scripts\carla_control.py `
  --host localhost `
  --port 2000 `
  --timeout 300 `
  --wait `
  --map RTB116 `
  --print-world
```

For source-built CARLA Editor startup, see `scripts\launch_carla_editor.ps1` and `docs\RUNNING_ZH.md`.

The runner automatically loads each scenario's `town` from metadata unless `--skip-load-world` is set. Metrics are computed during run close and can be recomputed later:

```powershell
leaderboard-eval `
  --frames G:\Codex\RoadTailBench\outputs\<run>\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\<run>\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\<run>\metrics.json
```

Scenario-owned ego vehicles should set `role_name="ego"` so the runner can bind logs, video, and natural termination to the correct actor. Reference trajectories are read through one shared parser and may be stored either as JSON arrays or copied multi-line text in `reference_trajectory`; supported formats include `x_y`, `x_y_yaw`, and `x_y_z_yaw`.

Plot a finished run:

```powershell
pip install matplotlib

leaderboard-plot `
  --run-dir G:\Codex\RoadTailBench\outputs\<run> `
  --output G:\Codex\RoadTailBench\outputs\plots\RTB116_report.png
```

`agent_ego` and Zoo adapters remain a later integration phase. The current priority is validating scenario execution and metrics in `scene_ego`.

## Repository Layout

- `leaderboard/runtime`: CARLA connection, map switching, scene process orchestration, frame logging.
- `leaderboard/scenarios`: scene discovery and metadata loading.
- `leaderboard/metrics`: metric implementations and composite scoring.
- `leaderboard/cli`: `leaderboard-run`, `leaderboard-eval`, and `leaderboard-plot`.
- `metadata`: scenario metadata such as `RTB116.json`.
- `scenes`: self-owned RTB scene scripts such as `RTB116.py`.
- `docs`: detailed usage and schema documentation.
