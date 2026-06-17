# Repository Guidelines

## Role In The Two-Repo Setup

`G:\Codex\RoadTailBench` is the benchmark/runtime repository. Its Python package is now named `leaderboard`, and it owns the CARLA code scenarios, metadata, frame logging, metric evaluation, and the `leaderboard-run` / `leaderboard-eval` command-line entry points.

`G:\Codex\RoadTailBench-Zoo` is the model adapter repository. It should be used only when the benchmark is run in `agent_ego` mode and an external model must produce CARLA controls through a `roadtailbench_zoo` adapter.

Run benchmark jobs from this repository unless the task is specifically about model adapter code.

## Project Structure & Module Organization

Core package code lives in `leaderboard/`: `runtime/` handles CARLA orchestration and frame logging, `scenarios/` discovers scene scripts and metadata, `metrics/` computes evaluation scores, `core/` contains shared geometry and I/O helpers, and `cli/` exposes console commands. Scenario scripts are currently stored directly under `scenes/`, with matching JSON metadata directly under `metadata/`. Tests live in `tests/`; generated run artifacts belong in `outputs/` and should not be committed. Additional usage and schema notes are in `docs/`.

## Build, Test, And Development Commands

- `pip install -e .`: install the benchmark package in editable mode and register `leaderboard-run`, `leaderboard-eval`, and `leaderboard-plot`.
- `pip install -e G:\Codex\RoadTailBench-Zoo`: install model adapters when running `agent_ego` mode later.
- `pytest`: run smoke tests and metric/discovery checks.
- `leaderboard-run --scene-root scenes --metadata-root metadata --scenes RTB116-RTB125 --dry-run`: validate scenario discovery without importing CARLA.
- `leaderboard-run --host localhost --port 2000 --scene-root scenes --metadata-root metadata --scenes RTB116-RTB125 --limit 3 --ego-mode scene_ego --scenario-timeout 300`: run a bounded batch against local CARLA.
- `leaderboard-eval --frames outputs/<run>/leaderboard_frame_log.jsonl --config outputs/<run>/leaderboard_scenario_config.json --output metrics.json`: recompute metrics from a saved run.

## Ego Mode Rules

`scene_ego` is the current focus. The scene script spawns and controls the ego vehicle. The runner must only discover it by `role_name` (`ego`, `hero`), `ego_type_id` / `ego_blueprint`, and `ego_start` proximity, then log frames and compute metrics.

`agent_ego` is for later model evaluation. The benchmark runner can spawn the ego from metadata and call a Zoo adapter every tick, but scene scripts may still contain ego-spawn/control code. Do not treat this mode as production-ready until scene-side ego generation is suppressed via `LEADERBOARD_EGO_MODE` / `ROADTAILBENCH_EGO_MODE`.

`script_ego` aliases to `scene_ego`; `external_ego` aliases to `agent_ego`.

## Coding Style & Naming Conventions

Use Python 3.8+ syntax and four-space indentation. Keep modules small and explicit; prefer typed, structured helpers in `leaderboard/core/` over duplicated parsing logic in CLI or metric code. Name scenario files and metadata as matching IDs, for example `RTB116.py` and `RTB116.json`. Test files should use `test_*.py` names and functions should use `test_*`. Avoid committing caches, logs, virtual environments, or generated `outputs/` content.

## Testing Guidelines

Tests use `pytest`. Add focused tests for discovery, metadata schema expectations, CLI options, timeout behavior, and metric edge cases when changing `leaderboard/scenarios/`, `leaderboard/runtime/`, or `leaderboard/metrics/`. CARLA-dependent behavior should be covered with dry-run or unit-level tests where possible; document any manual CARLA run needed to validate runtime changes.

## Security & Configuration Tips

Keep external CARLA assets and third-party code out of the repository unless their license is compatible with Apache-2.0. Do not vendor Bench2Drive or Bench2DriveZoo code. Prefer local paths in examples, and avoid committing machine-specific configuration, checkpoints, or large run artifacts.
