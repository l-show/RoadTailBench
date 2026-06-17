# Repository Guidelines

## Role In The Two-Repo Setup

`G:\Codex\RoadTailBench` is the benchmark/runtime repository. It owns the CARLA code scenarios, metadata, frame logging, metric evaluation, and the `rtb-run` / `rtb-eval` command-line entry points.

`G:\Codex\RoadTailBench-Zoo` is the model adapter repository. It should be used only when the benchmark is run in `agent_ego` mode and an external model must produce CARLA controls through a RoadTailBench adapter.

Run benchmark jobs from this repository unless the task is specifically about model adapter code.

## Project Structure & Module Organization

Core package code lives in `roadtailbench/`: `runtime/` handles CARLA orchestration and frame logging, `scenarios/` discovers scene scripts and metadata, `metrics/` computes evaluation scores, `core/` contains shared geometry and I/O helpers, and `cli/` exposes console commands. Scenario scripts are currently stored directly under `scenes/`, with matching JSON metadata directly under `metadata/`. Tests live in `tests/`; generated run artifacts belong in `outputs/` and should not be committed. Additional usage and schema notes are in `docs/`.

## Build, Test, And Development Commands

- `pip install -e .`: install the benchmark package in editable mode and register `rtb-run` and `rtb-eval`.
- `pip install -e G:\Codex\RoadTailBench-Zoo`: install model adapters when running `agent_ego` mode.
- `pytest`: run smoke tests and metric/discovery checks.
- `rtb-run --scene-root scenes --metadata-root metadata --scenes RTB116-RTB125 --dry-run`: validate scenario discovery without importing CARLA.
- `rtb-run --host localhost --port 2000 --scene-root scenes --metadata-root metadata --scenes RTB116 --ego-mode scene_ego`: run a script-owned ego scenario against a local CARLA server.
- `rtb-run --host localhost --port 2000 --scene-root scenes --metadata-root metadata --scenes RTB116 --ego-mode agent_ego --agent roadtailbench_zoo.adapters.rule_based:RuleBasedAdapter`: run with a benchmark-spawned ego controlled by a Zoo adapter.
- `rtb-eval --frames outputs/<run>/roadtailbench_frame_log.jsonl --config outputs/<run>/roadtailbench_scenario_config.json --output metrics.json`: recompute metrics from a saved run.

## Ego Mode Rules

`scene_ego` is the default benchmark mode. The scene script spawns and controls the ego vehicle. The runner must only discover it by `role_name` (`ego`, `hero`), `ego_type_id` / `ego_blueprint`, and `ego_start` proximity, then log frames and compute metrics.

`agent_ego` is for model evaluation. The benchmark runner spawns the ego from metadata and calls a Zoo adapter every tick. Scene scripts may still contain ego-spawn/control code, so future work may need a compatibility patch that suppresses scene-side ego generation when `ROADTAILBENCH_EGO_MODE=agent_ego`.

`script_ego` aliases to `scene_ego`; `external_ego` aliases to `agent_ego`.

## Coding Style & Naming Conventions

Use Python 3.8+ syntax and four-space indentation. Keep modules small and explicit; prefer typed, structured helpers in `roadtailbench/core/` over duplicated parsing logic in CLI or metric code. Name scenario files and metadata as matching IDs, for example `RTB116.py` and `RTB116.json`. Test files should use `test_*.py` names and functions should use `test_*`. Avoid committing caches, logs, virtual environments, or generated `outputs/` content.

## Testing Guidelines

Tests use `pytest`. Add focused tests for discovery, metadata schema expectations, and metric edge cases when changing `roadtailbench/scenarios/` or `roadtailbench/metrics/`. CARLA-dependent behavior should be covered with dry-run or unit-level tests where possible; document any manual CARLA run needed to validate runtime changes.

## Security & Configuration Tips

Keep external CARLA assets and third-party code out of the repository unless their license is compatible with Apache-2.0. Do not vendor Bench2Drive or Bench2DriveZoo code. Prefer local paths in examples, and avoid committing machine-specific configuration, checkpoints, or large run artifacts.
