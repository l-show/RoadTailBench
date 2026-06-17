# Repository Guidelines

## Project Structure & Module Organization

RoadTailBench is a Python package for closed-loop CARLA scenario evaluation. Core package code lives in `roadtailbench/`: `runtime/` handles CARLA orchestration and frame logging, `scenarios/` discovers scene scripts and metadata, `metrics/` computes evaluation scores, `core/` contains shared geometry and I/O helpers, and `cli/` exposes console commands. Scenario scripts are stored under `scenes/rtb116_125/`, with matching JSON metadata in `metadata/rtb116_125/`. Tests live in `tests/`; generated run artifacts belong in `outputs/` and should not be committed. Additional usage and schema notes are in `docs/`.

## Build, Test, and Development Commands

- `pip install -e .`: install the package in editable mode and register `rtb-run` and `rtb-eval`.
- `pytest`: run the smoke tests and metric/discovery checks.
- `rtb-run --scene-root scenes/rtb116_125 --metadata-root metadata/rtb116_125 --scenes RTB116-RTB125 --dry-run`: validate scenario discovery without importing CARLA.
- `rtb-run --host localhost --port 2000 --scene-root scenes/rtb116_125 --metadata-root metadata/rtb116_125 --scenes RTB116 --ego-mode scene_ego`: run a scenario against a local CARLA server.
- `rtb-eval --frames outputs/<run>/roadtailbench_frame_log.jsonl --config outputs/<run>/roadtailbench_scenario_config.json --output metrics.json`: recompute metrics from a saved run.

## Coding Style & Naming Conventions

Use Python 3.8+ syntax and four-space indentation. Keep modules small and explicit; prefer typed, structured helpers in `roadtailbench/core/` over duplicated parsing logic in CLI or metric code. Name scenario files and metadata as matching IDs, for example `RTB116.py` and `RTB116.json`. Test files should use `test_*.py` names and functions should use `test_*`. Avoid committing caches, logs, virtual environments, or generated `outputs/` content.

## Testing Guidelines

Tests use `pytest`. Add focused tests for discovery, metadata schema expectations, and metric edge cases when changing `roadtailbench/scenarios/` or `roadtailbench/metrics/`. CARLA-dependent behavior should be covered with dry-run or unit-level tests where possible; document any manual CARLA run needed to validate runtime changes.

## Commit & Pull Request Guidelines

No Git history is available in this checkout, so use clear imperative commit subjects such as `Add RTB126 metadata validation` or `Fix route completion scoring`. Pull requests should include a concise description, the scenarios or metrics affected, commands run, and any CARLA version or server assumptions. Include screenshots or output paths only when visual reports or generated plots are relevant.

## Security & Configuration Tips

Keep external CARLA assets and third-party code out of the repository unless their license is compatible with Apache-2.0. Do not vendor Bench2Drive or Bench2DriveZoo code. Prefer local paths in examples, and avoid committing machine-specific configuration or large run artifacts.
