# Repository Guidelines

## Project Structure & Module Organization

This repository is the RoadTailBench benchmark/runtime project. The Python package is `leaderboard`.

- `leaderboard/`: core runtime package. `runtime/` orchestrates CARLA, logging, crash handling, and video I/O; `metrics/` computes scores; `scenarios/` discovers scene scripts and metadata; `core/` contains shared parsing and geometry helpers; `cli/` exposes command-line tools.
- `scenes/`: CARLA Python scenario scripts named `RTBXXX.py`.
- `metadata/`: matching scenario metadata named `RTBXXX.json`.
- `tests/`: pytest smoke and unit tests.
- `docs/`: usage notes, especially `docs/RUNNING_ZH.md`.
- `outputs/`: generated run artifacts. Do not commit outputs, videos, logs, or large data files.

## Build, Test, and Development Commands

- `pip install -e .`: install the package in editable mode and register `leaderboard-run`, `leaderboard-eval`, `leaderboard-plot`, and `leaderboard-video`.
- `pytest -q`: run the full local test suite.
- `python -m py_compile leaderboard\runtime\runner.py`: quick syntax check for edited modules.
- `leaderboard-run --scene-root scenes --metadata-root metadata --scenes RTB116-RTB125 --dry-run`: validate discovery without CARLA.
- `leaderboard-run --host localhost --port 2000 --scene-root scenes --metadata-root metadata --scenes RTB116 --ego-mode scene_ego`: run one CARLA scenario.
- `leaderboard-eval --frames outputs\<run>\leaderboard_frame_log.jsonl --config outputs\<run>\leaderboard_scenario_config.json --output outputs\<run>\metrics_recomputed.json`: recompute metrics.

## Coding Style & Naming Conventions

Use Python 3.8+ and four-space indentation. Keep runtime and metric changes small, typed where practical, and consistent with existing helper modules. Prefer structured JSON parsing and shared helpers over ad hoc string handling. Scenario and metadata files must keep matching IDs, for example `scenes/RTB116.py` and `metadata/RTB116.json`.

## Testing Guidelines

Tests use `pytest`. Add focused tests for CLI options, metadata expectations, runtime failure handling, natural termination, and metric edge cases. CARLA-dependent behavior should be covered with dry-run or unit-level tests when possible; document manual CARLA validation in `docs/RUNNING_ZH.md` when needed.

## Commit & Pull Request Guidelines

Recent history uses short Chinese summaries such as `修复了一些问题` and `更新了视频IO和启动`. Keep commits concise but specific. PRs should describe changed behavior, affected commands, validation performed, and any CARLA/manual test gaps. Include plots or output paths only as evidence; do not commit generated `outputs/` artifacts.

## Security & Configuration Tips

Do not commit large spreadsheets, videos, CARLA assets, checkpoints, or machine-specific paths beyond documentation examples. Keep GitHub’s 100 MB limit in mind; use external storage or Git LFS only when intentionally configured.
