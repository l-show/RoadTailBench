# RoadTailBench

RoadTailBench is a standalone closed-loop evaluation toolkit for CARLA code scenarios.

This repository intentionally does not use Bench2Drive XML/XOSC route loading. A RoadTailBench scene is a Python script such as `RTB116.py`; the script owns its weather, actors, hazards, and dynamic behavior. The runner loads the CARLA map, starts the scene script, finds the ego vehicle, records frames, and evaluates RoadTailBench metrics.

## License Boundary

This project is Apache-2.0. Do not copy code from repositories whose top-level license forbids derivative redistribution. Bench2Drive and Bench2DriveZoo top-level repositories are not vendored here. CARLA MIT-licensed interfaces may be used as external dependencies or referenced with their notices.

## Quick Start

```powershell
pip install -e G:\Codex\RoadTailBench

rtb-run `
  --scene-root G:\Codex\RoadTailBench\scenes\rtb116_125 `
  --metadata-root G:\Codex\RoadTailBench\metadata\rtb116_125 `
  --scenes RTB116-RTB125 `
  --dry-run
```

For real CARLA execution:

```powershell
rtb-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes\rtb116_125 `
  --metadata-root G:\Codex\RoadTailBench\metadata\rtb116_125 `
  --scenes RTB116 `
  --ego-mode scene_ego `
  --output-root G:\Codex\RoadTailBench\outputs
```

## Repository Layout

- `roadtailbench/runtime`: CARLA connection, frame logging, runner orchestration.
- `roadtailbench/scenarios`: scene discovery and metadata loading.
- `roadtailbench/metrics`: RoadTailBench metrics.
- `metadata/rtb116_125`: initial metadata for RTB116-RTB125.
- `scenes/rtb116_125`: self-owned RTB scene scripts.
- `docs`: detailed usage and schema documentation.
