# 运行 RoadTailBench Leaderboard

## Dry Run

```powershell
leaderboard-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --dry-run
```

## 自动批量运行 scene_ego

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

`scene_ego` 模式下，`RTBXXX.py` 自己生成并控制 ego。Leaderboard runner 只寻找 ego、记录帧、计算指标。每个场景开始前会根据 metadata 的 `town` 自动切换 CARLA 地图；如果传 `--skip-load-world`，则使用当前地图。

## 重新计算指标

```powershell
leaderboard-eval `
  --frames G:\Codex\RoadTailBench\outputs\<run>\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\<run>\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\<run>\metrics.json
```

## agent_ego 状态

`agent_ego` 是后续模型接入阶段。当前可以保留参数兼容，但不要把它当作已完成路径；多数场景仍需要根据 `LEADERBOARD_EGO_MODE=agent_ego` / `ROADTAILBENCH_EGO_MODE=agent_ego` 屏蔽脚本内部 ego 生成和控制。
