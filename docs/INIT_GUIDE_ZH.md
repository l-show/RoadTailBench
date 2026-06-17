# RoadTailBench Leaderboard /init 指南

## 当前仓库分工

- `G:\Codex\RoadTailBench`：Bench 本体和 Leaderboard 评测层。Python 包名是 `leaderboard`，负责 CARLA 场景脚本、元数据、自动切图运行、帧日志和指标计算。
- `G:\Codex\RoadTailBench-Zoo`：后续模型 adapter 层。当前阶段不接模型，先用 `scene_ego` 验证指标链路。

本仓库是因为 Bench2Drive 顶层开源协议边界而做的大幅重构。Bench2Drive 可以作为论文和功能参考，但不把原 `leaderboard/`、`scenario_runner/`、XML/XOSC loader 或模型 wrapper 代码复制进来。

## 重要目录

- `leaderboard/runtime/`：CARLA 连接、自动加载地图、启动场景脚本、寻找 ego、记录帧、场景超时处理。
- `leaderboard/metrics/`：从帧日志和配置计算指标。
- `leaderboard/scenarios/`：发现 `RTBXXX.py` 并加载同名 metadata。
- `leaderboard/cli/`：`leaderboard-run`、`leaderboard-eval`、`leaderboard-plot`。
- `scenes/`：`RTBXXX.py` 场景代码。当前 `scene_ego` 模式下，脚本自己生成和控制 ego。
- `metadata/`：`RTBXXX.json` 场景元数据。`town` 用于自动切换 CARLA 地图。
- `outputs/`：运行输出目录，不提交。

## 基础安装

```powershell
pip install -e G:\Codex\RoadTailBench
```

如果后续进入 `agent_ego` 模型接入阶段，再安装：

```powershell
pip install -e G:\Codex\RoadTailBench-Zoo
```

## 场景发现检查

```powershell
leaderboard-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --dry-run
```

这一步不导入 CARLA，用来确认 `RTBXXX.py` 和 `RTBXXX.json` 能匹配。

## scene_ego 自动化运行

`scene_ego` 是当前重点。场景脚本自己生成和控制 ego，runner 只负责：

- 按 metadata 的 `town` 自动 `load_world()`。
- 启动对应 `RTBXXX.py` 场景进程。
- 按 `ego_role_names`、`ego_type_id` / `ego_blueprint`、`ego_start` 附近位置寻找 ego。
- 逐帧记录 ego、周边 actor、控制量和碰撞。
- 场景结束或超时后写指标，并自动切到下一个场景。

示例：

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --ego-mode scene_ego `
  --carla-timeout 180 `
  --scenario-timeout 300 `
  --output-root G:\Codex\RoadTailBench\outputs
```

参数说明：

- `--scenes RTB116-RTB125`：选择场景范围，也支持逗号组合。
- `--limit 3`：只跑发现结果里的前 3 个，便于小批量验证。
- `--scenario-timeout 300`：单场景 wall-clock 超时，超时后标记为 `timeout` 并继续下一个。
- `--carla-timeout 180`：CARLA RPC timeout。
- `--skip-load-world`：不自动切图，使用当前 CARLA world。

## 输出和指标

每次运行会在 `outputs/<scene>_<timestamp>/` 下生成：

- `leaderboard_frame_log.jsonl`
- `leaderboard_scenario_config.json`
- `leaderboard_metrics.json`
- `leaderboard_run_summary.json`

批量汇总写到：

```text
outputs/leaderboard_batch_summary.json
```

重新计算指标：

```powershell
leaderboard-eval `
  --frames G:\Codex\RoadTailBench\outputs\<run>\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\<run>\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\<run>\metrics.json
```

## 当前缺少但暂不迁移的 Bench2Drive 能力

- 不迁移 XML/XOSC route loader。
- 不迁移原 scenario_runner 行为树体系。
- 不迁移原 agent wrapper、sensor interface、Zoo 模型接入。
- 不迁移原 leaderboard statistics manager 的复杂恢复/提交逻辑。

当前重要替代能力是：批量 `scene_ego` runner、metadata 驱动地图加载、帧日志、指标计算、批量 summary。

## 后续 agent_ego 注意

当前不少 `scenes/RTBXXX.py` 场景内部仍有自己的 ego 生成和 PID 控制逻辑。真正接算法跑 `agent_ego` 时，需要补兼容逻辑，让场景在 `LEADERBOARD_EGO_MODE=agent_ego` 或 `ROADTAILBENCH_EGO_MODE=agent_ego` 时不再生成或控制 scene-side ego，只保留 NPC、hazard 和场景动态行为。这个改动应优先放在 Bench 仓库的场景运行兼容层或场景公共 helper 中，而不是放到 Zoo adapter 里。
