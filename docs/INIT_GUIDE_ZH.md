# RoadTailBench /init 指南

## 当前仓库分工

- `G:\Codex\RoadTailBench`：Bench 本体。这里放 CARLA 场景脚本、场景元数据、运行器、帧日志和指标计算。后续做 benchmark 运行、场景发现、指标评估、leaderboard 接回时，优先看这个仓库。
- `G:\Codex\RoadTailBench-Zoo`：Zoo 接口层。这里放模型 adapter，把外部自动驾驶模型的输入输出转成 RoadTailBench 的统一闭环协议。只有在 `agent_ego` 模式或改模型接入时才需要动它。

## 重要目录

Bench 仓库：

- `scenes/`：`RTBXXX.py` 场景代码。场景脚本负责 CARLA 关卡内的天气、NPC、hazard 和动态行为。
- `metadata/`：`RTBXXX.json` 场景元数据。用于地图、ego 起终点、ego 蓝图、参考路线和 hazard 标注。
- `roadtailbench/runtime/`：CARLA 连接、加载地图、启动场景脚本、寻找或生成 ego、记录帧。
- `roadtailbench/metrics/`：从帧日志和配置计算指标。
- `roadtailbench/cli/`：`rtb-run` 和 `rtb-eval` 命令。
- `outputs/`：运行输出目录，不提交。

Zoo 仓库：

- `roadtailbench_zoo/protocol/`：adapter 基类和 `RTBControl` 等协议类型。
- `roadtailbench_zoo/adapters/`：不同模型的 adapter。
- `configs/`、`checkpoints/`：模型配置和本地权重占位；权重不要提交。

## 基础安装

```powershell
pip install -e G:\Codex\RoadTailBench
pip install -e G:\Codex\RoadTailBench-Zoo
```

如果只跑 `scene_ego`，第二条不是必需；如果跑 `agent_ego`，需要安装 Zoo 或保证 adapter 可被 Python import。

## 场景发现检查

```powershell
rtb-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --dry-run
```

这一步不导入 CARLA，用来确认 `RTBXXX.py` 和 `RTBXXX.json` 能匹配。

## scene_ego 运行方式

`scene_ego` 是默认模式。场景脚本自己生成和控制 ego，runner 只负责识别该车、记录帧并计算指标。识别优先级来自代码和元数据：

- `ego_role_names` / `ego_role_name`，默认命令参数是 `ego,hero`。
- `ego_type_id` / `ego_blueprint`。
- `ego_start` 附近的车辆位置匹配。

示例：

```powershell
rtb-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116 `
  --ego-mode scene_ego `
  --output-root G:\Codex\RoadTailBench\outputs
```

这个模式不需要模型 adapter。只要场景中 ego 的车名、role name 或蓝图能和 metadata 对上，指标计算就可以继续。

## agent_ego 运行方式

`agent_ego` 用于接入外部算法。runner 先根据 metadata 里的 `ego_start` 和 `ego_blueprint` / `ego_type_id` 生成 ego，再每 tick 调用 Zoo adapter 输出控制量。

示例：

```powershell
rtb-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116 `
  --ego-mode agent_ego `
  --agent roadtailbench_zoo.adapters.rule_based:RuleBasedAdapter `
  --output-root G:\Codex\RoadTailBench\outputs
```

注意：当前不少 `scenes/RTBXXX.py` 场景内部仍有自己的 ego 生成和 PID 控制逻辑。真正接算法跑 `agent_ego` 时，需要补一层兼容逻辑，让场景在 `ROADTAILBENCH_EGO_MODE=agent_ego` 时不再生成或控制 scene-side ego，只保留 NPC、hazard 和场景动态行为。这个改动应优先放在 Bench 仓库的场景运行兼容层或场景公共 helper 中，而不是放到 Zoo adapter 里。

## 输出和指标

每次运行会在 `outputs/<scene>_<timestamp>/` 下生成：

- `roadtailbench_frame_log.jsonl`
- `roadtailbench_scenario_config.json`
- `roadtailbench_metrics.json`
- `roadtailbench_run_summary.json`

重新计算指标：

```powershell
rtb-eval `
  --frames G:\Codex\RoadTailBench\outputs\<run>\roadtailbench_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\<run>\roadtailbench_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\<run>\metrics.json
```

## 后续开发原则

- Bench 是运行和评分入口；Zoo 是模型接口层。
- 不把 Bench2Drive、Bench2DriveZoo 或不兼容许可的上游代码 vendored 进来。
- 场景脚本和元数据必须同名，例如 `RTB116.py` / `RTB116.json`。
- 修改 discovery、metadata schema 或 metrics 时跑 `pytest` 和 dry-run。
- 修改 CARLA runtime 时，优先补不依赖 CARLA 的单元测试；真实 CARLA 验证步骤写清楚。
