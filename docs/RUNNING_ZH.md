# 运行 RoadTailBench Leaderboard

## 0. 环境和包名冲突

如果在 `G:\Bench2Drive` 目录下执行 `python -m leaderboard...`，会优先导入旧 Bench2Drive 的 `leaderboard\` 目录，导致 `No module named 'leaderboard.cli'`。推荐先进入本仓库：

```powershell
conda activate Carla-0915
cd G:\Codex\RoadTailBench
python -m pip install -e . --no-build-isolation
```

如果不安装，直接用绝对路径脚本：

```powershell
python G:\Codex\RoadTailBench\run_leaderboard.py --help
```

## 1. `--limit` 是什么

`--scenes RTB116-RTB125` 会发现 RTB116 到 RTB125 共 10 个场景。

`--limit 3` 表示只跑发现结果排序后的前 3 个，也就是 RTB116、RTB117、RTB118。它用于小批量调试，避免一次跑完整范围。

如果想跑全部 RTB116-RTB125，删掉 `--limit 3`，或设置：

```powershell
--limit 0
```

## 2. 检查 CARLA Python API 是否可用

只打开 UE/CARLA 地图但没有进入 Play/Simulate 时，2000 端口通常不可用。这个时候下面命令超时是正常的，不代表 Python 脚本坏了。必须进入 Play/Simulate 后，CARLA Python RPC 才会响应。

```powershell
python G:\Codex\RoadTailBench\scripts\carla_control.py `
  --host localhost `
  --port 2000 `
  --timeout 30 `
  --wait `
  --print-world
```

如果这个命令超时，说明当前还没有可用的 CARLA Python RPC。先启动 UE/CARLA 并进入 Play/Simulate。

## 3. 启动 CARLA/UE Editor

本仓库提供了启动脚本：

```powershell
powershell -ExecutionPolicy Bypass -File G:\Codex\RoadTailBench\scripts\launch_carla_editor.ps1 `
  -CarlaRoot D:\carla0.9.15 `
  -MapName RTB116 `
  -CommandPlay `
  -Visible
```

说明：

- `-CarlaRoot` 是 CARLA 源码根目录，本机示例是 `D:\carla0.9.15`。
- `-MapName RTB116` 会设置启动钩子需要的地图名环境变量。
- `-CommandPlay` 会设置 `ROADTAILBENCH_COMMAND_PLAY=1` / `LEADERBOARD_COMMAND_PLAY=1`，供 UE Python 启动钩子读取后执行 `unreal.EditorLevelLibrary.editor_play_simulate()`。
- `-Visible` 表示显示启动窗口；不传则隐藏窗口。

如果脚本找不到 VS2019，需要显式传：

```powershell
-VsDevCmd "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"
```

注意：`-CommandPlay` 依赖 UE Python 启动钩子。本仓库根目录提供了 `init_unreal.py`，启动脚本会把仓库根目录加入 `PYTHONPATH`。如果你的 UE Python 插件会执行 `PYTHONPATH` 上的 `init_unreal.py`，就会自动调用 `unreal.EditorLevelLibrary.editor_play_simulate()`；如果没有执行，需要在 UE 项目 Python startup scripts 里手动配置这个文件。

## 4. 已启动 CARLA 后切换地图

如果 CARLA 已经启动并进入 Play，可以用 CARLA Python API 切图：

```powershell
python G:\Codex\RoadTailBench\scripts\carla_control.py `
  --host localhost `
  --port 2000 `
  --timeout 300 `
  --wait `
  --map RTB116 `
  --sleep-after-load 3 `
  --print-world
```

如果这里也卡住或超时，问题在 CARLA/UE 地图加载本身，Leaderboard runner 里也会卡在同一步。

## 5. Dry Run

Dry run 不导入 CARLA，只检查场景脚本和 metadata：

```powershell
leaderboard-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --dry-run
```

不安装时：

```powershell
python G:\Codex\RoadTailBench\run_leaderboard.py `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --dry-run
```

## 6. 自动批量运行 scene_ego

默认模式是在 runner 进程内调用 `client.load_world()`：

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --ego-mode scene_ego `
  --carla-timeout 30 `
  --map-load-timeout 120 `
  --spectator-mode ego_start `
  --scenario-timeout 60 `
  --output-root G:\Codex\RoadTailBench\outputs
```

如果你怀疑进程内 `load_world()` 卡住，可以让 runner 每次切图时调用独立 helper 进程：

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --ego-mode scene_ego `
  --map-load-mode helper `
  --map-load-timeout 300 `
  --map-load-sleep 3 `
  --spectator-mode ego_start `
  --scenario-timeout 60 `
  --output-root G:\Codex\RoadTailBench\outputs
```

重要：`--scenario-timeout` 是场景脚本启动后的现实时间预算。到 60 秒会正常截断，summary 状态是 `completed_timeout`，不是失败。它不一定能打断 CARLA 自己正在卡住的 `load_world()`；如果卡在切图阶段，优先调大 `--map-load-timeout`，并用第 4 节的 `carla_control.py --map ...` 单独排查。

默认不会在每个场景结束后把 CARLA 切回异步模式，避免 scene 脚本管理同步模式时导致 UE/CARLA 不稳定。只有确实需要时才加 `--restore-world-settings`。

`--spectator-mode ego_start` 会在切图后把观察者放到 metadata 的 `ego_start` 上方，方便你看到道路附近。若不想改视角，用 `--spectator-mode none`。

`scene_ego` 模式下，`RTBXXX.py` 自己生成并控制 ego。Leaderboard runner 只寻找 ego、记录帧、计算指标。每个场景开始前会根据 metadata 的 `town` 自动切换 CARLA 地图；如果传 `--skip-load-world`，则使用当前地图。

输出文件夹由 `--output-root` 控制。每个场景会自动生成：

```text
<output-root>\RTB116_YYYYMMDD_HHMMSS\
```

## 7. 重新计算指标

`leaderboard_metrics.json` 在每次运行结束时已经自动生成。重新计算指标主要用于：你修改了 metrics 代码、想用同一份 frame log 重新评分，或想把结果另存为新文件。

```powershell
leaderboard-eval `
  --frames G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\metrics_recomputed.json
```

不安装时：

```powershell
python G:\Codex\RoadTailBench\eval_leaderboard.py `
  --frames G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038\metrics_recomputed.json
```

## 8. 画图

安装绘图库：

```powershell
conda install matplotlib
```

默认把图保存到运行目录里的 `leaderboard_report.png`：

```powershell
leaderboard-plot `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038
```

手动指定图片保存位置和文件名：

```powershell
leaderboard-plot `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038 `
  --output G:\Codex\RoadTailBench\outputs\plots\RTB116_report.png `
  --dpi 400
```

不安装时：

```powershell
python G:\Codex\RoadTailBench\plot_leaderboard.py `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260617_160038 `
  --output G:\Codex\RoadTailBench\outputs\plots\RTB116_report.png
```

所以画图保存目录不用改代码，直接改 `--output`。如果不传 `--output`，默认保存在 `--run-dir` 指定的运行文件夹里。

文档里的 `RTB116_20260617_160038` 只是示例。实际使用时要替换为你 `outputs` 下真实存在的运行目录名。

## 9. agent_ego 状态

`agent_ego` 是后续模型接入阶段。当前可以保留参数兼容，但不要把它当作已完成路径；多数场景仍需要根据 `LEADERBOARD_EGO_MODE=agent_ego` / `ROADTAILBENCH_EGO_MODE=agent_ego` 屏蔽脚本内部 ego 生成和控制。
