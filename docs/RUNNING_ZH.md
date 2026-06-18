# RoadTailBench 运行说明

本文档说明 CARLA 启动、地图切换、批量仿真、崩溃处理、观察者视角、录像、评估和画图命令。建议所有命令都在本仓库根目录执行：

```powershell
conda activate Carla-0915
cd G:\Codex\RoadTailBench
pip install -e .
```

如果没有安装 console scripts，也可以用 `python -m leaderboard.cli.run`、`python -m leaderboard.cli.eval`、`python -m leaderboard.cli.plot_run`、`python -m leaderboard.cli.video`。

## 1. 启动 CARLA Editor 并切换地图

推荐使用仓库脚本：

```powershell
powershell -ExecutionPolicy Bypass -File G:\Codex\RoadTailBench\scripts\launch_carla_editor.ps1 `
  -CarlaRoot D:\carla0.9.15 `
  -MapName RTB116 `
  -CommandPlay `
  -Visible `
  -WaitTimeout 300 `
  -SleepAfterLoad 3
```

参数说明：

- `-CarlaRoot`: CARLA 源码/Editor 根目录，例如 `D:\carla0.9.15`。
- `-VsDevCmd`: VS2019 Native Tools 脚本路径。默认会自动搜索常见位置，找不到时手动传入。
- `-MapName`: 要加载的地图名，例如 `RTB116`。现在脚本会在 Editor/Play 启动后调用 `carla_control.py --map <MapName>` 真正切图。
- `-HostName`: CARLA RPC host，默认 `localhost`。
- `-Port`: CARLA RPC port，默认 `2000`。
- `-WaitTimeout`: 等待 CARLA RPC 端口可用的秒数，默认 `300`。
- `-SleepAfterLoad`: 地图 load_world 后额外等待秒数，默认 `3`。
- `-CommandPlay`: 设置 Play/Simulate 启动环境变量；和 `-MapName` 一起使用。
- `-Visible`: 显示 Editor 启动窗口；不传则隐藏。
- `-SkipMapLoad`: 只启动 Editor/Play，不自动调用 `carla_control.py` 切图。

如果 CARLA 已经启动并进入 Play，可单独切图：

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

`carla_control.py` 参数：

- `--host`: CARLA RPC host，默认 `127.0.0.1`。
- `--port`: CARLA RPC port，默认 `2000`。
- `--timeout`: 等待端口、连接和 load_world 的超时时间。
- `--wait`: 先等待端口可连接，再创建 CARLA client。
- `--map`: 要加载的地图名。可以是 `RTB116` 或完整 Unreal 路径。
- `--sleep-after-load`: load_world 后等待秒数。
- `--print-world`: 打印当前 world map 名称。
- `--command-play`: 在 Unreal Python 环境里调用 Play/Simulate；普通外部 Python 不使用。

## 2. Dry Run

Dry run 只检查场景发现和 metadata，不连接 CARLA：

```powershell
leaderboard-run `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 3 `
  --dry-run
```

## 3. 批量运行 scene_ego

基础命令：

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 0 `
  --ego-mode scene_ego `
  --carla-timeout 30 `
  --map-load-timeout 120 `
  --spectator-mode ego_start `
  --scenario-timeout 180 `
  --output-root G:\Codex\RoadTailBench\outputs
```

### 场景选择和路径

- `--scene-root`: 必填。`RTBXXX.py` 场景脚本目录。
- `--metadata-root`: metadata JSON 目录。为空时只运行脚本，不加载 metadata。
- `--scenes`: 场景选择表达式。支持单个、逗号和范围，例如 `RTB116`、`RTB116,RTB118`、`RTB116-RTB125`。
- `--limit`: 最多运行多少个已发现场景。`0` 表示不限制。
- `--output-root`: 输出根目录。每个场景会生成 `<output-root>\RTBXXX_YYYYMMDD_HHMMSS\`。
- `--dry-run`: 只发现场景和 metadata，不导入 CARLA，不运行仿真。

### CARLA 连接和地图加载

- `--host`: CARLA host，默认 `localhost`。
- `--port`: CARLA RPC port，默认 `2000`。
- `--carla-timeout`: CARLA client RPC 超时，默认 `180` 秒。地图大或 Editor 卡顿时可调大。
- `--town`: 强制使用某个地图名；不传时优先用 metadata 的 `town`。
- `--skip-load-world`: 不调用 `client.load_world()`，直接使用当前 CARLA world。适合手动切图后调试。
- `--map-load-mode api|helper`: 地图加载方式。`api` 在 runner 进程里调用 `client.load_world()`；`helper` 调用 `scripts/carla_control.py` 作为独立进程切图。
- `--map-load-timeout`: 地图加载超时，默认 `300` 秒。
- `--map-load-sleep`: helper 切图后额外等待秒数，默认 `3`。
- `--fixed-delta-seconds`: runner 设置同步模式时使用的固定步长，默认 `0.05`。
- `--restore-world-settings`: 兼容保留参数。当前 runner 每个场景结束都会尝试恢复异步；如果 CARLA 已崩溃会跳过可能阻塞的恢复调用。

### Ego 模式和发现

- `--ego-mode scene_ego|script_ego|agent_ego|external_ego`: ego 控制模式。`scene_ego` 是当前主模式，场景脚本自己生成并控制 ego；`script_ego` 是别名。`agent_ego`/`external_ego` 用于后续模型接入。
- `--ego-role-name`: 查找 ego 的 role_name 列表，默认 `ego,hero`。
- `--ego-type-id`: 没有 role_name 时按车型匹配 ego。
- `--ego-wait-timeout`: 等待场景脚本生成 ego 的秒数，默认 `20`。
- `--ego-blueprint`: `agent_ego` 模式下 runner 自己生成 ego 时使用的车型，默认 `vehicle.tesla.model3`。
- `--cleanup-ego`: `agent_ego` 模式结束后销毁 runner 生成的 ego。CARLA 不可用时会跳过销毁，避免卡死。
- `--agent`: `agent_ego` 模式下模型 adapter，格式 `module:Class`。
- `--agent-config`: 传给 adapter 的配置字符串或路径。

### 仿真终止

- `--max-ticks`: 单场景最多采集 tick 数，默认 `4000`。
- `--scenario-timeout`: 单场景 wall-clock 超时秒数。`0` 表示禁用。超时是兜底，不替代自然结束。
- `--tick-wait-timeout`: `wait_for_tick()` 的等待超时，默认 `5` 秒。
- `--natural-end-distance-m`: ego 距离终点多近算到达，默认 `5` 米。
- `--natural-end-min-ticks`: 连续多少帧在终点范围内才结束，默认 `5`。
- `--disable-natural-end`: 禁用自然结束，只依赖脚本退出、max ticks 或 timeout。
- `--runner-drives-scene-ticks`: 默认 `scene_ego` 下 runner 被动 `wait_for_tick()`，场景脚本自己 `world.tick()`。加这个参数后 runner 主动 `world.tick()`。
- `--min-ticks-after-script-exit`: 场景脚本退出后继续采集多少 tick，默认 `20`。

自然结束只看 ego：ego 到终点或 ego 被销毁。其他 actor 销毁不会结束场景。

### 观察者视角

- `--spectator-mode ego_start`: 切图后把 spectator 放到 metadata `ego_start` 上方约 45m，并用俯视角向下看。这个位置只设置一次，不会在运行中跟随 ego。
- `--spectator-mode none`: runner 不修改 spectator。

如果场景脚本内部也在设置 spectator，运行过程中仍可能覆盖这个初始俯视角。runner 现在不再提供 ego 跟随观察者，避免额外依赖 ego 发现逻辑。

### Actor 记录和 stdout

- `--actor-log-radius-m`: 记录 ego 周围多大半径内的动态/静态 actor，默认 `120` 米。
- `--capture-scenario-stdout`: 兼容参数。当前 runner 会把场景脚本 stdout/stderr 写入每个 run 目录的 `scenario_stdout.log`，便于崩溃后追踪。

每个 run 目录主要输出：

```text
leaderboard_frame_log.jsonl
leaderboard_scenario_config.json
leaderboard_metrics.json
leaderboard_run_summary.json
scenario_stdout.log
```

### CARLA 崩溃检测

- `--abort-on-carla-crash`: 默认开启。检测到 CARLA fatal error、端口断开或 RPC 不可用后，当前场景标记 `carla_crashed`，并停止后续场景。
- `--no-abort-on-carla-crash`: 不因 CARLA 崩溃停止 batch。一般不建议使用，因为后续场景会继续超时。
- `--carla-health-timeout`: 崩溃健康检查短超时，默认 `3` 秒。
- `--process-exit-timeout`: 终止场景子进程后等待秒数，默认 `2` 秒，超时则 kill。

崩溃 summary 会记录：

- `status=carla_crashed`
- `failure_class=carla_unavailable`
- `last_rpc`
- `carla_alive_before`
- `carla_alive_after`
- `scenario_stdout_log`

## 4. 录像

默认不录像。当前只保留 ego 绑定的 6 路 RGB 相机录像，不再提供 spectator 单视角录像。

```powershell
leaderboard-run `
  --host localhost --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116 `
  --ego-mode scene_ego `
  --spectator-mode ego_start `
  --record-video `
  --record-video-mode ego_6cam `
  --video-fps 10 `
  --output-root G:\Codex\RoadTailBench\outputs
```

也可以在批量命令里直接加：

```powershell
--record-video --record-video-mode ego_6cam --video-fps 10
```

录像参数：

- `--record-video`: 启用录像。
- `--record-video-mode ego_6cam`: 录前、左前、右前、后、左后、右后 6 个 ego 车载 RGB camera。当前唯一模式，保留这个参数是为了命令显式。
- `--video-fps`: 录像帧率，默认 `10`。建议低于仿真 20 FPS，减少文件体积。
- `--video-width`: 相机宽度，默认 `1280`。
- `--video-height`: 相机高度，默认 `720`。
- `--video-fov`: 相机 FOV，默认 `90`。
- `--video-image-format jpg|png`: 保存帧时的图片格式，默认 `jpg`。
- `--video-save-frames`: 保存图片帧而不是只写 mp4。需要 360 合成时使用。
- `--video-synth-360`: 场景结束后自动合成 360 视频。只有配合 `--video-save-frames` 和 6 路相机才有效。

输出位置：

```text
<run-dir>\video\CAM_FRONT.mp4
<run-dir>\video\CAM_FRONT_LEFT.mp4
<run-dir>\video\CAM_FRONT_RIGHT.mp4
<run-dir>\video\CAM_BACK.mp4
<run-dir>\video\CAM_BACK_LEFT.mp4
<run-dir>\video\CAM_BACK_RIGHT.mp4
<run-dir>\video\video_manifest.json
```

如果 `cv2` 或 `numpy` 不可用，录像模块会在 `video_manifest.json` 里记录错误。安装方式：

```powershell
pip install opencv-python numpy
```

## 5. 360 视频合成

如果运行时用了：

```powershell
--record-video --record-video-mode ego_6cam --video-save-frames
```

可以后处理合成 360 视频：

```powershell
leaderboard-video `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000 `
  --mode synth-360 `
  --fps 10
```

`leaderboard-video` 参数：

- `--run-dir`: 单个场景输出目录，必须包含 `video/CAM_*` 帧目录。
- `--mode synth-360`: 合成 360 全景，目前唯一模式。
- `--fps`: 输出视频帧率。

## 6. 重新计算指标

```powershell
leaderboard-eval `
  --frames G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000\leaderboard_frame_log.jsonl `
  --config G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000\leaderboard_scenario_config.json `
  --output G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000\metrics_recomputed.json
```

`leaderboard-eval` 参数：

- `--frames`: frame log 路径。
- `--config`: scenario config 路径。
- `--output`: 输出 metrics JSON 路径。

## 7. 画图

```powershell
leaderboard-plot `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000 `
  --dpi 400
```

指定额外 overview 图：

```powershell
leaderboard-plot `
  --run-dir G:\Codex\RoadTailBench\outputs\RTB116_20260618_120000 `
  --output G:\Codex\RoadTailBench\outputs\plots\RTB116_overview.png `
  --dpi 400
```

`leaderboard-plot` 参数：

- `--run-dir`: 单个场景输出目录，必须包含 frame log、metrics 和 config。
- `--output`: 可选，额外输出 overview 图。默认四张详细图直接写入 `--run-dir`。
- `--dpi`: 图片 DPI，默认 `400`。

默认输出：

```text
leaderboard_trajectory.png
leaderboard_ego_timeseries.png
leaderboard_metric_scores.png
leaderboard_ability_breakdown.png
```

## 8. RTB122 崩溃排查建议

如果 RTB122 触发关卡蓝图、trigger box、落石 physics 后导致 CARLA fatal error，runner 现在会立即停止 batch，并在当前 run 目录写出：

```text
leaderboard_run_summary.json
scenario_stdout.log
```

建议单独运行 RTB122：

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB122 `
  --ego-mode scene_ego `
  --scenario-timeout 180 `
  --spectator-mode ego_start `
  --abort-on-carla-crash `
  --output-root G:\Codex\RoadTailBench\outputs
```

如果 summary 的 `last_rpc` 是 `find_scene_ego.get_actors`、`world.wait_for_tick` 或 `client.load_world`，说明 runner 看到的是 CARLA 已不可用；真正 fatal error 原因需要看 UE/CARLA Editor 日志和 `scenario_stdout.log`。对于关卡蓝图场景，优先确认 trigger box 触发的 actor、碰撞体和 simulate physics 在 CARLA 0.9.15 中稳定。

当前推荐的完整批量测试命令如下。crash 检测默认开启，不需要额外传 `--abort-on-carla-crash`；CARLA 崩溃后 runner 会保存当前场景 summary/stdout，并停止后续场景，避免终端长时间卡死。

```powershell
leaderboard-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes `
  --metadata-root G:\Codex\RoadTailBench\metadata `
  --scenes RTB116-RTB125 `
  --limit 0 `
  --ego-mode scene_ego `
  --carla-timeout 30 `
  --map-load-timeout 180 `
  --map-load-sleep 3 `
  --spectator-mode ego_start `
  --scenario-timeout 180 `
  --tick-wait-timeout 5 `
  --natural-end-distance-m 5 `
  --natural-end-min-ticks 5 `
  --actor-log-radius-m 120 `
  --record-video `
  --record-video-mode ego_6cam `
  --video-fps 10 `
  --video-width 1280 `
  --video-height 720 `
  --video-fov 90 `
  --video-image-format jpg `
  --video-save-frames `
  --video-synth-360 `
  --output-root G:\Codex\RoadTailBench\outputs
```

这条命令会记录 ego 绑定的 6 路相机，并在每个场景结束后尝试合成 360 全景视频。因为 `--video-save-frames` 会保留 6 路原始图片帧，磁盘占用会明显变大；如果只需要 6 路 mp4、不需要 360 合成，可以删掉：

```powershell
  --video-save-frames `
  --video-synth-360 `
```

当前已经删除的旧参数不要再使用：

- `--spectator-mode ego_follow`
- `--record-video-mode spectator`
- `--record-video-mode both`

只有你明确想让 CARLA 崩溃后还继续尝试后续场景，才传 `--no-abort-on-carla-crash`。一般不建议这么做，因为 CARLA 进程不可用时后续场景只会继续报连接失败或超时。
