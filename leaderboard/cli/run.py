import argparse
from argparse import RawTextHelpFormatter

from leaderboard.runtime.runner import CodeScenarioRunner, normalize_ego_mode
from leaderboard.scenarios.discovery import discover_scenarios


def build_argparser():
    parser = argparse.ArgumentParser(
        description="Leaderboard code-scenario runner. Runs RTBXXX.py directly without XML/XOSC.",
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--carla-timeout", default=180.0, type=float, help="CARLA client RPC timeout in seconds.")
    parser.add_argument("--timeout", dest="carla_timeout", type=float, default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    parser.add_argument("--town", default="")
    parser.add_argument("--skip-load-world", action="store_true", help="Use the currently loaded CARLA world instead of calling client.load_world().")
    parser.add_argument("--map-load-mode", choices=["api", "helper"], default="api", help="api: load maps in-process; helper: call scripts/carla_control.py before reconnecting.")
    parser.add_argument("--map-load-timeout", default=300.0, type=float, help="Timeout for map loading in seconds.")
    parser.add_argument("--map-load-sleep", default=3.0, type=float, help="Seconds to wait after a helper map load.")
    parser.add_argument("--spectator-mode", choices=["ego_start", "none"], default="ego_start", help="Set editor spectator to a fixed overhead ego_start view after map load.")
    parser.add_argument("--restore-world-settings", action="store_true", help="Compatibility flag; world settings are always restored to async after each scenario.")
    parser.add_argument("--fixed-delta-seconds", default=0.05, type=float)
    parser.add_argument("--scene-root", required=True)
    parser.add_argument("--metadata-root", default="")
    parser.add_argument("--scenes", default="")
    parser.add_argument("--limit", default=0, type=int, help="Maximum number of discovered scenarios to run. 0 means unlimited.")
    parser.add_argument("--output-root", default="outputs")
    parser.add_argument("--ego-mode", choices=["scene_ego", "script_ego", "agent_ego", "external_ego"], default="scene_ego")
    parser.add_argument("--ego-role-name", default="ego,hero")
    parser.add_argument("--ego-type-id", default="")
    parser.add_argument("--ego-wait-timeout", default=20.0, type=float)
    parser.add_argument("--ego-blueprint", default="vehicle.tesla.model3")
    parser.add_argument("--cleanup-ego", action="store_true")
    parser.add_argument("--agent", default="", help="Python path in module:Class form for agent_ego mode.")
    parser.add_argument("--agent-config", default="")
    parser.add_argument("--max-ticks", default=4000, type=int)
    parser.add_argument("--scenario-timeout", default=0.0, type=float, help="Per-scenario wall-clock timeout in seconds. 0 disables it.")
    parser.add_argument("--tick-wait-timeout", default=5.0, type=float)
    parser.add_argument("--natural-end-distance-m", default=5.0, type=float, help="Finish a scenario when ego remains within this distance of route end.")
    parser.add_argument("--natural-end-min-ticks", default=5, type=int, help="Consecutive ticks required inside the natural-end distance.")
    parser.add_argument("--disable-natural-end", action="store_true", help="Disable natural scenario termination and rely on script exit/max ticks/timeout.")
    parser.add_argument("--runner-drives-scene-ticks", dest="scene_drives_ticks", action="store_false", help="Make the runner call world.tick() even in scene_ego mode. Default scene_ego behavior is passive wait_for_tick().")
    parser.add_argument("--min-ticks-after-script-exit", default=20, type=int)
    parser.add_argument("--actor-log-radius-m", default=120.0, type=float)
    parser.add_argument("--capture-scenario-stdout", action="store_true")
    parser.add_argument("--abort-on-carla-crash", dest="abort_on_carla_crash", action="store_true", default=True, help="Stop the batch immediately when CARLA is unreachable.")
    parser.add_argument("--no-abort-on-carla-crash", dest="abort_on_carla_crash", action="store_false")
    parser.add_argument("--carla-health-timeout", default=3.0, type=float, help="Short timeout used for crash/health probes.")
    parser.add_argument("--process-exit-timeout", default=2.0, type=float, help="Seconds to wait for scene subprocess exit before kill.")
    parser.add_argument("--record-video", action="store_true", help="Record optional runtime videos.")
    parser.add_argument("--record-video-mode", choices=["ego_6cam"], default="ego_6cam", help="Record the ego-mounted six-camera rig.")
    parser.add_argument("--video-fps", default=10.0, type=float)
    parser.add_argument("--video-width", default=1280, type=int)
    parser.add_argument("--video-height", default=720, type=int)
    parser.add_argument("--video-fov", default=90.0, type=float)
    parser.add_argument("--video-image-format", choices=["jpg", "png"], default="jpg")
    parser.add_argument("--video-save-frames", action="store_true")
    parser.add_argument("--video-synth-360", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Discover scenarios and metadata without importing CARLA.")
    return parser


def main():
    args = build_argparser().parse_args()
    args.ego_mode = normalize_ego_mode(args.ego_mode)
    scenarios = discover_scenarios(args.scene_root, args.metadata_root or None, args.scenes)
    if args.limit:
        scenarios = scenarios[: max(0, int(args.limit))]
    print(f"[leaderboard] ego_mode={args.ego_mode}", flush=True)
    print(f"[leaderboard] discovered {len(scenarios)} scenario(s)", flush=True)
    for scenario in scenarios:
        metadata = f" metadata={scenario.metadata_path}" if scenario.metadata_path else " metadata=<missing>"
        print(f"  - {scenario.scene_id}: {scenario.script_path}{metadata}", flush=True)
    if args.dry_run or not scenarios:
        return 0
    summaries = CodeScenarioRunner(args).run(scenarios)
    successful_statuses = {"completed", "completed_timeout"}
    return 1 if any(s.get("status") not in successful_statuses for s in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
