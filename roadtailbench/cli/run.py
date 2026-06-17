import argparse
from argparse import RawTextHelpFormatter

from roadtailbench.runtime.runner import CodeScenarioRunner, normalize_ego_mode
from roadtailbench.scenarios.discovery import discover_scenarios


def build_argparser():
    parser = argparse.ArgumentParser(
        description="RoadTailBench code-scenario runner. Runs RTBXXX.py directly without XML/XOSC.",
        formatter_class=RawTextHelpFormatter,
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout", default=180.0, type=float)
    parser.add_argument("--town", default="")
    parser.add_argument("--skip-load-world", action="store_true", help="Use the currently loaded CARLA world instead of calling client.load_world().")
    parser.add_argument("--fixed-delta-seconds", default=0.05, type=float)
    parser.add_argument("--scene-root", required=True)
    parser.add_argument("--metadata-root", default="")
    parser.add_argument("--scenes", default="")
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
    parser.add_argument("--tick-wait-timeout", default=5.0, type=float)
    parser.add_argument("--runner-drives-scene-ticks", dest="scene_drives_ticks", action="store_false", help="Make the runner call world.tick() even in scene_ego mode. Default scene_ego behavior is passive wait_for_tick().")
    parser.add_argument("--min-ticks-after-script-exit", default=20, type=int)
    parser.add_argument("--actor-log-radius-m", default=120.0, type=float)
    parser.add_argument("--capture-scenario-stdout", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Discover scenarios and metadata without importing CARLA.")
    return parser


def main():
    args = build_argparser().parse_args()
    args.ego_mode = normalize_ego_mode(args.ego_mode)
    scenarios = discover_scenarios(args.scene_root, args.metadata_root or None, args.scenes)
    print(f"[RoadTailBench] ego_mode={args.ego_mode}", flush=True)
    print(f"[RoadTailBench] discovered {len(scenarios)} scenario(s)", flush=True)
    for scenario in scenarios:
        metadata = f" metadata={scenario.metadata_path}" if scenario.metadata_path else " metadata=<missing>"
        print(f"  - {scenario.scene_id}: {scenario.script_path}{metadata}", flush=True)
    if args.dry_run or not scenarios:
        return 0
    summaries = CodeScenarioRunner(args).run(scenarios)
    return 1 if any(s.get("status") != "completed" for s in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
