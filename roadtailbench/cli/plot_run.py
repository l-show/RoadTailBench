import argparse
import json
from pathlib import Path


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def load_frames(path):
    frames = []
    with Path(path).open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                frames.append(json.loads(line))
    return frames


def main():
    parser = argparse.ArgumentParser(description="Plot a RoadTailBench run summary PNG.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--dpi", default=400, type=int)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    frames = load_frames(run_dir / "roadtailbench_frame_log.jsonl")
    metrics = load_json(run_dir / "roadtailbench_metrics.json")
    config = load_json(run_dir / "roadtailbench_scenario_config.json")
    output = Path(args.output) if args.output else run_dir / "roadtailbench_report.png"

    import matplotlib.pyplot as plt

    times = [f["time"] for f in frames]
    t0 = times[0] if times else 0.0
    rel_t = [t - t0 for t in times]
    ego_xy = [(f["ego"]["location"][0], f["ego"]["location"][1]) for f in frames]
    speeds = [float(f["ego"].get("speed_mps", 0.0)) * 3.6 for f in frames]
    collision_times = []
    for f in frames:
        if f.get("collisions"):
            collision_times.extend([f["time"] - t0] * len(f["collisions"]))

    route = config.get("centerline_route") or config.get("route") or []
    route_xy = [(p[0], p[1]) if isinstance(p, list) else (p["x"], p["y"]) for p in route]
    metric_map = metrics.get("metrics", {})
    score_items = [
        ("route", metric_map.get("route_completion", {}).get("score", 0.0)),
        ("collision", metric_map.get("collision_penalty", {}).get("score", 0.0)),
        ("centerline", metric_map.get("drivable_area", {}).get("score", 0.0)),
        ("interaction", metric_map.get("omnidirectional_interaction_risk", {}).get("score", 0.0)),
        ("comfort", metric_map.get("comfort", {}).get("score", 0.0)),
        ("stability", metric_map.get("control_stability", {}).get("score", 0.0)),
        ("hazard", metric_map.get("long_tail_hazard_response", {}).get("score", 0.0)),
    ]

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    gs = fig.add_gridspec(2, 2)
    ax0 = fig.add_subplot(gs[:, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1])

    if route_xy:
        ax0.plot([p[0] for p in route_xy], [p[1] for p in route_xy], "--", color="#777777", linewidth=1.2, label="metadata centerline")
    if ego_xy:
        ax0.plot([p[0] for p in ego_xy], [p[1] for p in ego_xy], color="#0072B2", linewidth=1.6, label="logged ego")
        ax0.scatter([ego_xy[0][0]], [ego_xy[0][1]], color="#009E73", s=28, label="start")
        ax0.scatter([ego_xy[-1][0]], [ego_xy[-1][1]], color="#D55E00", s=28, label="end")
    for hz in config.get("hazards", []):
        center = hz.get("center", [0.0, 0.0])
        ax0.scatter([center[0]], [center[1]], color="#CC79A7", s=35, marker="x", label="hazard")
    ax0.set_title(f"{config.get('scenario_id', 'unknown')} trajectory")
    ax0.set_xlabel("x (m)")
    ax0.set_ylabel("y (m)")
    ax0.axis("equal")
    ax0.grid(True, alpha=0.25)
    ax0.legend(fontsize=7)

    ax1.plot(rel_t, speeds, color="#0072B2", linewidth=1.2)
    ax1.set_title("ego speed")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("km/h")
    ax1.grid(True, alpha=0.25)
    if collision_times:
        ax1.scatter(collision_times, [0.0] * len(collision_times), color="#D55E00", s=8, label="collision")
        ax1.legend(fontsize=7)

    ax2.bar([k for k, _ in score_items], [v for _, v in score_items], color="#56B4E9")
    ax2.set_ylim(0, 1.05)
    ax2.set_title(f"metric scores; driving={metric_map.get('roadtailbench_driving_score', {}).get('score', 0.0):.2f}")
    ax2.tick_params(axis="x", rotation=35, labelsize=7)
    ax2.grid(True, axis="y", alpha=0.25)

    first_type = frames[0]["ego"].get("type_id", "unknown") if frames else "unknown"
    first_role = frames[0]["ego"].get("role_name", "") if frames else ""
    fig.suptitle(f"Logged ego type={first_type}, role={first_role}, frames={len(frames)}", fontsize=10)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=args.dpi)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
