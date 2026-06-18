import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from leaderboard.core.trajectory import reference_xy


CORE_METRIC_NAMES = [
    "route_completion",
    "collision_penalty",
    "driving_efficiency",
    "speed_appropriateness",
    "drivable_area",
    "omnidirectional_interaction_risk",
    "road_engineering_hazard_adaptation",
    "comfort",
    "control_stability",
    "long_tail_hazard_response",
]


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


def point_xy(value):
    if isinstance(value, dict):
        loc = value.get("location", value)
        if isinstance(loc, dict):
            return (float(loc.get("x", 0.0)), float(loc.get("y", 0.0)))
        return (float(loc[0]), float(loc[1]))
    return (float(value[0]), float(value[1]))


def rel_times(frames):
    times = [float(f.get("time", 0.0)) for f in frames]
    t0 = times[0] if times else 0.0
    return [t - t0 for t in times]


def norm3(values):
    vals = list(values or [0.0, 0.0, 0.0])
    while len(vals) < 3:
        vals.append(0.0)
    return math.sqrt(vals[0] * vals[0] + vals[1] * vals[1] + vals[2] * vals[2])


def collect_collision_points(frames):
    points = []
    times = rel_times(frames)
    for index, frame in enumerate(frames):
        ego_loc = frame.get("ego", {}).get("location", [0.0, 0.0, 0.0])
        for collision in frame.get("collisions", []):
            loc = collision.get("location") or ego_loc
            other_xy = None
            other_id = collision.get("other_actor_id")
            for actor in frame.get("actors", []):
                if actor.get("id") == other_id and actor.get("location"):
                    other_loc = actor["location"]
                    other_xy = (float(other_loc[0]), float(other_loc[1]))
                    break
            if loc:
                points.append({
                    "time": times[index] if index < len(times) else 0.0,
                    "xy": (float(loc[0]), float(loc[1])),
                    "other_xy": other_xy,
                    "other_actor_id": collision.get("other_actor_id"),
                    "other_actor_type": collision.get("other_actor_type", "unknown"),
                })
    return points


def collect_actor_tracks(frames):
    tracks = defaultdict(list)
    for frame in frames:
        for actor in frame.get("actors", []):
            type_id = str(actor.get("type_id", ""))
            if not (type_id.startswith("vehicle.") or type_id.startswith("walker.")):
                continue
            loc = actor.get("location")
            if loc:
                tracks[int(actor.get("id", -1))].append((float(loc[0]), float(loc[1]), type_id))
    return tracks


def route_xy(config):
    route = reference_xy(config)
    return route


def plot_trajectory(frames, config, output, plt, dpi):
    ego_xy = [(float(f["ego"]["location"][0]), float(f["ego"]["location"][1])) for f in frames if f.get("ego")]
    route = route_xy(config)
    actor_tracks = collect_actor_tracks(frames)
    collisions = collect_collision_points(frames)

    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    if route:
        ax.plot([p[0] for p in route], [p[1] for p in route], "--", color="#6E6E6E", linewidth=1.2, label="reference trajectory")
        ax.scatter([route[-1][0]], [route[-1][1]], color="#CC79A7", s=36, marker="*", label="ego goal")
    for track in actor_tracks.values():
        if len(track) < 2:
            continue
        ax.plot([p[0] for p in track], [p[1] for p in track], color="#999999", linewidth=0.8, alpha=0.35)
    if actor_tracks:
        ax.plot([], [], color="#999999", linewidth=0.8, alpha=0.65, label="dynamic actors")
    if ego_xy:
        ax.plot([p[0] for p in ego_xy], [p[1] for p in ego_xy], color="#0072B2", linewidth=1.8, label="ego")
        ax.scatter([ego_xy[0][0]], [ego_xy[0][1]], color="#009E73", s=32, label="ego start")
        ax.scatter([ego_xy[-1][0]], [ego_xy[-1][1]], color="#D55E00", s=32, label="ego end")
    hazard_labeled = False
    for hazard in config.get("hazards", []):
        center = hazard.get("center", [0.0, 0.0])
        ax.scatter([center[0]], [center[1]], color="#E69F00", s=40, marker="x", label=None if hazard_labeled else "hazard")
        hazard_labeled = True
    if collisions:
        ax.scatter(
            [p["xy"][0] for p in collisions],
            [p["xy"][1] for p in collisions],
            color="#D55E00",
            edgecolors="black",
            linewidths=0.4,
            s=46,
            marker="X",
            label="ego collision",
        )
        other_points = [p["other_xy"] for p in collisions if p.get("other_xy")]
        if other_points:
            ax.scatter(
                [p[0] for p in other_points],
                [p[1] for p in other_points],
                facecolors="none",
                edgecolors="#D55E00",
                linewidths=1.0,
                s=56,
                marker="o",
                label="other collision actor",
            )
    ax.set_title(f"{config.get('scenario_id', 'unknown')} trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_timeseries(frames, output, plt, dpi):
    times = rel_times(frames)
    speeds = [float(f.get("ego", {}).get("speed_mps", 0.0)) * 3.6 for f in frames]
    accels = [norm3(f.get("ego", {}).get("acceleration", [0.0, 0.0, 0.0])) for f in frames]
    collision_times = [p["time"] for p in collect_collision_points(frames)]
    controls = [f.get("ego", {}).get("control", {}) for f in frames]
    throttle = [float(c.get("throttle", 0.0)) for c in controls]
    brake = [float(c.get("brake", 0.0)) for c in controls]
    steer = [float(c.get("steer", 0.0)) for c in controls]

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
    ax0.plot(times, speeds, color="#0072B2", linewidth=1.2, label="speed km/h")
    ax0b = ax0.twinx()
    ax0b.plot(times, accels, color="#009E73", linewidth=1.0, alpha=0.85, label="accel m/s2")
    if collision_times:
        ymax = max(speeds) if speeds else 1.0
        ax0.scatter(collision_times, [ymax * 0.05] * len(collision_times), color="#D55E00", s=18, marker="X", label="collision")
    ax0.set_ylabel("speed (km/h)")
    ax0b.set_ylabel("acceleration (m/s2)")
    ax0.grid(True, alpha=0.25)
    lines, labels = ax0.get_legend_handles_labels()
    lines_b, labels_b = ax0b.get_legend_handles_labels()
    ax0.legend(lines + lines_b, labels + labels_b, fontsize=8, loc="upper right")

    ax1.plot(times, throttle, color="#0072B2", linewidth=1.1, label="throttle")
    ax1.plot(times, brake, color="#D55E00", linewidth=1.1, label="brake")
    ax1.plot(times, steer, color="#CC79A7", linewidth=1.1, label="steer")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("control")
    ax1.set_ylim(-1.05, 1.05)
    ax1.grid(True, alpha=0.25)
    ax1.legend(fontsize=8)
    fig.suptitle("ego time series", fontsize=11)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_metric_scores(metrics, output, plt, dpi):
    metric_map = metrics.get("metrics", {})
    labels = [name.replace("_", "\n") for name in CORE_METRIC_NAMES]
    values = [float(metric_map.get(name, {}).get("score", 0.0)) for name in CORE_METRIC_NAMES]
    driving = float(metric_map.get("leaderboard_driving_score", {}).get("score", 0.0))

    fig, ax = plt.subplots(figsize=(12, 5), constrained_layout=True)
    ax.bar(labels, values, color="#56B4E9")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("score")
    ax.set_title(f"10 core metric scores; driving score={driving:.2f}")
    ax.tick_params(axis="x", labelsize=8)
    ax.grid(True, axis="y", alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ability_breakdown(metrics, output, plt, dpi):
    metric_map = metrics.get("metrics", {})
    ability = metric_map.get("ability_score", {})
    group_scores = ability.get("details", {}).get("group_scores", {})
    subtype_scores = ability.get("details", {}).get("subtype_scores", {})
    hazard_zones = metric_map.get("road_engineering_hazard_adaptation", {}).get("details", {}).get("zone_scores", [])
    hazard_responses = metric_map.get("long_tail_hazard_response", {}).get("details", {}).get("hazard_responses", [])

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), constrained_layout=True)
    group_labels = ["A", "B", "C"]
    group_values = [group_scores.get(label) for label in group_labels]
    axes[0].bar(group_labels, [0.0 if v is None else float(v) for v in group_values], color="#009E73")
    axes[0].set_ylim(0, 1.05)
    axes[0].set_title(f"ability groups; ability score={float(ability.get('score', 0.0)):.2f}")
    axes[0].grid(True, axis="y", alpha=0.25)

    subtype_labels = list(subtype_scores.keys())
    subtype_values = [float(subtype_scores[label]) for label in subtype_labels]
    axes[1].bar(subtype_labels or ["no ability subtypes"], subtype_values or [0.0], color="#56B4E9")
    axes[1].set_ylim(0, 1.05)
    axes[1].set_title("ability subtype scores")
    axes[1].tick_params(axis="x", rotation=25, labelsize=8)
    axes[1].grid(True, axis="y", alpha=0.25)

    zone_labels = [str(item.get("subtype") or item.get("id") or index) for index, item in enumerate(hazard_zones)]
    zone_values = [float(item.get("score", 0.0)) for item in hazard_zones]
    axes[2].bar(zone_labels or ["no hazard zones"], zone_values or [0.0], color="#E69F00")
    axes[2].set_ylim(0, 1.05)
    axes[2].set_title("road engineering hazard sub-scores")
    axes[2].tick_params(axis="x", rotation=25, labelsize=8)
    axes[2].grid(True, axis="y", alpha=0.25)

    response_labels = [str(item.get("type") or item.get("id") or index) for index, item in enumerate(hazard_responses)]
    response_values = [float(item.get("score", 0.0)) for item in hazard_responses]
    axes[3].bar(response_labels or ["no hazard responses"], response_values or [0.0], color="#CC79A7")
    axes[3].set_ylim(0, 1.05)
    axes[3].set_title("long-tail hazard response sub-scores")
    axes[3].tick_params(axis="x", rotation=25, labelsize=8)
    axes[3].grid(True, axis="y", alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_overview(frames, metrics, config, output, plt, dpi):
    times = rel_times(frames)
    ego_xy = [(f["ego"]["location"][0], f["ego"]["location"][1]) for f in frames if f.get("ego")]
    speeds = [float(f.get("ego", {}).get("speed_mps", 0.0)) * 3.6 for f in frames]
    metric_map = metrics.get("metrics", {})
    score_items = [(name, metric_map.get(name, {}).get("score", 0.0)) for name in CORE_METRIC_NAMES]

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    gs = fig.add_gridspec(2, 2)
    ax0 = fig.add_subplot(gs[:, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1])
    route = route_xy(config)
    if route:
        ax0.plot([p[0] for p in route], [p[1] for p in route], "--", color="#777777", linewidth=1.2, label="reference trajectory")
    if ego_xy:
        ax0.plot([p[0] for p in ego_xy], [p[1] for p in ego_xy], color="#0072B2", linewidth=1.6, label="ego")
        ax0.scatter([ego_xy[0][0]], [ego_xy[0][1]], color="#009E73", s=28, label="start")
        ax0.scatter([ego_xy[-1][0]], [ego_xy[-1][1]], color="#D55E00", s=28, label="end")
    ax0.axis("equal")
    ax0.grid(True, alpha=0.25)
    ax0.legend(fontsize=7)
    ax0.set_title("trajectory")
    ax1.plot(times, speeds, color="#0072B2", linewidth=1.2)
    ax1.set_title("ego speed")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("km/h")
    ax1.grid(True, alpha=0.25)
    ax2.bar([k.replace("_", "\n") for k, _ in score_items], [v for _, v in score_items], color="#56B4E9")
    ax2.set_ylim(0, 1.05)
    ax2.set_title(f"metrics; driving={metric_map.get('leaderboard_driving_score', {}).get('score', 0.0):.2f}")
    ax2.tick_params(axis="x", labelsize=6)
    ax2.grid(True, axis="y", alpha=0.25)
    first_type = frames[0]["ego"].get("type_id", "unknown") if frames else "unknown"
    first_role = frames[0]["ego"].get("role_name", "") if frames else ""
    fig.suptitle(f"Logged ego type={first_type}, role={first_role}, frames={len(frames)}", fontsize=10)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot leaderboard run summary PNGs.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--dpi", default=400, type=int)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    if "<" in str(run_dir) or ">" in str(run_dir):
        raise SystemExit(
            f"--run-dir contains a placeholder: {run_dir}\n"
            "Replace <run> with a real output directory name, for example RTB116_20260617_160038."
        )
    frames_path = run_dir / "leaderboard_frame_log.jsonl"
    metrics_path = run_dir / "leaderboard_metrics.json"
    config_path = run_dir / "leaderboard_scenario_config.json"
    missing = [str(path) for path in (frames_path, metrics_path, config_path) if not path.exists()]
    if missing:
        raise SystemExit("Missing required run files:\n" + "\n".join(missing))

    frames = load_frames(frames_path)
    metrics = load_json(metrics_path)
    config = load_json(config_path)

    import matplotlib.pyplot as plt

    outputs = [
        run_dir / "leaderboard_trajectory.png",
        run_dir / "leaderboard_ego_timeseries.png",
        run_dir / "leaderboard_metric_scores.png",
        run_dir / "leaderboard_ability_breakdown.png",
    ]
    for output in outputs:
        output.parent.mkdir(parents=True, exist_ok=True)
    plot_trajectory(frames, config, outputs[0], plt, args.dpi)
    plot_timeseries(frames, outputs[1], plt, args.dpi)
    plot_metric_scores(metrics, outputs[2], plt, args.dpi)
    plot_ability_breakdown(metrics, outputs[3], plt, args.dpi)

    if args.output:
        overview = Path(args.output)
        overview.parent.mkdir(parents=True, exist_ok=True)
        plot_overview(frames, metrics, config, overview, plt, args.dpi)
        outputs.append(overview)

    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
