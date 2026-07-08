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
    "trajectory_adherence",
    "proximity_risk",
    "comfort",
    "control_stability",
    "energy_efficiency",
    "long_tail_hazard_response",
    "behavior_capability_score",
    "hazard_capability_score",
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


def apply_plot_style(plt, style):
    if style == "nc":
        plt.rcParams.update({
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#222222",
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "grid.color": "#D0D0D0",
            "grid.linewidth": 0.4,
            "grid.alpha": 0.35,
            "font.size": 8,
            "axes.titlesize": 9,
            "axes.labelsize": 8,
            "legend.fontsize": 7,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "lines.linewidth": 1.1,
        })
    elif style == "ieee":
        plt.rcParams.update({
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "font.size": 8,
            "axes.titlesize": 8,
            "axes.labelsize": 8,
            "legend.fontsize": 7,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "lines.linewidth": 1.0,
        })


def smooth_series(values, times, window_s):
    if window_s <= 0 or len(values) < 3:
        return list(values)
    cleaned = [float(v) if v is not None and not math.isnan(float(v)) else float("nan") for v in values]
    out = []
    for index, time in enumerate(times):
        lo_t = time - 0.5 * window_s
        hi_t = time + 0.5 * window_s
        window = [cleaned[i] for i, t in enumerate(times) if lo_t <= t <= hi_t and not math.isnan(cleaned[i])]
        out.append(sum(window) / len(window) if window else cleaned[index])
    return out


def ego_longitudinal_accel(frame):
    ego = frame.get("ego", {})
    accel = ego.get("acceleration", [0.0, 0.0, 0.0])
    yaw = math.radians(float(ego.get("rotation", [0.0, 0.0, 0.0])[2]))
    return float(accel[0]) * math.cos(yaw) + float(accel[1]) * math.sin(yaw)


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


def plot_timeseries(frames, output, plt, dpi, smooth_window_s=0.5):
    times = rel_times(frames)
    speeds = [float(f.get("ego", {}).get("speed_mps", 0.0)) * 3.6 for f in frames]
    long_accels = [ego_longitudinal_accel(f) for f in frames]
    abs_accels = [norm3(f.get("ego", {}).get("acceleration", [0.0, 0.0, 0.0])) for f in frames]
    speeds = smooth_series(speeds, times, smooth_window_s)
    long_accels = smooth_series(long_accels, times, smooth_window_s)
    abs_accels = smooth_series(abs_accels, times, smooth_window_s)
    collision_times = [p["time"] for p in collect_collision_points(frames)]
    controls = [f.get("ego", {}).get("control", {}) for f in frames]
    throttle = [float(c.get("throttle", 0.0)) for c in controls]
    brake = [float(c.get("brake", 0.0)) for c in controls]
    steer = [float(c.get("steer", 0.0)) for c in controls]

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
    ax0.plot(times, speeds, color="#0072B2", linewidth=1.2, label="speed km/h")
    ax0b = ax0.twinx()
    ax0b.plot(times, abs_accels, color="#B0B0B0", linewidth=0.8, alpha=0.45, label="|accel| m/s2")
    ax0b.plot(times, long_accels, color="#009E73", linewidth=1.0, alpha=0.9, label="long. accel m/s2")
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


def plot_proximity_timeseries(frames, metrics, output, plt, dpi, smooth_window_s=0.5):
    times = rel_times(frames)
    prox = metrics.get("metrics", {}).get("proximity_risk", {}).get("details", {})
    longitudinal, lateral, ttc, tlc, env_distance = [], [], [], [], []
    long_danger, long_safe = [], []
    for frame in frames:
        p = frame.get("proximity", {})
        ego = frame.get("ego", {})
        speed = float(ego.get("speed_mps", 0.0))
        danger = float(prox.get("actor_longitudinal_distance_danger_m", 3.0))
        safe = max(danger, speed * float(prox.get("actor_longitudinal_headway_safe_s", 1.5)))
        long_danger.append(danger)
        long_safe.append(safe)
        env_max = p.get("raycast_max_distance_m", 30.0)
        env_min_hit = float(prox.get("environment_raycast_min_hit_distance_m", p.get("raycast_min_hit_distance_m", 1.0)))
        env_hits = [h for h in (p.get("environment_hits") or []) if float(h.get("distance_m", env_max)) >= env_min_hit]
        env_distance.append(min((float(h.get("distance_m", env_max)) for h in env_hits), default=env_max))
        longitudinal.append(float("nan"))
        lateral.append(float("nan"))
        ttc.append(float("nan"))
        tlc.append(float("nan"))
        actors = frame.get("actors", [])
        if not ego or not actors:
            continue
        ex, ey = ego.get("location", [0.0, 0.0])[:2]
        yaw = math.radians(float(ego.get("rotation", [0.0, 0.0, 0.0])[2]))
        fx, fy = math.cos(yaw), math.sin(yaw)
        lx, ly = -fy, fx
        nearest = None
        for actor in actors:
            loc = actor.get("location", [0.0, 0.0])
            dx, dy = float(loc[0]) - float(ex), float(loc[1]) - float(ey)
            d = math.hypot(dx, dy)
            if nearest is None or d < nearest[0]:
                nearest = (d, dx, dy, actor)
        if nearest:
            _, dx, dy, actor = nearest
            long_d = dx * fx + dy * fy
            lat_d = dx * lx + dy * ly
            longitudinal[-1] = abs(long_d)
            lateral[-1] = abs(lat_d)
            ev = ego.get("velocity", [0.0, 0.0])
            av = actor.get("velocity", [0.0, 0.0])
            closing = (float(ev[0]) - float(av[0])) * fx + (float(ev[1]) - float(av[1])) * fy
            if long_d > 0.0 and closing > 0.1:
                ttc[-1] = long_d / closing
            lateral_closing = abs((float(ev[0]) - float(av[0])) * lx + (float(ev[1]) - float(av[1])) * ly)
            if lateral_closing > 0.1:
                clearance = max(0.0, abs(lat_d) - float(prox.get("actor_lateral_clearance_danger_m", 1.0)))
                tlc[-1] = clearance / lateral_closing

    def clean(vals):
        return [float(v) if v is not None else float("nan") for v in vals]

    time_cap = 8.0
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
    long_plot = smooth_series(clean(longitudinal), times, smooth_window_s)
    env_plot = smooth_series(clean(env_distance), times, smooth_window_s)
    ttc_plot = [min(v, time_cap) if not math.isnan(v) else v for v in smooth_series(clean(ttc), times, smooth_window_s)]
    lat_plot = smooth_series(clean(lateral), times, smooth_window_s)
    tlc_plot = [min(v, time_cap) if not math.isnan(v) else v for v in smooth_series(clean(tlc), times, smooth_window_s)]
    ax0.plot(times, long_plot, color="#4C78A8", linewidth=1.1, label="actor longitudinal distance")
    ax0.plot(times, env_plot, color="#54A24B", linewidth=1.0, alpha=0.85, label="environment ray distance")
    ax0.plot(times, long_danger, color="#D62728", linestyle="--", linewidth=0.8, label="longitudinal danger")
    ax0.plot(times, long_safe, color="#F2A541", linestyle=":", linewidth=0.8, label="longitudinal safe")
    ax0b = ax0.twinx()
    ax0b.plot(times, ttc_plot, color="#E45756", linewidth=1.0, label="actor longitudinal TTC")
    ax0.set_ylabel("distance (m)")
    ax0b.set_ylabel("TTC (s, capped at 8)")
    ax0.grid(True, alpha=0.25)
    lines, labels = ax0.get_legend_handles_labels()
    lines_b, labels_b = ax0b.get_legend_handles_labels()
    ax0.legend(lines + lines_b, labels + labels_b, fontsize=8)

    ax1.plot(times, lat_plot, color="#B279A2", linewidth=1.1, label="actor lateral distance")
    ax1b = ax1.twinx()
    ax1b.plot(times, tlc_plot, color="#72B7B2", linewidth=1.0, label="actor lateral TLC")
    if prox.get("actor_lateral_clearance_danger_m") is not None:
        ax1.axhline(float(prox["actor_lateral_clearance_danger_m"]), color="#D55E00", linestyle="--", linewidth=0.9, label="lateral danger")
    if prox.get("default_lane_width_m") is not None:
        ax1.axhline(float(prox["default_lane_width_m"]) * 0.5, color="#E69F00", linestyle=":", linewidth=0.9, label="half lane width")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("lateral distance (m)")
    ax1b.set_ylabel("TLC (s, capped at 8)")
    ax1.grid(True, alpha=0.25)
    lines, labels = ax1.get_legend_handles_labels()
    lines_b, labels_b = ax1b.get_legend_handles_labels()
    ax1.legend(lines + lines_b, labels + labels_b, fontsize=8)
    fig.suptitle("proximity risk time series", fontsize=11)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_metric_scores(metrics, output, plt, dpi):
    metric_map = metrics.get("metrics", {})
    names = [name for name in CORE_METRIC_NAMES if name in metric_map]
    labels = [name.replace("_", "\n") for name in names]
    values = [float(metric_map.get(name, {}).get("score", 0.0)) for name in names]
    driving = float(metric_map.get("leaderboard_driving_score", {}).get("score", 0.0))

    fig, ax = plt.subplots(figsize=(12, 5), constrained_layout=True)
    ax.bar(labels, values, color="#56B4E9")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("score")
    ax.set_title(f"core metric scores; driving score={driving:.2f}")
    ax.tick_params(axis="x", labelsize=8)
    ax.grid(True, axis="y", alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_ability_breakdown(metrics, output, plt, dpi):
    metric_map = metrics.get("metrics", {})
    ability = metric_map.get("ability_score", {})
    behavior = metric_map.get("behavior_capability_score", {})
    hazard = metric_map.get("hazard_capability_score", {})
    hazard_responses = metric_map.get("long_tail_hazard_response", {}).get("details", {}).get("hazard_responses", [])

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    group_labels = ["ego_action", "hazard_type", "aggregate"]
    group_values = [
        behavior.get("score"),
        hazard.get("score"),
        ability.get("score"),
    ]
    axes[0].bar(group_labels, [0.0 if v is None else float(v) for v in group_values], color="#4C78A8")
    axes[0].set_ylim(0, 1.05)
    axes[0].set_title(f"capability scores; aggregate={float(ability.get('score', 0.0)):.2f}")
    axes[0].grid(True, axis="y", alpha=0.25)

    response_labels = [str(item.get("type") or item.get("id") or index) for index, item in enumerate(hazard_responses)]
    response_values = [float(item.get("score", 0.0)) for item in hazard_responses]
    axes[1].bar(response_labels or ["no hazard responses"], response_values or [0.0], color="#CC79A7")
    axes[1].set_ylim(0, 1.05)
    axes[1].set_title("long-tail hazard response")
    axes[1].tick_params(axis="x", rotation=25, labelsize=8)
    axes[1].grid(True, axis="y", alpha=0.25)
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def plot_overview(frames, metrics, config, output, plt, dpi, smooth_window_s=0.5, layout="paper"):
    times = rel_times(frames)
    ego_xy = [(f["ego"]["location"][0], f["ego"]["location"][1]) for f in frames if f.get("ego")]
    speeds = smooth_series([float(f.get("ego", {}).get("speed_mps", 0.0)) * 3.6 for f in frames], times, smooth_window_s)
    long_accels = smooth_series([ego_longitudinal_accel(f) for f in frames], times, smooth_window_s)
    metric_map = metrics.get("metrics", {})
    score_items = [(name, metric_map.get(name, {}).get("score", 0.0)) for name in CORE_METRIC_NAMES]

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    gs = fig.add_gridspec(3, 2)
    ax0 = fig.add_subplot(gs[:, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    axp = fig.add_subplot(gs[1, 1])
    ax2 = fig.add_subplot(gs[2, 1])
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
    ax1b = ax1.twinx()
    ax1b.plot(times, long_accels, color="#009E73", linewidth=1.0, alpha=0.85)
    ax1.set_title("ego state")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("km/h")
    ax1b.set_ylabel("m/s2")
    ax1.grid(True, alpha=0.25)
    prox_score = metric_map.get("proximity_risk", {}).get("score", 0.0)
    collision_score = metric_map.get("collision_penalty", {}).get("score", 0.0)
    axp.bar(["proximity", "collision"], [prox_score, collision_score], color=["#54A24B", "#E45756"])
    axp.set_ylim(0, 1.05)
    axp.set_title("safety summary")
    axp.grid(True, axis="y", alpha=0.25)
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
    parser.add_argument("--smooth-window-s", default=0.5, type=float)
    parser.add_argument("--no-smooth", action="store_true")
    parser.add_argument("--style", choices=["nc", "ieee", "default"], default="nc")
    parser.add_argument("--overview-layout", choices=["paper", "compact"], default="paper")
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
    apply_plot_style(plt, args.style)
    smooth_window_s = 0.0 if args.no_smooth else max(0.0, float(args.smooth_window_s))

    outputs = [
        run_dir / "leaderboard_trajectory.png",
        run_dir / "leaderboard_ego_timeseries.png",
        run_dir / "leaderboard_metric_scores.png",
        run_dir / "leaderboard_ability_breakdown.png",
        run_dir / "leaderboard_proximity_timeseries.png",
    ]
    for output in outputs:
        output.parent.mkdir(parents=True, exist_ok=True)
    plot_trajectory(frames, config, outputs[0], plt, args.dpi)
    plot_timeseries(frames, outputs[1], plt, args.dpi, smooth_window_s)
    plot_metric_scores(metrics, outputs[2], plt, args.dpi)
    plot_ability_breakdown(metrics, outputs[3], plt, args.dpi)
    plot_proximity_timeseries(frames, metrics, outputs[4], plt, args.dpi, smooth_window_s)

    if args.output:
        overview = Path(args.output)
        overview.parent.mkdir(parents=True, exist_ok=True)
        plot_overview(frames, metrics, config, overview, plt, args.dpi, smooth_window_s, args.overview_layout)
        outputs.append(overview)

    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
