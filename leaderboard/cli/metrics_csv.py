import argparse
from pathlib import Path

from leaderboard.core.io import load_json
from leaderboard.core.metrics_csv import save_metrics_csv


def main():
    parser = argparse.ArgumentParser(description="Convert leaderboard metrics JSON to a flat CSV file.")
    parser.add_argument("--metrics", default="", help="Path to leaderboard_metrics.json.")
    parser.add_argument("--run-dir", default="", help="Run directory containing leaderboard_metrics.json.")
    parser.add_argument("--output", default="", help="CSV output path. Defaults to <metrics-stem>.csv.")
    args = parser.parse_args()

    if not args.metrics and not args.run_dir:
        raise SystemExit("Provide either --metrics or --run-dir.")
    metrics_path = Path(args.metrics) if args.metrics else Path(args.run_dir) / "leaderboard_metrics.json"
    if "<" in str(metrics_path) or ">" in str(metrics_path):
        raise SystemExit(f"metrics path contains a placeholder: {metrics_path}")
    if not metrics_path.exists():
        raise SystemExit(f"metrics JSON does not exist: {metrics_path}")
    output = Path(args.output) if args.output else metrics_path.with_suffix(".csv")
    save_metrics_csv(output, load_json(metrics_path))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
