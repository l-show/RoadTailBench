import argparse
from pathlib import Path

from leaderboard.core.io import load_json, load_jsonl, save_json
from leaderboard.core.metrics_csv import save_metrics_csv
from leaderboard.metrics.evaluator import evaluate_leaderboard


def reject_placeholder(path, label):
    text = str(path)
    if "<" in text or ">" in text:
        raise SystemExit(
            f"{label} contains a placeholder: {text}\n"
            "Replace <run> with a real output directory name, for example RTB116_20260617_160038."
        )
    if not Path(text).exists():
        raise SystemExit(f"{label} does not exist: {text}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate leaderboard metrics from frame log and config JSON.")
    parser.add_argument("--frames", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--csv-output", default="", help="Optional CSV output path for flattened metric scores.")
    args = parser.parse_args()
    reject_placeholder(args.frames, "--frames")
    reject_placeholder(args.config, "--config")
    frames = load_jsonl(args.frames)
    config = load_json(args.config)
    result = evaluate_leaderboard(frames, config)
    save_json(args.output, result)
    if args.csv_output:
        save_metrics_csv(args.csv_output, result)
        print(f"[leaderboard] wrote {args.csv_output}", flush=True)
    print(f"[leaderboard] wrote {args.output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
