import argparse

from leaderboard.core.io import load_json, load_jsonl, save_json
from leaderboard.metrics.evaluator import evaluate_leaderboard


def main():
    parser = argparse.ArgumentParser(description="Evaluate leaderboard metrics from frame log and config JSON.")
    parser.add_argument("--frames", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    frames = load_jsonl(args.frames)
    config = load_json(args.config)
    save_json(args.output, evaluate_leaderboard(frames, config))
    print(f"[leaderboard] wrote {args.output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
