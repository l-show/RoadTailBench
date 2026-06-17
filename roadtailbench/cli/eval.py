import argparse

from roadtailbench.core.io import load_json, load_jsonl, save_json
from roadtailbench.metrics.evaluator import evaluate_roadtailbench


def main():
    parser = argparse.ArgumentParser(description="Evaluate RoadTailBench metrics from frame log and config JSON.")
    parser.add_argument("--frames", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    frames = load_jsonl(args.frames)
    config = load_json(args.config)
    save_json(args.output, evaluate_roadtailbench(frames, config))
    print(f"[RoadTailBench] wrote {args.output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
