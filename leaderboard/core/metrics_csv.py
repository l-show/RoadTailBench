import csv
import json
from pathlib import Path


def metric_rows(result):
    scenario_id = result.get("scenario_id", "unknown")
    route_id = result.get("route_id", scenario_id)
    for name, metric in result.get("metrics", {}).items():
        yield {
            "scenario_id": scenario_id,
            "route_id": route_id,
            "metric_name": name,
            "score": metric.get("score"),
            "details_json": json.dumps(metric.get("details", {}), ensure_ascii=False, sort_keys=True),
        }


def save_metrics_csv(path, result):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(metric_rows(result))
    fieldnames = ["scenario_id", "route_id", "metric_name", "score", "details_json"]
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
