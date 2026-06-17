from . import CORE_METRICS
from .ability_score import AbilityScoreMetric
from .composite_score import CompositeScoreMetric


def evaluate_leaderboard(frames, config):
    results = {}
    for metric_cls in CORE_METRICS:
        metric = metric_cls()
        result = metric.compute(frames, config, results)
        results[result["name"]] = result
    for metric in (CompositeScoreMetric(), AbilityScoreMetric()):
        result = metric.compute(frames, config, results)
        results[result["name"]] = result
    return {
        "scenario_id": config.get("scenario_id", "unknown"),
        "route_id": config.get("route_id", config.get("scenario_id", "unknown")),
        "metrics": results,
    }
