class BaseMetric:
    name = "base"

    def compute(self, frames, config, context=None):
        raise NotImplementedError


class MetricResult:
    @staticmethod
    def make(name, score, details=None):
        return {
            "name": name,
            "score": float(score),
            "details": details or {},
        }
