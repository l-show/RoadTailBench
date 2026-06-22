from .ability_score import AbilityScoreMetric
from .collision_penalty import CollisionPenaltyMetric
from .comfort import ComfortMetric
from .composite_score import CompositeScoreMetric
from .control_stability import ControlStabilityMetric
from .driving_efficiency import DrivingEfficiencyMetric
from .energy_efficiency import EnergyEfficiencyMetric
from .interaction_risk import InteractionRiskMetric
from .long_tail_hazard_response import LongTailHazardResponseMetric
from .route_completion import RouteCompletionMetric
from .speed_appropriateness import SpeedAppropriatenessMetric
from .trajectory_adherence import TrajectoryAdherenceMetric

CORE_METRICS = [
    RouteCompletionMetric,
    CollisionPenaltyMetric,
    DrivingEfficiencyMetric,
    SpeedAppropriatenessMetric,
    TrajectoryAdherenceMetric,
    InteractionRiskMetric,
    ComfortMetric,
    ControlStabilityMetric,
    EnergyEfficiencyMetric,
    LongTailHazardResponseMetric,
]
