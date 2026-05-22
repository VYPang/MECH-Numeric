"""Track-level metric definitions for the dashboard.

Metrics are split into two layers so baseline rankings can be reused once
trajectory-optimized results are added:

- Geometry metrics depend only on the path samples (vehicle-independent).
- Performance metrics depend on a solved speed profile (vehicle-dependent).

A composite difficulty score is also exposed. It normalizes a small set of
metrics across the available track set so that rankings stay meaningful
when the vehicle model or solver is changed.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Iterable

import math
import statistics


@dataclass(frozen=True)
class GeometryMetrics:
    total_length_m: float
    closure_gap_m: float
    spacing_cv: float
    mean_abs_curvature: float
    max_abs_curvature: float
    rms_curvature: float
    corner_severity_index: float
    width_mean_m: float
    width_min_m: float


@dataclass(frozen=True)
class PerformanceMetrics:
    lap_time_s: float
    average_speed_mps: float
    max_speed_mps: float
    min_speed_mps: float
    speed_stdev_mps: float
    mean_abs_long_accel_mps2: float
    accel_fraction: float
    brake_fraction: float
    coast_fraction: float
    phase_transition_count: int
    lateral_limit_fraction: float


@dataclass(frozen=True)
class TrackMetrics:
    track_name: str
    method: str
    geometry: GeometryMetrics
    performance: PerformanceMetrics

    def to_dict(self) -> dict:
        return {
            "track_name": self.track_name,
            "method": self.method,
            "geometry": asdict(self.geometry),
            "performance": asdict(self.performance),
        }


# ---------------------------------------------------------------------------
# Geometry metrics
# ---------------------------------------------------------------------------

def compute_geometry_metrics(
    *,
    total_length_m: float,
    closure_gap_m: float,
    spacing_cv: float,
    curvature: list[float],
    width_right: list[float],
    width_left: list[float],
) -> GeometryMetrics:
    abs_curvature = [abs(value) for value in curvature]
    mean_abs_curvature = statistics.fmean(abs_curvature) if abs_curvature else 0.0
    max_abs_curvature = max(abs_curvature) if abs_curvature else 0.0
    rms_curvature = (
        math.sqrt(statistics.fmean(value * value for value in curvature))
        if curvature
        else 0.0
    )
    corner_severity_index = _corner_severity_index(abs_curvature)

    combined_width = [a + b for a, b in zip(width_right, width_left)]
    width_mean = statistics.fmean(combined_width) if combined_width else 0.0
    width_min = min(combined_width) if combined_width else 0.0

    return GeometryMetrics(
        total_length_m=total_length_m,
        closure_gap_m=closure_gap_m,
        spacing_cv=spacing_cv,
        mean_abs_curvature=mean_abs_curvature,
        max_abs_curvature=max_abs_curvature,
        rms_curvature=rms_curvature,
        corner_severity_index=corner_severity_index,
        width_mean_m=width_mean,
        width_min_m=width_min,
    )


def _corner_severity_index(abs_curvature: list[float]) -> float:
    """Sum of curvature above the 75th percentile, scaled by sample count.

    This emphasizes the few tightest curvature regions per lap rather than
    smoothing them out via a global mean.
    """
    if not abs_curvature:
        return 0.0
    sorted_values = sorted(abs_curvature)
    cutoff_index = max(0, int(0.75 * len(sorted_values)))
    threshold = sorted_values[cutoff_index]
    severe = [value for value in abs_curvature if value >= threshold]
    if not severe:
        return 0.0
    return sum(severe) / len(abs_curvature)


# ---------------------------------------------------------------------------
# Performance metrics
# ---------------------------------------------------------------------------

def compute_performance_metrics(
    *,
    lap_time_s: float,
    total_length_m: float,
    final_speed_mps: list[float],
    longitudinal_accel_mps2: list[float],
    lateral_accel_mps2: list[float],
    ay_max_mps2: float,
    accel_threshold_mps2: float = 0.25,
    lateral_limit_fraction_threshold: float = 0.95,
) -> PerformanceMetrics:
    if not final_speed_mps:
        raise ValueError("final_speed_mps must not be empty.")
    if not longitudinal_accel_mps2:
        raise ValueError("longitudinal_accel_mps2 must not be empty.")

    average_speed = total_length_m / lap_time_s if lap_time_s > 0 else 0.0
    max_speed = max(final_speed_mps)
    min_speed = min(final_speed_mps)
    speed_stdev = (
        statistics.pstdev(final_speed_mps) if len(final_speed_mps) > 1 else 0.0
    )

    abs_long_accel = [abs(value) for value in longitudinal_accel_mps2]
    mean_abs_long_accel = statistics.fmean(abs_long_accel)

    accel_count = sum(1 for value in longitudinal_accel_mps2 if value > accel_threshold_mps2)
    brake_count = sum(1 for value in longitudinal_accel_mps2 if value < -accel_threshold_mps2)
    coast_count = len(longitudinal_accel_mps2) - accel_count - brake_count
    total = len(longitudinal_accel_mps2)
    accel_fraction = accel_count / total
    brake_fraction = brake_count / total
    coast_fraction = coast_count / total

    phase_transitions = _phase_transition_count(longitudinal_accel_mps2, accel_threshold_mps2)

    if ay_max_mps2 > 0:
        lateral_limit_fraction = (
            sum(
                1
                for value in lateral_accel_mps2
                if value >= lateral_limit_fraction_threshold * ay_max_mps2
            )
            / len(lateral_accel_mps2)
        )
    else:
        lateral_limit_fraction = 0.0

    return PerformanceMetrics(
        lap_time_s=lap_time_s,
        average_speed_mps=average_speed,
        max_speed_mps=max_speed,
        min_speed_mps=min_speed,
        speed_stdev_mps=speed_stdev,
        mean_abs_long_accel_mps2=mean_abs_long_accel,
        accel_fraction=accel_fraction,
        brake_fraction=brake_fraction,
        coast_fraction=coast_fraction,
        phase_transition_count=phase_transitions,
        lateral_limit_fraction=lateral_limit_fraction,
    )


def _phase_transition_count(longitudinal_accel: list[float], threshold: float) -> int:
    previous_state = _phase_state(longitudinal_accel[0], threshold)
    transitions = 0
    for value in longitudinal_accel[1:]:
        current_state = _phase_state(value, threshold)
        if current_state != previous_state:
            transitions += 1
            previous_state = current_state
    return transitions


def _phase_state(value: float, threshold: float) -> str:
    if value > threshold:
        return "accelerating"
    if value < -threshold:
        return "braking"
    return "neutral"


# ---------------------------------------------------------------------------
# Composite difficulty score
# ---------------------------------------------------------------------------

# Weights are intentionally small and transparent. Each contributing metric
# is min-max normalized across the available tracks, then combined linearly.
DIFFICULTY_WEIGHTS: dict[str, float] = {
    "rms_curvature": 0.30,
    "corner_severity_index": 0.25,
    "speed_stdev_mps": 0.15,
    "mean_abs_long_accel_mps2": 0.15,
    "lateral_limit_fraction": 0.15,
}


def compute_difficulty_scores(metrics_by_track: dict[str, TrackMetrics]) -> dict[str, float]:
    if not metrics_by_track:
        return {}

    raw_values: dict[str, dict[str, float]] = {key: {} for key in DIFFICULTY_WEIGHTS}
    for track_name, metrics in metrics_by_track.items():
        raw_values["rms_curvature"][track_name] = metrics.geometry.rms_curvature
        raw_values["corner_severity_index"][track_name] = metrics.geometry.corner_severity_index
        raw_values["speed_stdev_mps"][track_name] = metrics.performance.speed_stdev_mps
        raw_values["mean_abs_long_accel_mps2"][track_name] = (
            metrics.performance.mean_abs_long_accel_mps2
        )
        raw_values["lateral_limit_fraction"][track_name] = (
            metrics.performance.lateral_limit_fraction
        )

    normalized: dict[str, dict[str, float]] = {}
    for metric_name, values in raw_values.items():
        normalized[metric_name] = _min_max_normalize(values)

    scores: dict[str, float] = {}
    for track_name in metrics_by_track:
        total = 0.0
        for metric_name, weight in DIFFICULTY_WEIGHTS.items():
            total += weight * normalized[metric_name][track_name]
        scores[track_name] = total
    return scores


def _min_max_normalize(values: dict[str, float]) -> dict[str, float]:
    if not values:
        return {}
    minimum = min(values.values())
    maximum = max(values.values())
    if math.isclose(minimum, maximum):
        return {key: 0.5 for key in values}
    return {key: (value - minimum) / (maximum - minimum) for key, value in values.items()}


def difficulty_metric_components() -> Iterable[str]:
    return DIFFICULTY_WEIGHTS.keys()
