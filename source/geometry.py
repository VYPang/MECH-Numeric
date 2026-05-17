from __future__ import annotations

from dataclasses import dataclass
import math
import statistics

from .data_loader import TrackData


@dataclass(frozen=True)
class TrackAudit:
    point_count: int
    closure_gap_m: float
    total_length_m: float
    segment_lengths_m: list[float]
    min_segment_m: float
    max_segment_m: float
    mean_segment_m: float
    stdev_segment_m: float
    width_right_min_m: float
    width_right_max_m: float
    width_right_mean_m: float
    width_left_min_m: float
    width_left_max_m: float
    width_left_mean_m: float
    spacing_cv: float
    outlier_segment_indices: list[int]


def compute_segment_lengths(track: TrackData) -> list[float]:
    segment_lengths: list[float] = []
    for index in range(track.point_count):
        next_index = (index + 1) % track.point_count
        dx = track.x[next_index] - track.x[index]
        dy = track.y[next_index] - track.y[index]
        segment_lengths.append(math.hypot(dx, dy))
    return segment_lengths


def cumulative_arc_length(segment_lengths: list[float]) -> list[float]:
    cumulative = [0.0]
    for length in segment_lengths:
        cumulative.append(cumulative[-1] + length)
    return cumulative


def compute_closure_gap(track: TrackData) -> float:
    return math.hypot(track.x[0] - track.x[-1], track.y[0] - track.y[-1])


def detect_segment_outliers(segment_lengths: list[float]) -> list[int]:
    if len(segment_lengths) < 2:
        return []
    mean_value = statistics.fmean(segment_lengths)
    stdev_value = statistics.pstdev(segment_lengths)
    if math.isclose(stdev_value, 0.0):
        return []
    threshold = mean_value + 2.5 * stdev_value
    return [index for index, value in enumerate(segment_lengths) if value > threshold]


def audit_track(track: TrackData) -> TrackAudit:
    segment_lengths = compute_segment_lengths(track)
    if not segment_lengths:
        raise ValueError(f"Track '{track.name}' does not contain enough points for audit.")

    min_segment = min(segment_lengths)
    max_segment = max(segment_lengths)
    mean_segment = statistics.fmean(segment_lengths)
    stdev_segment = statistics.pstdev(segment_lengths) if len(segment_lengths) > 1 else 0.0
    spacing_cv = stdev_segment / mean_segment if not math.isclose(mean_segment, 0.0) else 0.0

    return TrackAudit(
        point_count=track.point_count,
        closure_gap_m=compute_closure_gap(track),
        total_length_m=sum(segment_lengths),
        segment_lengths_m=segment_lengths,
        min_segment_m=min_segment,
        max_segment_m=max_segment,
        mean_segment_m=mean_segment,
        stdev_segment_m=stdev_segment,
        width_right_min_m=min(track.width_right),
        width_right_max_m=max(track.width_right),
        width_right_mean_m=statistics.fmean(track.width_right),
        width_left_min_m=min(track.width_left),
        width_left_max_m=max(track.width_left),
        width_left_mean_m=statistics.fmean(track.width_left),
        spacing_cv=spacing_cv,
        outlier_segment_indices=detect_segment_outliers(segment_lengths),
    )