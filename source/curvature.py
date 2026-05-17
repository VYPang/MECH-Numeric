from __future__ import annotations

from dataclasses import dataclass
import math
import statistics

from .data_loader import TrackData


@dataclass(frozen=True)
class CurvatureProfile:
    method: str
    x: list[float]
    y: list[float]
    s: list[float]
    curvature: list[float]
    total_length_m: float
    uniform_ds_m: float


@dataclass(frozen=True)
class CurvatureComparison:
    raw: CurvatureProfile
    resampled: CurvatureProfile
    mean_abs_difference: float
    max_abs_difference: float
    rms_difference: float


@dataclass(frozen=True)
class CircleValidationResult:
    radius_m: float
    point_count: int
    expected_curvature: float
    mean_abs_error: float
    max_abs_error: float
    rms_error: float


def closed_segment_lengths(x: list[float], y: list[float]) -> list[float]:
    point_count = len(x)
    return [
        math.hypot(x[(index + 1) % point_count] - x[index], y[(index + 1) % point_count] - y[index])
        for index in range(point_count)
    ]


def cumulative_values(values: list[float]) -> list[float]:
    cumulative = [0.0]
    for value in values:
        cumulative.append(cumulative[-1] + value)
    return cumulative


def periodic_central_difference_curvature(x: list[float], y: list[float], total_length_m: float | None = None) -> CurvatureProfile:
    if len(x) != len(y):
        raise ValueError("x and y must contain the same number of points.")
    if len(x) < 3:
        raise ValueError("At least three points are required to compute curvature.")

    point_count = len(x)
    segment_lengths = closed_segment_lengths(x, y)
    total_length = total_length_m if total_length_m is not None else sum(segment_lengths)
    uniform_ds = total_length / point_count
    s = cumulative_values(segment_lengths)[:-1]
    curvature: list[float] = []

    for index in range(point_count):
        previous_index = (index - 1) % point_count
        next_index = (index + 1) % point_count

        dx_ds = (x[next_index] - x[previous_index]) / (2.0 * uniform_ds)
        dy_ds = (y[next_index] - y[previous_index]) / (2.0 * uniform_ds)
        d2x_ds2 = (x[next_index] - 2.0 * x[index] + x[previous_index]) / (uniform_ds**2)
        d2y_ds2 = (y[next_index] - 2.0 * y[index] + y[previous_index]) / (uniform_ds**2)

        denominator = (dx_ds**2 + dy_ds**2) ** 1.5
        if math.isclose(denominator, 0.0):
            curvature.append(0.0)
        else:
            curvature.append((dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator)

    return CurvatureProfile(
        method="periodic central difference",
        x=x,
        y=y,
        s=s,
        curvature=curvature,
        total_length_m=total_length,
        uniform_ds_m=uniform_ds,
    )


def resample_closed_path_by_arc_length(x: list[float], y: list[float], point_count: int) -> tuple[list[float], list[float], list[float], float]:
    if point_count < 3:
        raise ValueError("At least three resampled points are required.")

    segment_lengths = closed_segment_lengths(x, y)
    cumulative = cumulative_values(segment_lengths)
    total_length = cumulative[-1]
    target_s = [index * total_length / point_count for index in range(point_count)]
    extended_x = x + [x[0]]
    extended_y = y + [y[0]]

    resampled_x: list[float] = []
    resampled_y: list[float] = []
    segment_index = 0

    for target in target_s:
        while segment_index < len(segment_lengths) - 1 and cumulative[segment_index + 1] < target:
            segment_index += 1

        segment_start = cumulative[segment_index]
        segment_length = segment_lengths[segment_index]
        ratio = 0.0 if math.isclose(segment_length, 0.0) else (target - segment_start) / segment_length
        resampled_x.append(extended_x[segment_index] + ratio * (extended_x[segment_index + 1] - extended_x[segment_index]))
        resampled_y.append(extended_y[segment_index] + ratio * (extended_y[segment_index + 1] - extended_y[segment_index]))

    return resampled_x, resampled_y, target_s, total_length


def compare_raw_and_resampled_curvature(track: TrackData, resampled_count: int | None = None) -> CurvatureComparison:
    point_count = resampled_count or track.point_count
    raw = periodic_central_difference_curvature(track.x, track.y)
    resampled_x, resampled_y, resampled_s, total_length = resample_closed_path_by_arc_length(track.x, track.y, point_count)
    resampled = periodic_central_difference_curvature(resampled_x, resampled_y, total_length)
    resampled = CurvatureProfile(
        method="arc-length resampled central difference",
        x=resampled.x,
        y=resampled.y,
        s=resampled_s,
        curvature=resampled.curvature,
        total_length_m=resampled.total_length_m,
        uniform_ds_m=resampled.uniform_ds_m,
    )

    comparison_count = min(len(raw.curvature), len(resampled.curvature))
    differences = [raw.curvature[index] - resampled.curvature[index] for index in range(comparison_count)]
    abs_differences = [abs(value) for value in differences]
    rms_difference = math.sqrt(statistics.fmean(value**2 for value in differences)) if differences else 0.0

    return CurvatureComparison(
        raw=raw,
        resampled=resampled,
        mean_abs_difference=statistics.fmean(abs_differences) if abs_differences else 0.0,
        max_abs_difference=max(abs_differences) if abs_differences else 0.0,
        rms_difference=rms_difference,
    )


def validate_curvature_on_circle(radius_m: float = 100.0, point_count: int = 240) -> CircleValidationResult:
    x = [radius_m * math.cos(2.0 * math.pi * index / point_count) for index in range(point_count)]
    y = [radius_m * math.sin(2.0 * math.pi * index / point_count) for index in range(point_count)]
    profile = periodic_central_difference_curvature(x, y, total_length_m=2.0 * math.pi * radius_m)
    expected = 1.0 / radius_m
    errors = [value - expected for value in profile.curvature]
    abs_errors = [abs(value) for value in errors]

    return CircleValidationResult(
        radius_m=radius_m,
        point_count=point_count,
        expected_curvature=expected,
        mean_abs_error=statistics.fmean(abs_errors),
        max_abs_error=max(abs_errors),
        rms_error=math.sqrt(statistics.fmean(value**2 for value in errors)),
    )