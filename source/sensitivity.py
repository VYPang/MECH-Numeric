from __future__ import annotations

from dataclasses import dataclass
import math
from time import perf_counter

from .config import VehicleConfig
from .curvature import arc_length_resampled_curvature
from .data_loader import TrackData, list_track_names, load_track
from .speed_profile import compute_speed_profile


DEFAULT_POINT_COUNTS = list(range(100, 1500, 100))


@dataclass(frozen=True)
class IntegrationSensitivitySample:
    point_count: int
    mean_ds_m: float
    solve_runtime_ms: float
    trapezoidal_time_s: float
    simpson_time_s: float | None
    kinematic_time_s: float
    mean_abs_curvature_1_per_m: float
    rms_curvature_1_per_m: float
    curvature_rms_abs_error_1_per_m: float
    trapezoidal_abs_error_s: float
    trapezoidal_rel_error_pct: float
    simpson_abs_error_s: float | None
    simpson_rel_error_pct: float | None

    @property
    def combined_abs_error_s(self) -> float:
        simpson_abs_error = 0.0 if self.simpson_abs_error_s is None else self.simpson_abs_error_s
        return max(self.trapezoidal_abs_error_s, simpson_abs_error)


@dataclass(frozen=True)
class IntegrationSensitivityStudy:
    track_name: str
    samples: list[IntegrationSensitivitySample]
    reference_point_count: int
    reference_mean_ds_m: float
    reference_trapezoidal_time_s: float
    reference_simpson_time_s: float | None
    reference_rms_curvature_1_per_m: float
    tolerance_s: float

    @property
    def point_counts(self) -> list[int]:
        return [sample.point_count for sample in self.samples]

    @property
    def recommended_point_count(self) -> int | None:
        for sample in self.samples:
            simpson_ok = sample.simpson_abs_error_s is None or sample.simpson_abs_error_s <= self.tolerance_s
            if sample.trapezoidal_abs_error_s <= self.tolerance_s and simpson_ok:
                return sample.point_count
        return None

    @property
    def recommended_mean_ds_m(self) -> float | None:
        recommended = self.recommended_point_count
        if recommended is None:
            return None
        for sample in self.samples:
            if sample.point_count == recommended:
                return sample.mean_ds_m
        return None

    def sample_for_point_count(self, point_count: int) -> IntegrationSensitivitySample:
        for sample in self.samples:
            if sample.point_count == point_count:
                return sample
        raise KeyError(f"Point count {point_count} is not in the sensitivity sweep.")


@dataclass(frozen=True)
class AllTracksIntegrationSensitivity:
    track_studies: dict[str, IntegrationSensitivityStudy]
    point_counts: list[int]
    reference_point_count: int
    tolerance_s: float

    @property
    def ordered_track_names(self) -> list[str]:
        return sorted(
            self.track_studies,
            key=lambda track_name: (
                self.track_studies[track_name].recommended_point_count is None,
                -(self.track_studies[track_name].recommended_point_count or 0),
                track_name,
            ),
        )

    @property
    def global_recommended_point_count(self) -> int | None:
        for point_count in self.point_counts:
            if self.counts_within_tolerance(point_count) == len(self.track_studies):
                return point_count
        return None

    def counts_within_tolerance(self, point_count: int) -> int:
        return sum(
            1
            for study in self.track_studies.values()
            if study.sample_for_point_count(point_count).combined_abs_error_s <= self.tolerance_s
        )

    def worst_track_for_point_count(self, point_count: int) -> tuple[str, float]:
        worst_track = ""
        worst_error = -math.inf
        for track_name, study in self.track_studies.items():
            sample = study.sample_for_point_count(point_count)
            if sample.combined_abs_error_s > worst_error:
                worst_track = track_name
                worst_error = sample.combined_abs_error_s
        return worst_track, worst_error


def normalize_point_counts(point_counts: list[int]) -> list[int]:
    normalized = sorted({_normalize_point_count(point_count) for point_count in point_counts})
    if not normalized:
        raise ValueError("At least one point count is required for the sensitivity sweep.")
    return normalized


def default_reference_point_count(point_counts: list[int]) -> int:
    return _normalize_point_count(2 * max(point_counts))


def compute_track_integration_sensitivity(
    track: TrackData,
    config: VehicleConfig,
    point_counts: list[int],
    reference_point_count: int | None = None,
    tolerance_s: float = 0.1,
) -> IntegrationSensitivityStudy:
    normalized_counts = normalize_point_counts(point_counts)
    resolved_reference_point_count = _normalize_point_count(
        reference_point_count if reference_point_count is not None else default_reference_point_count(normalized_counts)
    )
    if resolved_reference_point_count <= max(normalized_counts):
        raise ValueError("Reference point count must be greater than every point count in the sweep.")

    reference_profile = arc_length_resampled_curvature(track, resolved_reference_point_count)
    reference_result = compute_speed_profile(reference_profile, config)
    reference_rms_curvature = _rms(reference_profile.curvature)

    samples: list[IntegrationSensitivitySample] = []
    for point_count in normalized_counts:
        start_time = perf_counter()
        profile = arc_length_resampled_curvature(track, point_count)
        result = compute_speed_profile(profile, config)
        solve_runtime_ms = 1000.0 * (perf_counter() - start_time)
        trapezoidal_abs_error = abs(result.integration.trapezoidal_time_s - reference_result.integration.trapezoidal_time_s)
        simpson_abs_error = _simpson_abs_error(result.integration.simpson_time_s, reference_result.integration.simpson_time_s)
        rms_curvature = _rms(profile.curvature)
        samples.append(
            IntegrationSensitivitySample(
                point_count=point_count,
                mean_ds_m=profile.total_length_m / point_count,
                solve_runtime_ms=solve_runtime_ms,
                trapezoidal_time_s=result.integration.trapezoidal_time_s,
                simpson_time_s=result.integration.simpson_time_s,
                kinematic_time_s=result.integration.kinematic_time_s,
                mean_abs_curvature_1_per_m=_mean_abs(profile.curvature),
                rms_curvature_1_per_m=rms_curvature,
                curvature_rms_abs_error_1_per_m=abs(rms_curvature - reference_rms_curvature),
                trapezoidal_abs_error_s=trapezoidal_abs_error,
                trapezoidal_rel_error_pct=_relative_error_pct(
                    trapezoidal_abs_error,
                    reference_result.integration.trapezoidal_time_s,
                ),
                simpson_abs_error_s=simpson_abs_error,
                simpson_rel_error_pct=None
                if simpson_abs_error is None or reference_result.integration.simpson_time_s is None
                else _relative_error_pct(simpson_abs_error, reference_result.integration.simpson_time_s),
            )
        )

    return IntegrationSensitivityStudy(
        track_name=track.name,
        samples=samples,
        reference_point_count=resolved_reference_point_count,
        reference_mean_ds_m=reference_profile.total_length_m / resolved_reference_point_count,
        reference_trapezoidal_time_s=reference_result.integration.trapezoidal_time_s,
        reference_simpson_time_s=reference_result.integration.simpson_time_s,
        reference_rms_curvature_1_per_m=reference_rms_curvature,
        tolerance_s=tolerance_s,
    )


def compute_all_tracks_integration_sensitivity(
    config: VehicleConfig,
    point_counts: list[int],
    reference_point_count: int | None = None,
    tolerance_s: float = 0.1,
    track_names: list[str] | None = None,
) -> AllTracksIntegrationSensitivity:
    normalized_counts = normalize_point_counts(point_counts)
    resolved_reference_point_count = _normalize_point_count(
        reference_point_count if reference_point_count is not None else default_reference_point_count(normalized_counts)
    )
    studies: dict[str, IntegrationSensitivityStudy] = {}
    for track_name in (track_names or list_track_names()):
        track = load_track(track_name)
        studies[track_name] = compute_track_integration_sensitivity(
            track,
            config,
            normalized_counts,
            reference_point_count=resolved_reference_point_count,
            tolerance_s=tolerance_s,
        )

    return AllTracksIntegrationSensitivity(
        track_studies=studies,
        point_counts=normalized_counts,
        reference_point_count=resolved_reference_point_count,
        tolerance_s=tolerance_s,
    )


def _normalize_point_count(point_count: int) -> int:
    if point_count < 4:
        raise ValueError("Point counts must be at least 4.")
    return point_count if point_count % 2 == 0 else point_count + 1


def _mean_abs(values: list[float]) -> float:
    return sum(abs(value) for value in values) / len(values)


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _relative_error_pct(abs_error: float, reference_value: float) -> float:
    if math.isclose(reference_value, 0.0):
        return 0.0
    return 100.0 * abs_error / abs(reference_value)


def _simpson_abs_error(value: float | None, reference_value: float | None) -> float | None:
    if value is None or reference_value is None:
        return None
    return abs(value - reference_value)