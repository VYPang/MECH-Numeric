"""Analysis service that wraps the existing solver for the web app.

This layer is method-aware so that the future trajectory-optimized results
can be added without changing the API contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import Any

from source.config import VehicleConfig, load_vehicle_config
from source.curvature import (
    CurvatureProfile,
    compare_raw_and_resampled_curvature,
)
from source.data_loader import TrackData, list_track_names, load_track
from source.geometry import TrackAudit, audit_track
from source.speed_profile import SpeedProfileResult, compute_speed_profile

from .metrics import (
    GeometryMetrics,
    PerformanceMetrics,
    TrackMetrics,
    compute_difficulty_scores,
    compute_geometry_metrics,
    compute_performance_metrics,
)


BASELINE_METHOD = "centerline_baseline"


@dataclass(frozen=True)
class MethodResult:
    """Solved trajectory plus metrics for a (track, method) pair."""

    track_name: str
    method: str
    track: TrackData
    audit: TrackAudit
    profile: CurvatureProfile
    speed: SpeedProfileResult
    metrics: TrackMetrics


@dataclass
class _AnalysisCache:
    by_track_method: dict[tuple[str, str], MethodResult] = field(default_factory=dict)
    difficulty_scores_by_method: dict[str, dict[str, float]] = field(default_factory=dict)


class AnalysisService:
    """Compute, cache, and expose per-track results.

    Centerline baseline is computed on demand and cached. Difficulty scores
    are computed across the full set of cached tracks per method, so they
    update as more tracks are visited.
    """

    def __init__(self, vehicle_config: VehicleConfig | None = None) -> None:
        self._vehicle_config = vehicle_config or load_vehicle_config()
        self._cache = _AnalysisCache()
        self._lock = Lock()

    @property
    def vehicle_config(self) -> VehicleConfig:
        return self._vehicle_config

    def list_track_names(self) -> list[str]:
        return list_track_names()

    def get_baseline(self, track_name: str) -> MethodResult:
        return self._get_or_compute(track_name, BASELINE_METHOD)

    def get_all_baselines(self) -> dict[str, MethodResult]:
        results: dict[str, MethodResult] = {}
        for track_name in self.list_track_names():
            try:
                results[track_name] = self._get_or_compute(track_name, BASELINE_METHOD)
            except Exception:  # noqa: BLE001
                # Skip tracks that cannot be solved so the dashboard still works.
                continue
        return results

    def get_difficulty_scores(self, method: str = BASELINE_METHOD) -> dict[str, float]:
        with self._lock:
            cached = self._cache.difficulty_scores_by_method.get(method)
        if cached is not None:
            return cached

        if method == BASELINE_METHOD:
            self.get_all_baselines()

        metrics_by_track = {
            track_name: result.metrics
            for (track_name, cached_method), result in self._cache.by_track_method.items()
            if cached_method == method
        }
        scores = compute_difficulty_scores(metrics_by_track)
        with self._lock:
            self._cache.difficulty_scores_by_method[method] = scores
        return scores

    def invalidate(self) -> None:
        with self._lock:
            self._cache = _AnalysisCache()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _get_or_compute(self, track_name: str, method: str) -> MethodResult:
        key = (track_name, method)
        with self._lock:
            cached = self._cache.by_track_method.get(key)
        if cached is not None:
            return cached

        if method != BASELINE_METHOD:
            raise NotImplementedError(
                f"Method '{method}' is not supported yet. Only '{BASELINE_METHOD}'."
            )

        result = self._compute_baseline(track_name)

        with self._lock:
            self._cache.by_track_method[key] = result
            # Difficulty cache depends on the full set; invalidate per-method.
            self._cache.difficulty_scores_by_method.pop(method, None)
        return result

    def _compute_baseline(self, track_name: str) -> MethodResult:
        track = load_track(track_name)
        audit = audit_track(track)

        resampled_count = (
            track.point_count + 1
            if track.point_count % 2 != 0
            else track.point_count
        )
        comparison = compare_raw_and_resampled_curvature(track, resampled_count=resampled_count)
        profile = comparison.resampled
        speed = compute_speed_profile(profile, self._vehicle_config)

        geometry = compute_geometry_metrics(
            total_length_m=audit.total_length_m,
            closure_gap_m=audit.closure_gap_m,
            spacing_cv=audit.spacing_cv,
            curvature=profile.curvature,
            width_right=track.width_right,
            width_left=track.width_left,
        )
        performance = compute_performance_metrics(
            lap_time_s=speed.integration.trapezoidal_time_s,
            total_length_m=speed.total_length_m,
            final_speed_mps=speed.final_speed_mps,
            longitudinal_accel_mps2=speed.longitudinal_accel_mps2,
            lateral_accel_mps2=speed.lateral_accel_mps2,
            ay_max_mps2=self._vehicle_config.ay_max_mps2,
        )
        metrics = TrackMetrics(
            track_name=track_name,
            method=BASELINE_METHOD,
            geometry=geometry,
            performance=performance,
        )
        return MethodResult(
            track_name=track_name,
            method=BASELINE_METHOD,
            track=track,
            audit=audit,
            profile=profile,
            speed=speed,
            metrics=metrics,
        )


# ---------------------------------------------------------------------------
# Serialization helpers
# ---------------------------------------------------------------------------

def baseline_to_payload(result: MethodResult) -> dict[str, Any]:
    """Plotly-ready arrays + summary fields for the track detail view."""
    speed = result.speed
    track = result.track
    audit = result.audit
    profile = result.profile

    integration = {
        "kinematic_time_s": speed.integration.kinematic_time_s,
        "left_rule_time_s": speed.integration.left_rule_time_s,
        "trapezoidal_time_s": speed.integration.trapezoidal_time_s,
        "simpson_time_s": speed.integration.simpson_time_s,
    }
    residuals = {
        "lateral_mps2": speed.residuals.lateral_mps2,
        "acceleration_mps2": speed.residuals.acceleration_mps2,
        "braking_mps2": speed.residuals.braking_mps2,
        "friction_circle_mps2": speed.residuals.friction_circle_mps2,
    }

    return {
        "track_name": result.track_name,
        "method": result.method,
        "geometry_method": speed.geometry_method,
        "audit": _audit_to_dict(audit),
        "track": {
            "x_m": track.x,
            "y_m": track.y,
            "width_right_m": track.width_right,
            "width_left_m": track.width_left,
        },
        "profile": {
            "x_m": profile.x,
            "y_m": profile.y,
            "s_m": profile.s,
            "curvature_1_per_m": profile.curvature,
            "uniform_ds_m": profile.uniform_ds_m,
        },
        "speed": {
            "s_nodes_m": speed.s_nodes_m,
            "s_midpoints_m": speed.s_midpoints_m,
            "segment_lengths_m": speed.segment_lengths_m,
            "speed_cap_mps": speed.speed_cap_mps,
            "forward_speed_mps": speed.forward_speed_mps,
            "final_speed_mps": speed.final_speed_mps,
            "longitudinal_accel_mps2": speed.longitudinal_accel_mps2,
            "lateral_accel_mps2": speed.lateral_accel_mps2,
            "forward_longitudinal_limit_mps2": speed.forward_longitudinal_limit_mps2,
            "braking_longitudinal_limit_mps2": speed.braking_longitudinal_limit_mps2,
            "friction_total_accel_mps2": speed.friction_total_accel_mps2,
            "total_length_m": speed.total_length_m,
        },
        "integration": integration,
        "residuals": residuals,
        "metrics": result.metrics.to_dict(),
    }


def _audit_to_dict(audit: TrackAudit) -> dict[str, Any]:
    return {
        "point_count": audit.point_count,
        "closure_gap_m": audit.closure_gap_m,
        "total_length_m": audit.total_length_m,
        "min_segment_m": audit.min_segment_m,
        "max_segment_m": audit.max_segment_m,
        "mean_segment_m": audit.mean_segment_m,
        "stdev_segment_m": audit.stdev_segment_m,
        "spacing_cv": audit.spacing_cv,
        "width_right_min_m": audit.width_right_min_m,
        "width_right_max_m": audit.width_right_max_m,
        "width_right_mean_m": audit.width_right_mean_m,
        "width_left_min_m": audit.width_left_min_m,
        "width_left_max_m": audit.width_left_max_m,
        "width_left_mean_m": audit.width_left_mean_m,
        "outlier_segment_count": len(audit.outlier_segment_indices),
    }
