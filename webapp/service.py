"""Analysis service that wraps the existing solver for the web app.

This layer is method-aware so that the future trajectory-optimized results
can be added without changing the API contract.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import shutil
from threading import Lock
from typing import Any

import numpy as np

from source.config import VehicleConfig, load_vehicle_config
from source.curvature import (
    CurvatureProfile,
    compare_raw_and_resampled_curvature,
    periodic_central_difference_curvature,
)
from source.data_loader import PROJECT_ROOT, TrackData, list_track_names, load_track
from source.geometry import TrackAudit, audit_track
from source.path_optimization import interpolate_widths_at_resampled, run_iterative_optimization
from source.path_optimization_b import resolve_parallel_workers, run_iterative_time_optimization
from source.path_optimization_c import run_apg_optimization
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
MIN_CURVATURE_METHOD = "min_curvature"
MIN_LAP_TIME_METHOD = "min_lap_time"
MIN_CURVATURE_CUSTOM_METHOD = "min_curvature_custom"
SUPPORTED_METHODS = (
    BASELINE_METHOD,
    MIN_CURVATURE_METHOD,
    MIN_LAP_TIME_METHOD,
    MIN_CURVATURE_CUSTOM_METHOD,
)
WEB_CACHE_DIR = PROJECT_ROOT / "outputs" / "web_cache"
CACHE_MANIFEST_PATH = WEB_CACHE_DIR / "manifest.json"
CACHE_VERSION = 2


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
    path_offset_m: list[float] | None = None


@dataclass
class _AnalysisCache:
    by_track_method: dict[tuple[str, str], MethodResult] = field(default_factory=dict)
    difficulty_scores_by_method: dict[str, dict[str, float]] = field(default_factory=dict)


@dataclass
class _PayloadCache:
    by_track_method: dict[tuple[str, str], dict[str, Any]] = field(default_factory=dict)
    difficulty_scores_by_method: dict[str, dict[str, float]] = field(default_factory=dict)


class AnalysisService:
    """Compute, cache, and expose per-track results.

    Results are cached per (track, method) pair. When preloaded, the service
    solves all supported methods up front so the web UI can read directly from
    the cache instead of recomputing on selection.
    """

    def __init__(
        self,
        vehicle_config: VehicleConfig | None = None,
        preload: bool = False,
        cache_dir: Path | None = None,
    ) -> None:
        self._vehicle_config = vehicle_config or load_vehicle_config()
        self._cache = _AnalysisCache()
        self._payload_cache = _PayloadCache()
        self._cache_dir = cache_dir or WEB_CACHE_DIR
        self._lock = Lock()
        if preload:
            self.preload_all(persist=True)

    @property
    def vehicle_config(self) -> VehicleConfig:
        return self._vehicle_config

    def list_track_names(self) -> list[str]:
        return list_track_names()

    def get_result(self, track_name: str, method: str = BASELINE_METHOD) -> MethodResult:
        return self._get_or_compute(track_name, method)

    def get_payload(self, track_name: str, method: str = BASELINE_METHOD) -> dict[str, Any]:
        key = (track_name, method)
        with self._lock:
            cached = self._payload_cache.by_track_method.get(key)
        if cached is not None:
            return cached

        result = self._get_or_compute(track_name, method)
        payload = method_to_payload(result)
        with self._lock:
            self._payload_cache.by_track_method[key] = payload
            self._payload_cache.difficulty_scores_by_method.pop(method, None)
        return payload

    def get_all_payloads(self, method: str = BASELINE_METHOD) -> dict[str, dict[str, Any]]:
        if method not in SUPPORTED_METHODS:
            raise ValueError(
                f"Method '{method}' is not supported. Choose one of: {', '.join(SUPPORTED_METHODS)}."
            )

        payloads: dict[str, dict[str, Any]] = {}
        for track_name in self.list_track_names():
            try:
                payloads[track_name] = self.get_payload(track_name, method)
            except Exception:  # noqa: BLE001
                continue
        return payloads

    def get_rankings(self, method: str = BASELINE_METHOD) -> dict[str, Any]:
        payloads = self.get_all_payloads(method)
        if not payloads:
            return {"method": method, "rows": [], "difficulty_scores": {}}

        difficulty = self.get_difficulty_scores(method)
        rows = []
        for track_name, payload in payloads.items():
            metrics = payload["metrics"]
            rows.append(
                {
                    "track_name": track_name,
                    "method": method,
                    "difficulty_score": difficulty.get(track_name, 0.0),
                    "geometry": metrics["geometry"],
                    "performance": metrics["performance"],
                }
            )
        rows.sort(key=lambda row: row["difficulty_score"], reverse=True)
        return {
            "method": method,
            "rows": rows,
            "difficulty_scores": difficulty,
        }

    def get_all_results(self, method: str = BASELINE_METHOD) -> dict[str, MethodResult]:
        results: dict[str, MethodResult] = {}
        for track_name in self.list_track_names():
            try:
                results[track_name] = self._get_or_compute(track_name, method)
            except Exception:  # noqa: BLE001
                # Skip tracks that cannot be solved so the dashboard still works.
                continue
        return results

    def get_baseline(self, track_name: str) -> MethodResult:
        return self._get_or_compute(track_name, BASELINE_METHOD)

    def get_all_baselines(self) -> dict[str, MethodResult]:
        return self.get_all_results(BASELINE_METHOD)

    def get_difficulty_scores(self, method: str = BASELINE_METHOD) -> dict[str, float]:
        with self._lock:
            cached = self._payload_cache.difficulty_scores_by_method.get(method)
            if cached is None:
                cached = self._cache.difficulty_scores_by_method.get(method)
        if cached is not None:
            return cached

        if method not in SUPPORTED_METHODS:
            raise ValueError(
                f"Method '{method}' is not supported. Choose one of: {', '.join(SUPPORTED_METHODS)}."
            )

        payloads = self.get_all_payloads(method)
        metrics_by_track = {
            track_name: _payload_to_metrics(payload)
            for track_name, payload in payloads.items()
        }
        scores = compute_difficulty_scores(metrics_by_track)
        with self._lock:
            self._payload_cache.difficulty_scores_by_method[method] = scores
            self._cache.difficulty_scores_by_method[method] = scores
        return scores

    def invalidate(self) -> None:
        with self._lock:
            self._cache = _AnalysisCache()
            self._payload_cache = _PayloadCache()

    def preload_all(
        self,
        progress_callback: Callable[[str, str, int, int], None] | None = None,
        persist: bool = False,
    ) -> None:
        track_names = self.list_track_names()
        total = len(track_names) * len(SUPPORTED_METHODS)
        completed = 0

        for method in SUPPORTED_METHODS:
            for track_name in track_names:
                self.get_payload(track_name, method)
                completed += 1
                if progress_callback is not None:
                    progress_callback(method, track_name, completed, total)
            self.get_difficulty_scores(method)

        if persist:
            self.save_persisted_cache()

    def load_persisted_cache(self) -> bool:
        manifest_path = self._manifest_path()
        if not manifest_path.exists():
            return False

        try:
            with manifest_path.open("r", encoding="utf-8") as handle:
                manifest = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return False

        signatures = manifest.get("signatures")
        if not isinstance(signatures, dict):
            return False

        if signatures.get("vehicle") != self._vehicle_signature():
            return False

        if signatures.get("optimization") != self._optimization_signature():
            return False

        payloads: dict[tuple[str, str], dict[str, Any]] = {}
        tracks_by_method = manifest.get("tracks_by_method", {})
        for method in SUPPORTED_METHODS:
            for track_name in tracks_by_method.get(method, []):
                payload_path = self._payload_path(method, track_name)
                if not payload_path.exists():
                    return False
                try:
                    with payload_path.open("r", encoding="utf-8") as handle:
                        payloads[(track_name, method)] = json.load(handle)
                except (OSError, json.JSONDecodeError):
                    return False

        difficulty_scores = manifest.get("difficulty_scores_by_method", {})
        with self._lock:
            self._payload_cache = _PayloadCache(
                by_track_method=payloads,
                difficulty_scores_by_method=difficulty_scores,
            )
            self._cache.difficulty_scores_by_method = dict(difficulty_scores)
        return True

    def load_persisted_optimization_cache(self) -> bool:
        manifest_path = self._manifest_path()
        if not manifest_path.exists():
            return False

        try:
            with manifest_path.open("r", encoding="utf-8") as handle:
                manifest = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return False

        signatures = manifest.get("signatures")
        if not isinstance(signatures, dict):
            return False

        if signatures.get("optimization") != self._optimization_signature():
            return False

        payloads: dict[tuple[str, str], dict[str, Any]] = {}
        tracks_by_method = manifest.get("tracks_by_method", {})
        for method in SUPPORTED_METHODS:
            for track_name in tracks_by_method.get(method, []):
                payload_path = self._payload_path(method, track_name)
                if not payload_path.exists():
                    return False
                try:
                    with payload_path.open("r", encoding="utf-8") as handle:
                        payloads[(track_name, method)] = json.load(handle)
                except (OSError, json.JSONDecodeError):
                    return False

        difficulty_scores = manifest.get("difficulty_scores_by_method", {})
        with self._lock:
            self._payload_cache = _PayloadCache(
                by_track_method=payloads,
                difficulty_scores_by_method=difficulty_scores,
            )
            self._cache = _AnalysisCache(difficulty_scores_by_method=dict(difficulty_scores))
        return True

    def refresh_vehicle_cache(
        self,
        progress_callback: Callable[[str, str, int, int], None] | None = None,
        persist: bool = False,
    ) -> None:
        with self._lock:
            payload_items = list(self._payload_cache.by_track_method.items())

        if not payload_items:
            raise RuntimeError("No cached payloads are loaded for vehicle-only refresh.")

        total = len(payload_items)
        refreshed_payloads: dict[tuple[str, str], dict[str, Any]] = {}
        for completed, ((track_name, method), payload) in enumerate(payload_items, start=1):
            refreshed_payloads[(track_name, method)] = refresh_payload_vehicle_data(
                payload,
                self._vehicle_config,
            )
            if progress_callback is not None:
                progress_callback(method, track_name, completed, total)

        difficulty_scores: dict[str, dict[str, float]] = {}
        for method in SUPPORTED_METHODS:
            metrics_by_track = {
                track_name: _payload_to_metrics(payload)
                for (track_name, payload_method), payload in refreshed_payloads.items()
                if payload_method == method
            }
            if metrics_by_track:
                difficulty_scores[method] = compute_difficulty_scores(metrics_by_track)

        with self._lock:
            self._payload_cache = _PayloadCache(
                by_track_method=refreshed_payloads,
                difficulty_scores_by_method=difficulty_scores,
            )
            self._cache = _AnalysisCache(difficulty_scores_by_method=dict(difficulty_scores))

        if persist:
            self.save_persisted_cache()

    def save_persisted_cache(self) -> None:
        payload_items = dict(self._payload_cache.by_track_method)
        difficulty_scores = dict(self._payload_cache.difficulty_scores_by_method)
        tracks_by_method: dict[str, list[str]] = {method: [] for method in SUPPORTED_METHODS}

        shutil.rmtree(self._cache_dir, ignore_errors=True)
        self._cache_dir.mkdir(parents=True, exist_ok=True)

        for (track_name, method), payload in payload_items.items():
            payload_path = self._payload_path(method, track_name)
            payload_path.parent.mkdir(parents=True, exist_ok=True)
            with payload_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle)
            tracks_by_method[method].append(track_name)

        manifest = {
            "signatures": {
                "vehicle": self._vehicle_signature(),
                "optimization": self._optimization_signature(),
            },
            "tracks_by_method": {
                method: sorted(track_names)
                for method, track_names in tracks_by_method.items()
            },
            "difficulty_scores_by_method": difficulty_scores,
        }
        with self._manifest_path().open("w", encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _get_or_compute(self, track_name: str, method: str) -> MethodResult:
        key = (track_name, method)
        with self._lock:
            cached = self._cache.by_track_method.get(key)
        if cached is not None:
            return cached

        if method == BASELINE_METHOD:
            result = self._compute_baseline(track_name)
        elif method == MIN_CURVATURE_METHOD:
            result = self._compute_min_curvature(track_name)
        elif method == MIN_LAP_TIME_METHOD:
            result = self._compute_min_lap_time(track_name)
        elif method == MIN_CURVATURE_CUSTOM_METHOD:
            result = self._compute_min_curvature_custom(track_name)
        else:
            raise ValueError(
                f"Method '{method}' is not supported. Choose one of: {', '.join(SUPPORTED_METHODS)}."
            )

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
            path_offset_m=[0.0 for _ in track.x],
        )

    def _compute_min_curvature(self, track_name: str) -> MethodResult:
        track = load_track(track_name)

        history = run_iterative_optimization(
            track,
            self._vehicle_config,
            reg_lambda=self._vehicle_config.path_optimization_reg_lambda,
        )
        final_iteration = history[-1]

        profile = periodic_central_difference_curvature(
            list(final_iteration.x_path),
            list(final_iteration.y_path),
            total_length_m=None,
        )
        speed = compute_speed_profile(profile, self._vehicle_config)

        width_left, width_right = interpolate_widths_at_resampled(track, profile.s)
        optimized_track = TrackData(
            name=track.name,
            x=list(profile.x),
            y=list(profile.y),
            width_right=list(width_right),
            width_left=list(width_left),
            path=track.path,
        )
        audit = audit_track(optimized_track)

        geometry = compute_geometry_metrics(
            total_length_m=audit.total_length_m,
            closure_gap_m=audit.closure_gap_m,
            spacing_cv=audit.spacing_cv,
            curvature=profile.curvature,
            width_right=optimized_track.width_right,
            width_left=optimized_track.width_left,
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
            method=MIN_CURVATURE_METHOD,
            geometry=geometry,
            performance=performance,
        )
        return MethodResult(
            track_name=track_name,
            method=MIN_CURVATURE_METHOD,
            track=track,
            audit=audit,
            profile=profile,
            speed=speed,
            metrics=metrics,
            path_offset_m=final_iteration.e_opt.tolist(),
        )

    def _compute_min_lap_time(self, track_name: str) -> MethodResult:
        seed_result = self._get_or_compute(track_name, MIN_CURVATURE_METHOD)
        if seed_result.path_offset_m is None:
            raise RuntimeError("Min-lap-time optimization requires a min-curvature warm start.")

        track = seed_result.track
        history = run_iterative_time_optimization(
            track,
            self._vehicle_config,
            e_init=np.array(seed_result.path_offset_m, dtype=float),
            max_iters=self._vehicle_config.path_optimization_b_max_iters,
            eps=self._vehicle_config.path_optimization_b_eps_m,
            alpha0=self._vehicle_config.path_optimization_b_alpha0,
            gtol=self._vehicle_config.path_optimization_b_gtol,
            ftol=self._vehicle_config.path_optimization_b_ftol_s,
            n_jobs=resolve_parallel_workers(self._vehicle_config.path_optimization_b_n_jobs),
            beta=self._vehicle_config.path_optimization_b_beta,
            verbose=False,
        )
        final_iteration = history[-1]

        profile = periodic_central_difference_curvature(
            list(final_iteration.x_path),
            list(final_iteration.y_path),
            total_length_m=None,
        )
        speed = compute_speed_profile(profile, self._vehicle_config)

        width_left, width_right = interpolate_widths_at_resampled(track, profile.s)
        optimized_track = TrackData(
            name=track.name,
            x=list(profile.x),
            y=list(profile.y),
            width_right=list(width_right),
            width_left=list(width_left),
            path=track.path,
        )
        audit = audit_track(optimized_track)

        geometry = compute_geometry_metrics(
            total_length_m=audit.total_length_m,
            closure_gap_m=audit.closure_gap_m,
            spacing_cv=audit.spacing_cv,
            curvature=profile.curvature,
            width_right=optimized_track.width_right,
            width_left=optimized_track.width_left,
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
            method=MIN_LAP_TIME_METHOD,
            geometry=geometry,
            performance=performance,
        )
        return MethodResult(
            track_name=track_name,
            method=MIN_LAP_TIME_METHOD,
            track=track,
            audit=audit,
            profile=profile,
            speed=speed,
            metrics=metrics,
            path_offset_m=final_iteration.e_opt.tolist(),
        )

    def _compute_min_curvature_custom(self, track_name: str) -> MethodResult:
        track = load_track(track_name)
        history = run_apg_optimization(
            track,
            self._vehicle_config,
            reg_lambda=self._vehicle_config.path_optimization_reg_lambda,
            apg_max_iters=self._vehicle_config.path_optimization_c_apg_max_iters,
            apg_tol=self._vehicle_config.path_optimization_c_apg_tol,
        )
        final_iteration = history[-1]

        profile = periodic_central_difference_curvature(
            list(final_iteration.x_path),
            list(final_iteration.y_path),
            total_length_m=None,
        )
        speed = compute_speed_profile(profile, self._vehicle_config)

        width_left, width_right = interpolate_widths_at_resampled(track, profile.s)
        optimized_track = TrackData(
            name=track.name,
            x=list(profile.x),
            y=list(profile.y),
            width_right=list(width_right),
            width_left=list(width_left),
            path=track.path,
        )
        audit = audit_track(optimized_track)

        geometry = compute_geometry_metrics(
            total_length_m=audit.total_length_m,
            closure_gap_m=audit.closure_gap_m,
            spacing_cv=audit.spacing_cv,
            curvature=profile.curvature,
            width_right=optimized_track.width_right,
            width_left=optimized_track.width_left,
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
            method=MIN_CURVATURE_CUSTOM_METHOD,
            geometry=geometry,
            performance=performance,
        )
        return MethodResult(
            track_name=track_name,
            method=MIN_CURVATURE_CUSTOM_METHOD,
            track=track,
            audit=audit,
            profile=profile,
            speed=speed,
            metrics=metrics,
            path_offset_m=final_iteration.e_opt.tolist(),
        )

    def _vehicle_signature(self) -> dict[str, Any]:
        return {
            "cache_version": CACHE_VERSION,
            "vehicle_config": self._vehicle_config.to_vehicle_dict(),
        }

    def _optimization_signature(self) -> dict[str, Any]:
        return {
            "cache_version": CACHE_VERSION,
            "optimization_config": self._vehicle_config.to_optimization_dict(),
            "supported_methods": list(SUPPORTED_METHODS),
            "track_names": self.list_track_names(),
        }

    def _payload_path(self, method: str, track_name: str) -> Path:
        return self._cache_dir / method / f"{track_name}.json"

    def _manifest_path(self) -> Path:
        return self._cache_dir / "manifest.json"


# ---------------------------------------------------------------------------
# Serialization helpers
# ---------------------------------------------------------------------------

def method_to_payload(result: MethodResult) -> dict[str, Any]:
    """Plotly-ready arrays + summary fields for the track detail view."""
    track = result.track
    audit = result.audit
    profile = result.profile
    speed_data, integration, residuals = _speed_sections(result.speed)

    return {
        "track_name": result.track_name,
        "method": result.method,
        "geometry_method": result.speed.geometry_method,
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
        "speed": speed_data,
        "integration": integration,
        "residuals": residuals,
        "metrics": result.metrics.to_dict(),
    }


def baseline_to_payload(result: MethodResult) -> dict[str, Any]:
    return method_to_payload(result)


def _payload_to_metrics(payload: dict[str, Any]) -> TrackMetrics:
    metrics = payload["metrics"]
    return TrackMetrics(
        track_name=metrics["track_name"],
        method=metrics["method"],
        geometry=GeometryMetrics(**metrics["geometry"]),
        performance=PerformanceMetrics(**metrics["performance"]),
    )


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


def refresh_payload_vehicle_data(payload: dict[str, Any], vehicle_config: VehicleConfig) -> dict[str, Any]:
    profile = CurvatureProfile(
        method=str(payload.get("geometry_method", payload["method"])),
        x=list(payload["profile"]["x_m"]),
        y=list(payload["profile"]["y_m"]),
        s=list(payload["profile"]["s_m"]),
        curvature=list(payload["profile"]["curvature_1_per_m"]),
        total_length_m=float(payload["speed"]["total_length_m"]),
        uniform_ds_m=float(payload["profile"]["uniform_ds_m"]),
    )
    refreshed_speed = compute_speed_profile(profile, vehicle_config)
    speed_data, integration, residuals = _speed_sections(refreshed_speed)
    performance = compute_performance_metrics(
        lap_time_s=refreshed_speed.integration.trapezoidal_time_s,
        total_length_m=refreshed_speed.total_length_m,
        final_speed_mps=refreshed_speed.final_speed_mps,
        longitudinal_accel_mps2=refreshed_speed.longitudinal_accel_mps2,
        lateral_accel_mps2=refreshed_speed.lateral_accel_mps2,
        ay_max_mps2=vehicle_config.ay_max_mps2,
    )

    return {
        **payload,
        "geometry_method": refreshed_speed.geometry_method,
        "speed": speed_data,
        "integration": integration,
        "residuals": residuals,
        "metrics": {
            "track_name": payload["metrics"]["track_name"],
            "method": payload["metrics"]["method"],
            "geometry": dict(payload["metrics"]["geometry"]),
            "performance": asdict(performance),
        },
    }


def _speed_sections(speed: SpeedProfileResult) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    speed_data = {
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
    }
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
    return speed_data, integration, residuals
