"""FastAPI app exposing track baseline analysis to the local dashboard."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .service import AnalysisService, BASELINE_METHOD, baseline_to_payload


PROJECT_ROOT = Path(__file__).resolve().parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"


def create_app() -> FastAPI:
    app = FastAPI(title="Centerline Baseline Dashboard", version="0.1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    service = AnalysisService()
    app.state.service = service

    @app.get("/api/health")
    def health() -> dict[str, str]:
        return {"status": "ok"}

    @app.get("/api/vehicle")
    def vehicle() -> dict[str, Any]:
        config = service.vehicle_config
        return {
            "ay_max_mps2": config.ay_max_mps2,
            "ax_engine_max_mps2": config.ax_engine_max_mps2,
            "brake_max_mps2": config.brake_max_mps2,
            "mu": config.mu,
            "F_z_n": config.F_z_n,
            "power_limit_w": config.power_limit_w,
            "v_max_mps": config.v_max_mps,
            "v_min_mps": config.v_min_mps,
        }

    @app.get("/api/tracks")
    def list_tracks() -> dict[str, Any]:
        return {"tracks": service.list_track_names()}

    @app.get("/api/rankings")
    def rankings(method: str = BASELINE_METHOD) -> dict[str, Any]:
        results = service.get_all_baselines() if method == BASELINE_METHOD else {}
        if not results:
            return {"method": method, "rows": [], "difficulty_scores": {}}

        difficulty = service.get_difficulty_scores(method)
        rows = []
        for track_name, result in results.items():
            metrics = result.metrics
            rows.append(
                {
                    "track_name": track_name,
                    "method": method,
                    "difficulty_score": difficulty.get(track_name, 0.0),
                    "geometry": _flatten_geometry(metrics.geometry),
                    "performance": _flatten_performance(metrics.performance),
                }
            )
        rows.sort(key=lambda row: row["difficulty_score"], reverse=True)
        return {
            "method": method,
            "rows": rows,
            "difficulty_scores": difficulty,
        }

    @app.get("/api/tracks/{track_name}/baseline")
    def track_baseline(track_name: str) -> dict[str, Any]:
        try:
            result = service.get_baseline(track_name)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return baseline_to_payload(result)

    @app.get("/api/tracks/{track_name}/summary")
    def track_summary(track_name: str) -> dict[str, Any]:
        try:
            result = service.get_baseline(track_name)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        difficulty = service.get_difficulty_scores(result.method)
        payload = baseline_to_payload(result)
        return {
            "track_name": track_name,
            "method": result.method,
            "audit": payload["audit"],
            "integration": payload["integration"],
            "residuals": payload["residuals"],
            "metrics": payload["metrics"],
            "difficulty_score": difficulty.get(track_name, 0.0),
        }

    if FRONTEND_DIR.exists():
        app.mount(
            "/static",
            StaticFiles(directory=str(FRONTEND_DIR)),
            name="static",
        )

        @app.get("/")
        def index() -> FileResponse:
            return FileResponse(FRONTEND_DIR / "index.html")

        @app.get("/track")
        def track_page() -> FileResponse:
            return FileResponse(FRONTEND_DIR / "track.html")

    return app


def _flatten_geometry(geometry) -> dict[str, float]:
    return {
        "total_length_m": geometry.total_length_m,
        "closure_gap_m": geometry.closure_gap_m,
        "spacing_cv": geometry.spacing_cv,
        "mean_abs_curvature": geometry.mean_abs_curvature,
        "max_abs_curvature": geometry.max_abs_curvature,
        "rms_curvature": geometry.rms_curvature,
        "corner_severity_index": geometry.corner_severity_index,
        "width_mean_m": geometry.width_mean_m,
        "width_min_m": geometry.width_min_m,
    }


def _flatten_performance(performance) -> dict[str, float]:
    return {
        "lap_time_s": performance.lap_time_s,
        "average_speed_mps": performance.average_speed_mps,
        "max_speed_mps": performance.max_speed_mps,
        "min_speed_mps": performance.min_speed_mps,
        "speed_stdev_mps": performance.speed_stdev_mps,
        "mean_abs_long_accel_mps2": performance.mean_abs_long_accel_mps2,
        "accel_fraction": performance.accel_fraction,
        "brake_fraction": performance.brake_fraction,
        "coast_fraction": performance.coast_fraction,
        "phase_transition_count": performance.phase_transition_count,
        "lateral_limit_fraction": performance.lateral_limit_fraction,
    }


app = create_app()
