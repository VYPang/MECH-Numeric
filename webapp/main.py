"""FastAPI app exposing track baseline analysis to the local dashboard."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .service import (
    AnalysisService,
    BASELINE_METHOD,
    MIN_CURVATURE_CUSTOM_METHOD,
    MIN_CURVATURE_METHOD,
    MIN_LAP_TIME_METHOD,
    SUPPORTED_METHODS,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"


def create_app(service: AnalysisService | None = None) -> FastAPI:
    app = FastAPI(title="Trajectory Analysis Dashboard", version="0.1.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    service = service or AnalysisService()
    if service is not None:
        service.load_persisted_cache()
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
            "path_optimization_reg_lambda": config.path_optimization_reg_lambda,
            "v_max_mps": config.v_max_mps,
            "v_min_mps": config.v_min_mps,
        }

    @app.get("/api/tracks")
    def list_tracks() -> dict[str, Any]:
        return {"tracks": service.list_track_names()}

    @app.get("/api/methods")
    def list_methods() -> dict[str, Any]:
        return {
            "default_method": BASELINE_METHOD,
            "methods": [
                {"value": BASELINE_METHOD, "label": "Centerline baseline"},
                {"value": MIN_CURVATURE_METHOD, "label": "Min curvature (A)"},
                {"value": MIN_LAP_TIME_METHOD, "label": "Min lap time (B)"},
                {"value": MIN_CURVATURE_CUSTOM_METHOD, "label": "Min curvature custom (C)"},
            ],
        }

    @app.get("/api/rankings")
    def rankings(method: str = BASELINE_METHOD) -> dict[str, Any]:
        if method not in SUPPORTED_METHODS:
            raise HTTPException(
                status_code=400,
                detail=f"Method '{method}' is not supported. Choose one of: {', '.join(SUPPORTED_METHODS)}.",
            )
        return service.get_rankings(method)

    @app.get("/api/tracks/{track_name}/baseline")
    def track_baseline(track_name: str) -> dict[str, Any]:
        try:
            return service.get_payload(track_name, BASELINE_METHOD)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/tracks/{track_name}/analysis")
    def track_analysis(track_name: str, method: str = BASELINE_METHOD) -> dict[str, Any]:
        try:
            return service.get_payload(track_name, method)
        except ValueError as exc:
            message = str(exc)
            status_code = 404 if message.startswith("Unknown track") else 400
            raise HTTPException(status_code=status_code, detail=message) from exc

    @app.get("/api/tracks/{track_name}/summary")
    def track_summary(track_name: str, method: str = BASELINE_METHOD) -> dict[str, Any]:
        try:
            payload = service.get_payload(track_name, method)
        except ValueError as exc:
            message = str(exc)
            status_code = 404 if message.startswith("Unknown track") else 400
            raise HTTPException(status_code=status_code, detail=message) from exc
        difficulty = service.get_difficulty_scores(payload["method"])
        return {
            "track_name": track_name,
            "method": payload["method"],
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
app = create_app()
