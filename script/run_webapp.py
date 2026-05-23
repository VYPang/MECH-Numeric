"""Launch the local dashboard.

Usage:
    uv run script/run_webapp.py
    uv run script/run_webapp.py --rerun-all
    uv run script/run_webapp.py --rerun-vehicle
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


from rich.console import Console
from rich.progress import BarColumn, MofNCompleteColumn, Progress, SpinnerColumn, TextColumn, TimeElapsedColumn

from webapp.main import create_app
from webapp.service import AnalysisService, SUPPORTED_METHODS


console = Console()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the local trajectory-analysis dashboard.")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface for the local web server.")
    parser.add_argument("--port", type=int, default=8000, help="Port for the local web server.")
    rerun_group = parser.add_mutually_exclusive_group()
    rerun_group.add_argument(
        "--rerun-all",
        action="store_true",
        help="Recompute optimized trajectories and velocity profiles, then overwrite the cached web UI results.",
    )
    rerun_group.add_argument(
        "--rerun-vehicle",
        action="store_true",
        help="Reuse cached trajectories and recompute only the vehicle-dependent velocity profiles and metrics.",
    )
    return parser.parse_args()


def _run_progress(
    service: AnalysisService,
    description: str,
    total: int,
    action: Callable[[Callable[[str, str, int, int], None]], None],
) -> None:
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        console=console,
    ) as progress:
        task_id = progress.add_task(description=description, total=total)

        def on_progress(method: str, track_name: str, completed: int, total_count: int) -> None:
            progress.update(
                task_id,
                completed=completed,
                total=total_count,
                description=f"{description} {method}: {track_name}",
            )

        action(on_progress)


def prepare_service(rerun_all: bool, rerun_vehicle: bool) -> AnalysisService:
    service = AnalysisService()

    if rerun_all:
        console.print("[yellow]Recomputing optimized trajectories and cached web UI results before launch.[/yellow]")
        total = len(service.list_track_names()) * len(SUPPORTED_METHODS)
        _run_progress(
            service,
            "Preloading",
            total,
            lambda on_progress: service.preload_all(progress_callback=on_progress, persist=True),
        )
        console.print("[green]Cached web UI results are ready.[/green]")
        return service

    if service.load_persisted_cache():
        console.print("[green]Loaded cached web UI results.[/green]")
        return service

    if rerun_vehicle:
        console.print("[yellow]Refreshing cached velocity profiles from the current vehicle config.[/yellow]")
        if service.load_persisted_optimization_cache():
            total = len(service.list_track_names()) * len(SUPPORTED_METHODS)
            _run_progress(
                service,
                "Refreshing vehicle-dependent results for",
                total,
                lambda on_progress: service.refresh_vehicle_cache(progress_callback=on_progress, persist=True),
            )
            console.print("[green]Vehicle-dependent cached results are ready.[/green]")
            return service

        console.print(
            "[yellow]No optimization-compatible cache was found for vehicle-only refresh. Recomputing everything instead.[/yellow]"
        )
    elif service.load_persisted_optimization_cache():
        console.print(
            "[yellow]Vehicle config changed but optimized trajectories are still valid. Refreshing velocity profiles from cache.[/yellow]"
        )
        total = len(service.list_track_names()) * len(SUPPORTED_METHODS)
        _run_progress(
            service,
            "Refreshing vehicle-dependent results for",
            total,
            lambda on_progress: service.refresh_vehicle_cache(progress_callback=on_progress, persist=True),
        )
        console.print("[green]Vehicle-dependent cached results are ready.[/green]")
        return service
    else:
        console.print("[yellow]No valid cached web UI results found. Computing them now.[/yellow]")

    total = len(service.list_track_names()) * len(SUPPORTED_METHODS)
    _run_progress(
        service,
        "Preloading",
        total,
        lambda on_progress: service.preload_all(progress_callback=on_progress, persist=True),
    )

    console.print("[green]Cached web UI results are ready.[/green]")
    return service


def main(
    host: str = "127.0.0.1",
    port: int = 8000,
    rerun_all: bool = False,
    rerun_vehicle: bool = False,
) -> None:
    import uvicorn

    service = prepare_service(rerun_all=rerun_all, rerun_vehicle=rerun_vehicle)
    uvicorn.run(create_app(service=service), host=host, port=port, reload=False)


if __name__ == "__main__":
    arguments = parse_args()
    main(
        host=arguments.host,
        port=arguments.port,
        rerun_all=arguments.rerun_all,
        rerun_vehicle=arguments.rerun_vehicle,
    )
