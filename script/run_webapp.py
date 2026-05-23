"""Launch the local dashboard.

Usage:
    uv run script/run_webapp.py
    uv run script/run_webapp.py --rerun-results
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
    parser.add_argument(
        "--rerun-results",
        action="store_true",
        help="Recompute and overwrite cached web UI results before launching the app.",
    )
    return parser.parse_args()


def prepare_service(rerun_results: bool) -> AnalysisService:
    service = AnalysisService()

    if not rerun_results and service.load_persisted_cache():
        console.print("[green]Loaded cached web UI results.[/green]")
        return service

    if rerun_results:
        console.print("[yellow]Recomputing cached web UI results before launch.[/yellow]")
    else:
        console.print("[yellow]No valid cached web UI results found. Computing them now.[/yellow]")

    total = len(service.list_track_names()) * len(SUPPORTED_METHODS)
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        MofNCompleteColumn(),
        TimeElapsedColumn(),
        console=console,
    ) as progress:
        task_id = progress.add_task(description="Preloading analysis results", total=total)

        def on_progress(method: str, track_name: str, completed: int, total_count: int) -> None:
            progress.update(
                task_id,
                completed=completed,
                total=total_count,
                description=f"Preloading {method}: {track_name}",
            )

        service.preload_all(progress_callback=on_progress, persist=True)

    console.print("[green]Cached web UI results are ready.[/green]")
    return service


def main(host: str = "127.0.0.1", port: int = 8000, rerun_results: bool = False) -> None:
    import uvicorn

    service = prepare_service(rerun_results=rerun_results)
    uvicorn.run(create_app(service=service), host=host, port=port, reload=False)


if __name__ == "__main__":
    arguments = parse_args()
    main(host=arguments.host, port=arguments.port, rerun_results=arguments.rerun_results)
