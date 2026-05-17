from __future__ import annotations

from pathlib import Path

import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich.table import Table

from .config import load_vehicle_config
from .curvature import compare_raw_and_resampled_curvature, validate_curvature_on_circle
from .data_loader import list_track_names, load_track
from .geometry import audit_track
from .plots import save_audit_plots, save_curvature_comparison_plots, save_speed_profile_plots
from .speed_profile import compute_speed_profile


app = typer.Typer(help="Track analysis CLI for three-pass racing-line experiments.")
console = Console()


def _validate_track_name(track_name: str) -> str:
    valid_names = list_track_names()
    normalized_map = {name.lower(): name for name in valid_names}
    normalized_input = track_name.strip().lower()
    if normalized_input not in normalized_map:
        choices = ", ".join(valid_names)
        raise typer.BadParameter(f"Unknown track '{track_name}'. Choose one of: {choices}")
    return normalized_map[normalized_input]


def _render_track_list_table(track_names: list[str]) -> None:
    table = Table(title="Available Tracks")
    table.add_column("#", justify="right")
    table.add_column("Track name", style="cyan")
    for index, track_name in enumerate(track_names, start=1):
        table.add_row(str(index), track_name)
    console.print(table)


def _render_audit_table(track_name: str, audit) -> None:
    table = Table(title=f"{track_name} Dataset Audit")
    table.add_column("Metric", style="cyan")
    table.add_column("Value", justify="right")
    table.add_row("Point count", str(audit.point_count))
    table.add_row("Total centerline length (m)", f"{audit.total_length_m:.3f}")
    table.add_row("Closure gap (m)", f"{audit.closure_gap_m:.3f}")
    table.add_row("Min segment length (m)", f"{audit.min_segment_m:.3f}")
    table.add_row("Max segment length (m)", f"{audit.max_segment_m:.3f}")
    table.add_row("Mean segment length (m)", f"{audit.mean_segment_m:.3f}")
    table.add_row("Segment length stdev (m)", f"{audit.stdev_segment_m:.3f}")
    table.add_row("Spacing coefficient of variation", f"{audit.spacing_cv:.4f}")
    table.add_row("Right width min/mean/max (m)", f"{audit.width_right_min_m:.3f} / {audit.width_right_mean_m:.3f} / {audit.width_right_max_m:.3f}")
    table.add_row("Left width min/mean/max (m)", f"{audit.width_left_min_m:.3f} / {audit.width_left_mean_m:.3f} / {audit.width_left_max_m:.3f}")
    table.add_row("Outlier segments", str(len(audit.outlier_segment_indices)))
    console.print(table)

    if audit.outlier_segment_indices:
        preview = ", ".join(str(index) for index in audit.outlier_segment_indices[:10])
        console.print(f"[yellow]Outlier segment indices (first 10):[/yellow] {preview}")


def _render_curvature_comparison_table(track_name: str, comparison, validation) -> None:
    table = Table(title=f"{track_name} Curvature Method Comparison")
    table.add_column("Metric", style="cyan")
    table.add_column("Value", justify="right")
    table.add_row("Raw points", str(len(comparison.raw.curvature)))
    table.add_row("Resampled points", str(len(comparison.resampled.curvature)))
    table.add_row("Raw total length (m)", f"{comparison.raw.total_length_m:.3f}")
    table.add_row("Resampled total length (m)", f"{comparison.resampled.total_length_m:.3f}")
    table.add_row("Raw assumed uniform ds (m)", f"{comparison.raw.uniform_ds_m:.6f}")
    table.add_row("Resampled uniform ds (m)", f"{comparison.resampled.uniform_ds_m:.6f}")
    table.add_row("Mean abs curvature difference (1/m)", f"{comparison.mean_abs_difference:.8f}")
    table.add_row("Max abs curvature difference (1/m)", f"{comparison.max_abs_difference:.8f}")
    table.add_row("RMS curvature difference (1/m)", f"{comparison.rms_difference:.8f}")
    console.print(table)

    validation_table = Table(title="Circle Curvature Validation")
    validation_table.add_column("Metric", style="cyan")
    validation_table.add_column("Value", justify="right")
    validation_table.add_row("Radius (m)", f"{validation.radius_m:.3f}")
    validation_table.add_row("Point count", str(validation.point_count))
    validation_table.add_row("Expected curvature (1/m)", f"{validation.expected_curvature:.8f}")
    validation_table.add_row("Mean abs error (1/m)", f"{validation.mean_abs_error:.10f}")
    validation_table.add_row("Max abs error (1/m)", f"{validation.max_abs_error:.10f}")
    validation_table.add_row("RMS error (1/m)", f"{validation.rms_error:.10f}")
    console.print(validation_table)


def _render_speed_profile_table(track_name: str, result) -> None:
    table = Table(title=f"{track_name} Speed Profile Study")
    table.add_column("Metric", style="cyan")
    table.add_column("Value", justify="right")
    table.add_row("Geometry method", result.geometry_method)
    table.add_row("Total length (m)", f"{result.total_length_m:.3f}")
    table.add_row("Friction total accel limit (m/s^2)", f"{result.friction_total_accel_mps2:.3f}")
    table.add_row("Kinematic lap time (s)", f"{result.integration.kinematic_time_s:.3f}")
    table.add_row("Left-rule lap time (s)", f"{result.integration.left_rule_time_s:.3f}")
    table.add_row("Trapezoidal lap time (s)", f"{result.integration.trapezoidal_time_s:.3f}")
    simpson_value = "not applicable" if result.integration.simpson_time_s is None else f"{result.integration.simpson_time_s:.3f}"
    table.add_row("Simpson lap time (s)", simpson_value)
    table.add_row("Finish speed (m/s)", f"{result.final_speed_mps[-1]:.3f}")
    table.add_row("Max speed (m/s)", f"{max(result.final_speed_mps):.3f}")
    table.add_row("Min lateral speed cap (m/s)", f"{min(result.speed_cap_mps):.3f}")
    table.add_row("Min forward accel limit (m/s^2)", f"{min(result.forward_longitudinal_limit_mps2):.3f}")
    table.add_row("Min braking limit (m/s^2)", f"{min(result.braking_longitudinal_limit_mps2):.3f}")
    table.add_row("Max longitudinal accel (m/s^2)", f"{max(result.longitudinal_accel_mps2):.3f}")
    table.add_row("Max braking accel (m/s^2)", f"{min(result.longitudinal_accel_mps2):.3f}")
    table.add_row("Lateral residual (m/s^2)", f"{result.residuals.lateral_mps2:.6e}")
    table.add_row("Acceleration residual (m^2/s^2)", f"{result.residuals.acceleration_m2ps2:.6e}")
    table.add_row("Braking residual (m^2/s^2)", f"{result.residuals.braking_m2ps2:.6e}")
    table.add_row("Friction-circle residual (m/s^2)", f"{result.residuals.friction_circle_mps2:.6e}")
    console.print(table)


@app.command("list-tracks")
def list_tracks() -> None:
    """List all available track names."""
    track_names = list_track_names()
    _render_track_list_table(track_names)


@app.command("audit-track")
def audit_track_command(
    track: str = typer.Option(..., "--track", "-t", prompt="Track name"),
    save_plots: bool = typer.Option(True, "--save-plots/--no-save-plots", help="Save milestone 1 audit plots."),
) -> None:
    """Audit one track dataset and optionally save milestone 1 plots."""
    track_name = _validate_track_name(track)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description=f"Loading {track_name} data", total=None)
        track_data = load_track(track_name)
        audit = audit_track(track_data)

    _render_audit_table(track_name, audit)
    console.print(f"[green]Source file:[/green] {track_data.path}")

    if save_plots:
        saved_paths = save_audit_plots(track_data, audit)
        console.print("[green]Saved plots:[/green]")
        for path in saved_paths:
            console.print(f"- {Path(path)}")


@app.command("compare-methods")
def compare_methods_command(
    track: str = typer.Option(..., "--track", "-t", prompt="Track name"),
    resampled_count: int | None = typer.Option(None, "--resampled-count", help="Number of arc-length-resampled points. Defaults to raw point count."),
    save_plots: bool = typer.Option(True, "--save-plots/--no-save-plots", help="Save curvature comparison plots."),
) -> None:
    """Compare raw and arc-length-resampled curvature methods for one track."""
    track_name = _validate_track_name(track)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description=f"Comparing curvature methods for {track_name}", total=None)
        track_data = load_track(track_name)
        comparison = compare_raw_and_resampled_curvature(track_data, resampled_count=resampled_count)
        validation = validate_curvature_on_circle()

    _render_curvature_comparison_table(track_name, comparison, validation)

    if save_plots:
        saved_paths = save_curvature_comparison_plots(track_data, comparison)
        console.print("[green]Saved plots:[/green]")
        for path in saved_paths:
            console.print(f"- {Path(path)}")


@app.command("analyze-track")
def analyze_track_command(
    track: str = typer.Option(..., "--track", "-t", prompt="Track name"),
    geometry: str = typer.Option("resampled", "--geometry", help="Geometry method: 'raw' or 'resampled'."),
    resampled_count: int | None = typer.Option(None, "--resampled-count", help="Number of resampled points. Defaults to raw point count."),
    save_plots: bool = typer.Option(True, "--save-plots/--no-save-plots", help="Save speed-profile and integration plots."),
) -> None:
    """Run the standing-start speed propagation and integration study for one track."""
    track_name = _validate_track_name(track)
    geometry_name = geometry.strip().lower()
    if geometry_name not in {"raw", "resampled"}:
        raise typer.BadParameter("Geometry must be either 'raw' or 'resampled'.")

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description=f"Running speed study for {track_name}", total=None)
        track_data = load_track(track_name)
        speed_resampled_count = resampled_count
        if geometry_name == "resampled" and speed_resampled_count is None and track_data.point_count % 2 != 0:
            speed_resampled_count = track_data.point_count + 1
        comparison = compare_raw_and_resampled_curvature(track_data, resampled_count=speed_resampled_count)
        profile = comparison.raw if geometry_name == "raw" else comparison.resampled
        config = load_vehicle_config()
        result = compute_speed_profile(profile, config)

    _render_speed_profile_table(track_name, result)

    if save_plots:
        saved_paths = save_speed_profile_plots(track_data, result)
        console.print("[green]Saved plots:[/green]")
        for path in saved_paths:
            console.print(f"- {Path(path)}")


if __name__ == "__main__":
    app()