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
from .path_optimization import plot_optimization_results, run_iterative_optimization
from .plots import (
    save_all_tracks_integration_sensitivity_plot,
    save_audit_plots,
    save_curvature_comparison_plots,
    save_speed_profile_plots,
    save_track_integration_sensitivity_plots,
)
from .sensitivity import (
    DEFAULT_POINT_COUNTS,
    compute_all_tracks_integration_sensitivity,
    compute_track_integration_sensitivity,
)
from .speed_profile import compute_speed_profile


app = typer.Typer(help="Track analysis CLI for three-pass racing-line experiments.")
console = Console()
DEFAULT_SENSITIVITY_START_COUNT = min(DEFAULT_POINT_COUNTS)
DEFAULT_SENSITIVITY_END_COUNT = max(DEFAULT_POINT_COUNTS)
DEFAULT_SENSITIVITY_STEP_COUNT = DEFAULT_POINT_COUNTS[1] - DEFAULT_POINT_COUNTS[0] if len(DEFAULT_POINT_COUNTS) > 1 else 100


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
    table.add_row("Acceleration residual (m/s^2)", f"{result.residuals.acceleration_mps2:.6e}")
    table.add_row("Braking residual (m/s^2)", f"{result.residuals.braking_mps2:.6e}")
    table.add_row("Friction-circle residual (m/s^2)", f"{result.residuals.friction_circle_mps2:.6e}")
    console.print(table)


def _render_optimization_table(track_name: str, history) -> None:
    baseline = history[0]
    final = history[-1]
    max_offset_m = max(abs(float(value)) for value in final.e_opt) if len(final.e_opt) else 0.0

    table = Table(title=f"{track_name} Path Optimization")
    table.add_column("Metric", style="cyan")
    table.add_column("Value", justify="right")
    table.add_row("Iterations completed", str(final.iteration))
    table.add_row("Baseline lap time (s)", f"{baseline.lap_time_s:.3f}")
    table.add_row("Final lap time (s)", f"{final.lap_time_s:.3f}")
    table.add_row("Lap time delta (s)", f"{final.lap_time_s - baseline.lap_time_s:+.3f}")
    table.add_row("Baseline path length (m)", f"{baseline.total_length_m:.3f}")
    table.add_row("Final path length (m)", f"{final.total_length_m:.3f}")
    table.add_row("Path length delta (m)", f"{final.total_length_m - baseline.total_length_m:+.3f}")
    table.add_row("Max lateral offset (m)", f"{max_offset_m:.3f}")
    console.print(table)


def _build_point_counts(start_count: int, end_count: int, step_count: int) -> list[int]:
    if start_count < 4:
        raise typer.BadParameter("Start count must be at least 4.")
    if end_count < start_count:
        raise typer.BadParameter("End count must be greater than or equal to start count.")
    if step_count <= 0:
        raise typer.BadParameter("Step count must be positive.")

    return list(range(start_count, end_count + 1, step_count))


def _render_track_integration_sensitivity_table(track_name: str, study) -> None:
    table = Table(title=f"{track_name} Integration Sensitivity")
    table.add_column("N", justify="right")
    table.add_column("Mean ds (m)", justify="right")
    table.add_column("Trap (s)", justify="right")
    table.add_column("Simpson (s)", justify="right")
    table.add_column("|Trap-ref| (s)", justify="right")
    table.add_column("|Simp-ref| (s)", justify="right")
    table.add_column("|RMS(k)-ref|", justify="right")
    for sample in study.samples:
        simpson_value = "n/a" if sample.simpson_time_s is None else f"{sample.simpson_time_s:.3f}"
        simpson_error = "n/a" if sample.simpson_abs_error_s is None else f"{sample.simpson_abs_error_s:.4f}"
        table.add_row(
            str(sample.point_count),
            f"{sample.mean_ds_m:.3f}",
            f"{sample.trapezoidal_time_s:.3f}",
            simpson_value,
            f"{sample.trapezoidal_abs_error_s:.4f}",
            simpson_error,
            f"{sample.curvature_rms_abs_error_1_per_m:.3e}",
        )
    console.print(table)

    console.print(
        "Reference: "
        f"N={study.reference_point_count}, mean ds={study.reference_mean_ds_m:.3f} m, "
        f"trap={study.reference_trapezoidal_time_s:.3f} s, "
        f"Simpson={study.reference_simpson_time_s:.3f} s"
        if study.reference_simpson_time_s is not None
        else "Reference Simpson not available"
    )
    if study.recommended_point_count is None:
        console.print(f"[yellow]No sweep point count satisfies the {study.tolerance_s:.3f} s tolerance.[/yellow]")
    else:
        console.print(
            f"[green]Recommended point count:[/green] {study.recommended_point_count} "
            f"(mean ds {study.recommended_mean_ds_m:.3f} m, tolerance {study.tolerance_s:.3f} s)"
        )


def _render_all_tracks_sensitivity_tables(summary) -> None:
    count_table = Table(title="All-Track Sensitivity by Point Count")
    count_table.add_column("N", justify="right")
    count_table.add_column("Tracks within tol", justify="right")
    count_table.add_column("Worst track", style="cyan")
    count_table.add_column("Worst error (s)", justify="right")
    for point_count in summary.point_counts:
        worst_track, worst_error = summary.worst_track_for_point_count(point_count)
        count_table.add_row(
            str(point_count),
            f"{summary.counts_within_tolerance(point_count)}/{len(summary.track_studies)}",
            worst_track,
            f"{worst_error:.4f}",
        )
    console.print(count_table)

    track_table = Table(title="Per-Track Recommended Point Count")
    track_table.add_column("Track", style="cyan")
    track_table.add_column("Recommended N", justify="right")
    track_table.add_column("Mean ds (m)", justify="right")
    for track_name in summary.ordered_track_names:
        study = summary.track_studies[track_name]
        recommended_n = "n/a" if study.recommended_point_count is None else str(study.recommended_point_count)
        recommended_ds = "n/a" if study.recommended_mean_ds_m is None else f"{study.recommended_mean_ds_m:.3f}"
        track_table.add_row(track_name, recommended_n, recommended_ds)
    console.print(track_table)

    if summary.global_recommended_point_count is None:
        console.print(f"[yellow]No global point count satisfies the {summary.tolerance_s:.3f} s tolerance for every track.[/yellow]")
    else:
        console.print(
            f"[green]Global recommended point count:[/green] {summary.global_recommended_point_count} "
            f"for tolerance {summary.tolerance_s:.3f} s"
        )


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


@app.command("optimize-path")
def optimize_path_command(
    track: str = typer.Option(..., "--track", "-t", prompt="Track name"),
    point_count: int | None = typer.Option(None, "--point-count", help="Override the optimizer resampled point count."),
    max_iters: int = typer.Option(3, "--max-iters", help="Maximum number of optimization iterations."),
    tol: float = typer.Option(1e-3, "--tol", help="Stop when the lap-time change falls below this threshold."),
    reg_lambda: float | None = typer.Option(None, "--reg-lambda", help="Quadratic regularization on the lateral offset. Defaults to the vehicle config value."),
    save_plots: bool = typer.Option(True, "--save-plots/--no-save-plots", help="Save overlay and convergence plots."),
) -> None:
    """Run the teammate path-optimization flow against the current project code."""
    track_name = _validate_track_name(track)
    config = load_vehicle_config()
    regularization_lambda = config.path_optimization_reg_lambda if reg_lambda is None else reg_lambda
    if max_iters <= 0:
        raise typer.BadParameter("Maximum iterations must be positive.")
    if tol < 0.0:
        raise typer.BadParameter("Tolerance must be non-negative.")
    if regularization_lambda < 0.0:
        raise typer.BadParameter("Regularization must be non-negative.")

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description=f"Optimizing path for {track_name}", total=None)
        track_data = load_track(track_name)
        history = run_iterative_optimization(
            track_data,
            config,
            n_points=point_count,
            max_iters=max_iters,
            tol=tol,
            reg_lambda=regularization_lambda,
        )

    _render_optimization_table(track_name, history)

    if save_plots:
        saved_paths = plot_optimization_results(history=history, track=track_data)
        console.print("[green]Saved plots:[/green]")
        for path in saved_paths:
            console.print(f"- {Path(path)}")


@app.command("sensitivity-track")
def sensitivity_track_command(
    track: str = typer.Option(..., "--track", "-t", prompt="Track name"),
    start_count: int = typer.Option(
        DEFAULT_SENSITIVITY_START_COUNT,
        "--start-count",
        help="First resampled point count in the sensitivity sweep.",
    ),
    end_count: int = typer.Option(
        DEFAULT_SENSITIVITY_END_COUNT,
        "--end-count",
        help="Last resampled point count in the sensitivity sweep.",
    ),
    step_count: int = typer.Option(
        DEFAULT_SENSITIVITY_STEP_COUNT,
        "--step-count",
        help="Increment between successive resampled point counts.",
    ),
    reference_count: int | None = typer.Option(
        None,
        "--reference-count",
        help="Reference resampled point count used for the fine-grid comparison.",
    ),
    tolerance_s: float = typer.Option(0.1, "--tolerance-s", help="Absolute lap-time error tolerance in seconds."),
    save_plots: bool = typer.Option(True, "--save-plots/--no-save-plots", help="Save sensitivity plots."),
) -> None:
    """Sweep resampled point counts for one track and compare lap-time convergence."""
    track_name = _validate_track_name(track)
    point_counts = _build_point_counts(start_count, end_count, step_count)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description=f"Running integration sensitivity study for {track_name}", total=None)
        track_data = load_track(track_name)
        config = load_vehicle_config()
        study = compute_track_integration_sensitivity(
            track_data,
            config,
            point_counts,
            reference_point_count=reference_count,
            tolerance_s=tolerance_s,
        )

    _render_track_integration_sensitivity_table(track_name, study)

    if save_plots:
        saved_paths = save_track_integration_sensitivity_plots(track_data, study)
        console.print("[green]Saved plots:[/green]")
        for path in saved_paths:
            console.print(f"- {Path(path)}")


@app.command("sensitivity-all")
def sensitivity_all_command(
    start_count: int = typer.Option(
        DEFAULT_SENSITIVITY_START_COUNT,
        "--start-count",
        help="First resampled point count in the sensitivity sweep.",
    ),
    end_count: int = typer.Option(
        DEFAULT_SENSITIVITY_END_COUNT,
        "--end-count",
        help="Last resampled point count in the sensitivity sweep.",
    ),
    step_count: int = typer.Option(
        DEFAULT_SENSITIVITY_STEP_COUNT,
        "--step-count",
        help="Increment between successive resampled point counts.",
    ),
    reference_count: int | None = typer.Option(
        None,
        "--reference-count",
        help="Reference resampled point count used for the fine-grid comparison.",
    ),
    tolerance_s: float = typer.Option(0.1, "--tolerance-s", help="Absolute lap-time error tolerance in seconds."),
    save_plot: bool = typer.Option(True, "--save-plot/--no-save-plot", help="Save the all-track sensitivity heatmap."),
) -> None:
    """Run the segmentation sensitivity study across all tracks."""
    point_counts = _build_point_counts(start_count, end_count, step_count)

    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console,
    ) as progress:
        progress.add_task(description="Running all-track integration sensitivity study", total=None)
        config = load_vehicle_config()
        summary = compute_all_tracks_integration_sensitivity(
            config,
            point_counts,
            reference_point_count=reference_count,
            tolerance_s=tolerance_s,
        )

    _render_all_tracks_sensitivity_tables(summary)

    if save_plot:
        saved_path = save_all_tracks_integration_sensitivity_plot(summary)
        console.print("[green]Saved plot:[/green]")
        console.print(f"- {Path(saved_path)}")


if __name__ == "__main__":
    app()