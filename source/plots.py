from __future__ import annotations

from pathlib import Path
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from .data_loader import PROJECT_ROOT, TrackData
from .curvature import CurvatureComparison
from .geometry import TrackAudit, cumulative_arc_length
from .speed_profile import SpeedProfileResult


FIGURES_DIR = PROJECT_ROOT / "outputs" / "figures"


def save_audit_plots(track: TrackData, audit: TrackAudit) -> list[Path]:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    saved_paths = [
        _save_track_geometry_plot(track),
        _save_segment_length_plot(track, audit),
        _save_segment_length_histogram(track, audit),
        _save_width_profile_plot(track, audit),
    ]
    plt.close("all")
    return saved_paths


def _save_track_geometry_plot(track: TrackData) -> Path:
    fig, ax = plt.subplots(figsize=(8, 8))
    left_x, left_y, right_x, right_y = _compute_track_boundaries(track)
    ax.plot(right_x, right_y, color="darkorange", linewidth=1.2, label="Right boundary")
    ax.plot(left_x, left_y, color="seagreen", linewidth=1.2, label="Left boundary")
    ax.plot(track.x, track.y, color="navy", linewidth=1.5, label="Centerline")
    ax.scatter(track.x[0], track.y[0], color="crimson", s=50, label="Start")
    ax.set_title(f"{track.name} Centerline and Track Boundaries")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    output_path = FIGURES_DIR / f"{track.name}_centerline.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _compute_track_boundaries(track: TrackData) -> tuple[list[float], list[float], list[float], list[float]]:
    point_count = track.point_count
    left_x: list[float] = []
    left_y: list[float] = []
    right_x: list[float] = []
    right_y: list[float] = []

    for index in range(point_count):
        previous_index = (index - 1) % point_count
        next_index = (index + 1) % point_count
        dx = track.x[next_index] - track.x[previous_index]
        dy = track.y[next_index] - track.y[previous_index]
        norm = math.hypot(dx, dy)

        if math.isclose(norm, 0.0):
            tangent_x, tangent_y = 1.0, 0.0
        else:
            tangent_x, tangent_y = dx / norm, dy / norm

        normal_x = -tangent_y
        normal_y = tangent_x

        left_x.append(track.x[index] + track.width_left[index] * normal_x)
        left_y.append(track.y[index] + track.width_left[index] * normal_y)
        right_x.append(track.x[index] - track.width_right[index] * normal_x)
        right_y.append(track.y[index] - track.width_right[index] * normal_y)

    return left_x, left_y, right_x, right_y


def _save_segment_length_plot(track: TrackData, audit: TrackAudit) -> Path:
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(range(len(audit.segment_lengths_m)), audit.segment_lengths_m, color="darkgreen")
    ax.set_title(f"{track.name} Segment Lengths")
    ax.set_xlabel("Segment index")
    ax.set_ylabel("Length [m]")
    ax.grid(True, alpha=0.3)
    output_path = FIGURES_DIR / f"{track.name}_segment_lengths.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_segment_length_histogram(track: TrackData, audit: TrackAudit) -> Path:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(audit.segment_lengths_m, bins=min(30, len(audit.segment_lengths_m)), color="slateblue", edgecolor="black")
    ax.set_title(f"{track.name} Segment Length Distribution")
    ax.set_xlabel("Length [m]")
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.3)
    output_path = FIGURES_DIR / f"{track.name}_segment_length_histogram.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_width_profile_plot(track: TrackData, audit: TrackAudit) -> Path:
    arc_length = cumulative_arc_length(audit.segment_lengths_m)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(arc_length[:-1], track.width_right[:-1], label="Right width", color="darkorange")
    ax.plot(arc_length[:-1], track.width_left[:-1], label="Left width", color="teal")
    ax.set_title(f"{track.name} Track Widths")
    ax.set_xlabel("Arc length [m]")
    ax.set_ylabel("Width [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    output_path = FIGURES_DIR / f"{track.name}_widths.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def save_curvature_comparison_plots(track: TrackData, comparison: CurvatureComparison) -> list[Path]:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    saved_paths = [
        _save_curvature_profile_plot(track, comparison),
        _save_curvature_difference_plot(track, comparison),
        _save_resampled_geometry_plot(track, comparison),
    ]
    plt.close("all")
    return saved_paths


def _save_curvature_profile_plot(track: TrackData, comparison: CurvatureComparison) -> Path:
    fig, ax = plt.subplots(figsize=(10, 4))
    raw_normalized_s = [value / comparison.raw.total_length_m for value in comparison.raw.s]
    resampled_normalized_s = [value / comparison.resampled.total_length_m for value in comparison.resampled.s]
    ax.plot(raw_normalized_s, comparison.raw.curvature, label="Raw central difference", color="navy", linewidth=1.1)
    ax.plot(resampled_normalized_s, comparison.resampled.curvature, label="Resampled central difference", color="crimson", linewidth=1.1, alpha=0.8)
    ax.set_title(f"{track.name} Curvature Method Comparison")
    ax.set_xlabel("Normalized lap position s/L")
    ax.set_ylabel("Curvature [1/m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    output_path = FIGURES_DIR / f"{track.name}_curvature_comparison.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_curvature_difference_plot(track: TrackData, comparison: CurvatureComparison) -> Path:
    comparison_count = min(len(comparison.raw.curvature), len(comparison.resampled.curvature))
    normalized_s = [index / comparison_count for index in range(comparison_count)]
    differences = [comparison.raw.curvature[index] - comparison.resampled.curvature[index] for index in range(comparison_count)]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(normalized_s, differences, color="purple", linewidth=1.1)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title(f"{track.name} Raw Minus Resampled Curvature")
    ax.set_xlabel("Normalized sample index")
    ax.set_ylabel("Curvature difference [1/m]")
    ax.grid(True, alpha=0.3)
    output_path = FIGURES_DIR / f"{track.name}_curvature_difference.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_resampled_geometry_plot(track: TrackData, comparison: CurvatureComparison) -> Path:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(track.x, track.y, color="lightgray", linewidth=2.0, label="Raw centerline")
    ax.scatter(comparison.resampled.x, comparison.resampled.y, color="crimson", s=4, label="Resampled points")
    ax.scatter(track.x[0], track.y[0], color="navy", s=50, label="Start")
    ax.set_title(f"{track.name} Arc-Length Resampling Check")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(markerscale=2)
    output_path = FIGURES_DIR / f"{track.name}_resampled_geometry.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def save_speed_profile_plots(track: TrackData, result: SpeedProfileResult) -> list[Path]:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    saved_paths = [
        _save_speed_map_plot(track, result),
        _save_speed_profile_plot(track, result),
        _save_acceleration_profile_plot(track, result),
        _save_integration_comparison_plot(track, result),
    ]
    plt.close("all")
    return saved_paths


def _save_speed_map_plot(track: TrackData, result: SpeedProfileResult) -> Path:
    point_count = len(result.x_path_m)
    segment_speeds = result.final_speed_mps[:-1]
    points = [(result.x_path_m[index], result.y_path_m[index]) for index in range(point_count)]
    segments = [
        [points[index], points[(index + 1) % point_count]]
        for index in range(point_count)
    ]

    fig, ax = plt.subplots(figsize=(8, 8))
    line_collection = LineCollection(segments, cmap="jet", linewidths=2.4)
    line_collection.set_array(segment_speeds)
    line_collection.set_clim(min(segment_speeds), max(segment_speeds))
    ax.add_collection(line_collection)

    accel_points, brake_points = _compute_event_points(result)
    if accel_points:
        ax.scatter(
            [point[0] for point in accel_points],
            [point[1] for point in accel_points],
            color="red",
            s=28,
            zorder=3,
            label="Acceleration start",
        )
    if brake_points:
        ax.scatter(
            [point[0] for point in brake_points],
            [point[1] for point in brake_points],
            color="blue",
            s=28,
            zorder=3,
            label="Brake start",
        )

    ax.scatter(result.x_path_m[0], result.y_path_m[0], color="white", edgecolors="black", s=45, zorder=3, label="Start")
    arrow_dx, arrow_dy = _compute_start_direction_arrow(result)
    ax.annotate(
        "",
        xy=(result.x_path_m[0] + arrow_dx, result.y_path_m[0] + arrow_dy),
        xytext=(result.x_path_m[0], result.y_path_m[0]),
        arrowprops={"arrowstyle": "->", "color": "black", "lw": 1.5},
        zorder=4,
    )
    ax.set_title(f"{track.name} Reference Trajectory Colored by Speed")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()

    colorbar = fig.colorbar(line_collection, ax=ax)
    colorbar.set_label("Speed (m/s)")

    output_path = FIGURES_DIR / f"{track.name}_speed_map.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _compute_start_direction_arrow(result: SpeedProfileResult) -> tuple[float, float]:
    if len(result.x_path_m) < 2:
        return 0.0, 0.0

    dx = result.x_path_m[1] - result.x_path_m[0]
    dy = result.y_path_m[1] - result.y_path_m[0]
    norm = math.hypot(dx, dy)
    if math.isclose(norm, 0.0):
        return 0.0, 0.0

    arrow_length = 0.02 * result.total_length_m
    return arrow_length * dx / norm, arrow_length * dy / norm


def _compute_event_points(result: SpeedProfileResult, threshold: float = 1e-3) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    accel_points: list[tuple[float, float]] = []
    brake_points: list[tuple[float, float]] = []
    previous_state = "neutral"
    point_count = len(result.x_path_m)

    for index, acceleration in enumerate(result.longitudinal_accel_mps2):
        if acceleration > threshold:
            current_state = "accelerating"
        elif acceleration < -threshold:
            current_state = "braking"
        else:
            current_state = "neutral"

        if current_state != previous_state:
            next_index = (index + 1) % point_count
            midpoint = (
                0.5 * (result.x_path_m[index] + result.x_path_m[next_index]),
                0.5 * (result.y_path_m[index] + result.y_path_m[next_index]),
            )
            if current_state == "accelerating":
                accel_points.append(midpoint)
            elif current_state == "braking":
                brake_points.append(midpoint)

        previous_state = current_state

    return accel_points, brake_points


def _save_speed_profile_plot(track: TrackData, result: SpeedProfileResult) -> Path:
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(result.s_nodes_m, result.speed_cap_mps, color="gray", linewidth=1.0, label="Lateral speed cap")
    ax.plot(result.s_nodes_m, result.forward_speed_mps, color="darkorange", linewidth=1.0, label="Forward pass")
    ax.plot(result.s_nodes_m, result.final_speed_mps, color="navy", linewidth=1.4, label="Final speed")
    ax.set_title(f"{track.name} Standing-Start Speed Profile")
    ax.set_xlabel("Arc length (m)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    output_path = FIGURES_DIR / f"{track.name}_speed_profile.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_acceleration_profile_plot(track: TrackData, result: SpeedProfileResult) -> Path:
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(result.s_midpoints_m, result.longitudinal_accel_mps2, color="crimson", linewidth=1.1, label="Longitudinal acceleration")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title(f"{track.name} Longitudinal Acceleration")
    ax.set_xlabel("Arc length (m)")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    output_path = FIGURES_DIR / f"{track.name}_longitudinal_acceleration.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path


def _save_integration_comparison_plot(track: TrackData, result: SpeedProfileResult) -> Path:
    labels = ["Kinematic", "Left rule", "Trapezoidal"]
    values = [
        result.integration.kinematic_time_s,
        result.integration.left_rule_time_s,
        result.integration.trapezoidal_time_s,
    ]
    if result.integration.simpson_time_s is not None:
        labels.append("Simpson")
        values.append(result.integration.simpson_time_s)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(labels, values, color=["navy", "slateblue", "darkorange", "seagreen"][: len(labels)])
    ax.set_title(f"{track.name} Lap-Time Integration Comparison")
    ax.set_ylabel("Lap time estimate (s)")
    ax.grid(True, axis="y", alpha=0.3)
    output_path = FIGURES_DIR / f"{track.name}_integration_comparison.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    return output_path