from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import warnings

import matplotlib
import numpy as np
import scipy.sparse as sp

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .config import VehicleConfig
from .curvature import (
    closed_segment_lengths,
    cumulative_values,
    periodic_central_difference_curvature,
    resample_closed_path_by_arc_length,
)
from .data_loader import PROJECT_ROOT, TrackData
from .speed_profile import compute_speed_profile


FIGURES_DIR = PROJECT_ROOT / "outputs" / "figures"


@dataclass(frozen=True)
class PathOptimizationIteration:
    iteration: int
    e_opt: np.ndarray
    x_path: np.ndarray
    y_path: np.ndarray
    lap_time_s: float
    total_length_m: float


def prepare_centerline_data(
    track: TrackData,
    n_points: int | None = None,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    float,
]:
    if n_points is None:
        point_count = track.point_count
        if point_count % 2 != 0:
            point_count += 1
    else:
        if n_points < 3:
            raise ValueError(f"Need at least 3 points, got {n_points}.")
        point_count = n_points

    x_list, y_list, s_list, total_length = resample_closed_path_by_arc_length(
        track.x,
        track.y,
        point_count,
    )
    x_center = np.array(x_list, dtype=float)
    y_center = np.array(y_list, dtype=float)

    profile_center = periodic_central_difference_curvature(
        list(x_center),
        list(y_center),
        total_length_m=total_length,
    )
    kappa_center = np.array(profile_center.curvature, dtype=float)

    normal_x, normal_y = compute_normals(x_center, y_center)
    width_left, width_right = interpolate_widths_at_resampled(track, s_list)
    ds0 = total_length / point_count

    return x_center, y_center, kappa_center, normal_x, normal_y, width_left, width_right, ds0


def compute_normals(x_path: np.ndarray, y_path: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    point_count = len(x_path)
    normal_x = np.zeros(point_count)
    normal_y = np.zeros(point_count)

    for index in range(point_count):
        next_index = (index + 1) % point_count
        previous_index = (index - 1) % point_count
        dx = x_path[next_index] - x_path[previous_index]
        dy = y_path[next_index] - y_path[previous_index]
        norm = math.hypot(dx, dy)
        if math.isclose(norm, 0.0):
            normal_x[index] = 1.0
            normal_y[index] = 0.0
        else:
            normal_x[index] = -dy / norm
            normal_y[index] = dx / norm

    return normal_x, normal_y


def interpolate_widths_at_resampled(
    track: TrackData,
    resampled_s: list[float],
) -> tuple[np.ndarray, np.ndarray]:
    segment_lengths = closed_segment_lengths(track.x, track.y)
    original_s = np.array(cumulative_values(segment_lengths)[:-1], dtype=float)
    target_s = np.array(resampled_s, dtype=float)
    width_left = np.interp(target_s, original_s, np.array(track.width_left, dtype=float))
    width_right = np.interp(target_s, original_s, np.array(track.width_right, dtype=float))
    return width_left, width_right


def build_second_diff_matrix(point_count: int) -> sp.coo_matrix:
    data: list[float] = []
    rows: list[int] = []
    cols: list[int] = []
    for index in range(point_count):
        rows.extend([index, index, index])
        cols.extend([(index - 1) % point_count, index, (index + 1) % point_count])
        data.extend([1.0, -2.0, 1.0])
    return sp.coo_matrix((data, (rows, cols)), shape=(point_count, point_count))


def optimize_path_qp(
    kappa_center: np.ndarray,
    ds: float,
    width_left: np.ndarray,
    width_right: np.ndarray,
    second_diff: sp.coo_matrix,
    warm_start: np.ndarray | None = None,
    reg_lambda: float = 1e-6,
) -> np.ndarray:
    try:
        import cvxpy as cp

        return _optimize_path_qp_cvxpy(
            cp,
            kappa_center,
            ds,
            width_left,
            width_right,
            second_diff,
            warm_start,
            reg_lambda,
        )
    except ImportError:
        return _optimize_path_qp_scipy(
            kappa_center,
            ds,
            width_left,
            width_right,
            second_diff,
            warm_start,
            reg_lambda,
        )


def _optimize_path_qp_cvxpy(
    cp,
    kappa_center: np.ndarray,
    ds: float,
    width_left: np.ndarray,
    width_right: np.ndarray,
    second_diff: sp.coo_matrix,
    warm_start: np.ndarray | None = None,
    reg_lambda: float = 1e-6,
) -> np.ndarray:
    point_count = len(kappa_center)
    e_offset = cp.Variable(point_count)
    kappa = kappa_center - (1.0 / ds**2) * (second_diff @ e_offset)
    objective = cp.Minimize(cp.sum_squares(kappa) + reg_lambda * cp.sum_squares(e_offset))
    constraints = [e_offset >= -width_left, e_offset <= width_right]
    problem = cp.Problem(objective, constraints)

    solver_kwargs: dict[str, object] = {}
    if warm_start is not None:
        e_offset.value = warm_start
        solver_kwargs["warm_start"] = True

    try:
        problem.solve(solver=cp.OSQP, **solver_kwargs)
    except (cp.error.SolverError, Exception):
        warnings.warn("OSQP failed, retrying without warm start.", stacklevel=2)
        problem.solve(solver=cp.OSQP)

    if e_offset.value is None:
        raise RuntimeError(
            "QP solver did not converge. Check track data or try a different point count."
        )
    return np.array(e_offset.value, dtype=float)


def _optimize_path_qp_scipy(
    kappa_center: np.ndarray,
    ds: float,
    width_left: np.ndarray,
    width_right: np.ndarray,
    second_diff: sp.coo_matrix,
    warm_start: np.ndarray | None = None,
    reg_lambda: float = 1e-6,
) -> np.ndarray:
    from scipy.optimize import Bounds, minimize

    point_count = len(kappa_center)
    x0 = warm_start if warm_start is not None else np.zeros(point_count)
    second_diff_dense = second_diff.toarray()

    def objective(e_offset: np.ndarray) -> float:
        kappa = kappa_center - second_diff_dense @ e_offset / ds**2
        return float(np.sum(kappa**2) + reg_lambda * np.sum(e_offset**2))

    def gradient(e_offset: np.ndarray) -> np.ndarray:
        kappa = kappa_center - second_diff_dense @ e_offset / ds**2
        return -2.0 * second_diff_dense.T @ kappa / ds**2 + 2.0 * reg_lambda * e_offset

    bounds = Bounds(lb=-width_left, ub=width_right)
    result = minimize(
        objective,
        x0,
        jac=gradient,
        bounds=bounds,
        method="L-BFGS-B",
        options={"ftol": 1e-10, "gtol": 1e-8, "maxiter": 1000},
    )
    if not result.success:
        warnings.warn(f"scipy L-BFGS-B did not converge: {result.message}", stacklevel=2)
    return np.array(result.x, dtype=float)


def run_iterative_optimization(
    track: TrackData,
    config: VehicleConfig,
    n_points: int | None = None,
    max_iters: int = 3,
    tol: float = 1e-3,
    reg_lambda: float = 1e-6,
) -> list[PathOptimizationIteration]:
    x_center, y_center, kappa_center, normal_x, normal_y, width_left, width_right, ds0 = prepare_centerline_data(
        track,
        n_points,
    )
    point_count = len(x_center)
    second_diff = build_second_diff_matrix(point_count)

    baseline_profile = periodic_central_difference_curvature(
        list(x_center),
        list(y_center),
        total_length_m=float(point_count * ds0),
    )
    baseline_result = compute_speed_profile(baseline_profile, config)
    current_length = baseline_result.total_length_m

    history: list[PathOptimizationIteration] = [
        PathOptimizationIteration(
            iteration=0,
            e_opt=np.zeros(point_count),
            x_path=x_center.copy(),
            y_path=y_center.copy(),
            lap_time_s=baseline_result.integration.trapezoidal_time_s,
            total_length_m=current_length,
        )
    ]

    warm_start: np.ndarray | None = None
    for iteration in range(1, max_iters + 1):
        ds = current_length / point_count

        try:
            e_opt = optimize_path_qp(
                kappa_center,
                ds,
                width_left,
                width_right,
                second_diff,
                warm_start,
                reg_lambda,
            )
        except RuntimeError:
            break

        x_new = x_center - e_opt * normal_x
        y_new = y_center - e_opt * normal_y

        optimized_profile = periodic_central_difference_curvature(
            list(x_new),
            list(y_new),
            total_length_m=None,
        )
        optimized_result = compute_speed_profile(optimized_profile, config)

        history.append(
            PathOptimizationIteration(
                iteration=iteration,
                e_opt=e_opt,
                x_path=x_new,
                y_path=y_new,
                lap_time_s=optimized_result.integration.trapezoidal_time_s,
                total_length_m=optimized_result.total_length_m,
            )
        )

        warm_start = e_opt
        current_length = optimized_result.total_length_m

        if abs(history[-1].lap_time_s - history[-2].lap_time_s) < tol:
            break

    return history


def plot_optimization_results(
    history: list[PathOptimizationIteration],
    track: TrackData,
    output_dir: Path | None = None,
) -> tuple[Path, Path]:
    output_dir = output_dir or FIGURES_DIR
    output_dir.mkdir(parents=True, exist_ok=True)

    overlay_path = _save_path_overlay_plot(track, history, output_dir)
    convergence_path = _save_convergence_plot(history, track.name, output_dir)

    plt.close("all")
    return overlay_path, convergence_path


def _compute_track_boundaries(track: TrackData) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    point_count = track.point_count
    left_x = np.zeros(point_count)
    left_y = np.zeros(point_count)
    right_x = np.zeros(point_count)
    right_y = np.zeros(point_count)

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
        left_x[index] = track.x[index] + track.width_left[index] * normal_x
        left_y[index] = track.y[index] + track.width_left[index] * normal_y
        right_x[index] = track.x[index] - track.width_right[index] * normal_x
        right_y[index] = track.y[index] - track.width_right[index] * normal_y

    return left_x, left_y, right_x, right_y


def _save_path_overlay_plot(
    track: TrackData,
    history: list[PathOptimizationIteration],
    output_dir: Path,
) -> Path:
    fig, ax = plt.subplots(figsize=(14, 12))

    left_x, left_y, right_x, right_y = _compute_track_boundaries(track)
    ax.fill(
        list(left_x) + list(right_x)[::-1],
        list(left_y) + list(right_y)[::-1],
        color="wheat",
        alpha=0.35,
        edgecolor="darkgray",
        linewidth=0.3,
        label="Track boundary",
    )
    ax.plot(
        track.x,
        track.y,
        color="silver",
        linewidth=0.6,
        linestyle="--",
        alpha=0.7,
        label="Centerline",
    )

    color_map = plt.cm.plasma
    iteration_count = len(history)
    for index, entry in enumerate(history):
        if index == 0:
            color = "dimgray"
            linewidth = 0.8
            label = f"Iter 0 (baseline): {entry.lap_time_s:.2f}s"
            zorder = 2
        elif index == iteration_count - 1:
            color = color_map(0.95)
            linewidth = 1.6
            label = f"Iter {entry.iteration} (final): {entry.lap_time_s:.2f}s"
            zorder = 4
        else:
            fraction = index / max(iteration_count - 1, 1)
            color = color_map(0.2 + 0.7 * fraction)
            linewidth = 1.0
            label = f"Iter {entry.iteration}: {entry.lap_time_s:.2f}s"
            zorder = 3

        ax.plot(
            list(entry.x_path),
            list(entry.y_path),
            color=color,
            linewidth=linewidth,
            label=label,
            zorder=zorder,
        )

    ax.scatter(track.x[0], track.y[0], color="black", s=80, zorder=6, label="Start")
    ax.set_title(f"{track.name} Path Optimization Overlay")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.2)
    ax.legend(fontsize=8, loc="lower right", framealpha=0.85)

    output_path = output_dir / f"{track.name}_path_optimization_overlay.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    return output_path


def _save_convergence_plot(
    history: list[PathOptimizationIteration],
    track_name: str,
    output_dir: Path,
) -> Path:
    fig, ax1 = plt.subplots(figsize=(9, 5.5))

    iterations = [entry.iteration for entry in history]
    lap_times = [entry.lap_time_s for entry in history]
    path_lengths = [entry.total_length_m for entry in history]

    time_color = "navy"
    ax1.plot(iterations, lap_times, marker="o", color=time_color, linewidth=2.0, markersize=8)
    baseline_time = lap_times[0]
    ax1.axhline(y=baseline_time, color="gray", linewidth=0.8, linestyle="--", alpha=0.5)
    for index, lap_time in enumerate(lap_times):
        delta = lap_time - baseline_time
        label = f"{lap_time:.3f}s" if index == 0 else f"{lap_time:.3f}s ({delta:+.3f}s)"
        ax1.annotate(
            label,
            (iterations[index], lap_time),
            textcoords="offset points",
            xytext=(0, 12 if index == 0 else -16),
            fontsize=8,
            ha="center",
            color=time_color,
        )
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Lap time [s]", color=time_color, fontsize=10)
    ax1.tick_params(axis="y", labelcolor=time_color)
    ax1.set_xticks(iterations)
    ax1.set_xticklabels([str(iteration) for iteration in iterations])

    ax2 = ax1.twinx()
    length_color = "darkorange"
    ax2.plot(
        iterations,
        path_lengths,
        marker="s",
        color=length_color,
        linewidth=1.6,
        markersize=7,
        linestyle="--",
    )
    ax2.set_ylabel("Path length [m]", color=length_color, fontsize=10)
    ax2.tick_params(axis="y", labelcolor=length_color)

    ax1.set_title(f"{track_name} Lap Time and Path Length Convergence")
    ax1.grid(True, alpha=0.2)
    ax1.legend(
        [
            plt.Line2D([0], [0], color=time_color, marker="o", linewidth=2.0),
            plt.Line2D([0], [0], color=length_color, marker="s", linewidth=1.6, linestyle="--"),
        ],
        ["Lap time", "Path length"],
        loc="best",
        fontsize=9,
    )

    output_path = output_dir / f"{track_name}_path_optimization_convergence.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    return output_path