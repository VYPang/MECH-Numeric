from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
import multiprocessing
import os
import sys
import time

import numpy as np

from .config import VehicleConfig
from .curvature import periodic_central_difference_curvature
from .data_loader import TrackData
from .path_optimization import prepare_centerline_data
from .speed_profile import compute_speed_profile


@dataclass(frozen=True)
class PathOptimizationIterationB:
    iteration: int
    e_opt: np.ndarray
    x_path: np.ndarray
    y_path: np.ndarray
    lap_time_s: float
    grad_norm: float
    alpha: float


def compute_lap_time(
    e_offset: np.ndarray,
    x_center: np.ndarray,
    y_center: np.ndarray,
    normal_x: np.ndarray,
    normal_y: np.ndarray,
    config: VehicleConfig,
) -> tuple[float, float]:
    x_path = x_center - e_offset * normal_x
    y_path = y_center - e_offset * normal_y

    profile = periodic_central_difference_curvature(
        list(x_path),
        list(y_path),
        total_length_m=None,
    )
    result = compute_speed_profile(profile, config)
    return result.integration.trapezoidal_time_s, result.total_length_m


def resolve_parallel_workers(requested_jobs: int | None = None) -> int:
    available = os.cpu_count() or 1
    if requested_jobs is None or requested_jobs <= 0:
        return max(1, min(8, available - 1 if available > 1 else 1))
    return max(1, min(requested_jobs, available))


def _gradient_worker_direct(args: tuple) -> tuple[int, float]:
    index, e_offset, eps, x_center, y_center, normal_x, normal_y, width_left, width_right, config = args

    e_work = e_offset.copy()
    e_work[index] = e_offset[index] + eps
    t_plus, _ = compute_lap_time(
        np.clip(e_work, -width_left, width_right),
        x_center,
        y_center,
        normal_x,
        normal_y,
        config,
    )

    e_work[index] = e_offset[index] - eps
    t_minus, _ = compute_lap_time(
        np.clip(e_work, -width_left, width_right),
        x_center,
        y_center,
        normal_x,
        normal_y,
        config,
    )

    return index, (t_plus - t_minus) / (2.0 * eps)


def finite_diff_gradient_direct(
    e_offset: np.ndarray,
    x_center: np.ndarray,
    y_center: np.ndarray,
    normal_x: np.ndarray,
    normal_y: np.ndarray,
    width_left: np.ndarray,
    width_right: np.ndarray,
    config: VehicleConfig,
    eps: float = 0.01,
    n_jobs: int = 1,
) -> np.ndarray:
    point_count = len(e_offset)
    gradient = np.zeros(point_count)

    if n_jobs > 1:
        if sys.platform == "darwin":
            mp_context = multiprocessing.get_context("spawn")
        else:
            mp_context = multiprocessing.get_context("fork")

        tasks = [
            (index, e_offset, eps, x_center, y_center, normal_x, normal_y, width_left, width_right, config)
            for index in range(point_count)
        ]
        with ProcessPoolExecutor(max_workers=n_jobs, mp_context=mp_context) as executor:
            for index, value in executor.map(_gradient_worker_direct, tasks):
                gradient[index] = value
        return gradient

    e_work = e_offset.copy()
    for index in range(point_count):
        e_work[index] = e_offset[index] + eps
        t_plus, _ = compute_lap_time(
            np.clip(e_work, -width_left, width_right),
            x_center,
            y_center,
            normal_x,
            normal_y,
            config,
        )

        e_work[index] = e_offset[index] - eps
        t_minus, _ = compute_lap_time(
            np.clip(e_work, -width_left, width_right),
            x_center,
            y_center,
            normal_x,
            normal_y,
            config,
        )

        e_work[index] = e_offset[index]
        gradient[index] = (t_plus - t_minus) / (2.0 * eps)

    return gradient


def armijo_backtrack(
    e_offset: np.ndarray,
    gradient: np.ndarray,
    x_center: np.ndarray,
    y_center: np.ndarray,
    normal_x: np.ndarray,
    normal_y: np.ndarray,
    width_left: np.ndarray,
    width_right: np.ndarray,
    config: VehicleConfig,
    lap_time_current: float,
    alpha0: float = 1.0,
    beta: float = 0.5,
    c1: float = 1e-4,
    max_backtracks: int = 20,
) -> tuple[float, float]:
    grad_sq = float(np.sum(gradient**2))
    if grad_sq < 1e-16:
        return 0.0, lap_time_current

    alpha = alpha0
    for _ in range(max_backtracks):
        e_trial = np.clip(e_offset - alpha * gradient, -width_left, width_right)
        lap_time_trial, _ = compute_lap_time(e_trial, x_center, y_center, normal_x, normal_y, config)
        if lap_time_trial <= lap_time_current - c1 * alpha * grad_sq:
            return alpha, lap_time_trial
        alpha *= beta

    return 0.0, lap_time_current


def run_iterative_time_optimization(
    track: TrackData,
    config: VehicleConfig,
    n_points: int | None = None,
    max_iters: int = 50,
    eps: float = 0.01,
    alpha0: float = 1.0,
    gtol: float = 1e-4,
    ftol: float = 1e-3,
    e_init: np.ndarray | None = None,
    n_jobs: int | None = None,
    beta: float = 0.5,
    verbose: bool = False,
) -> list[PathOptimizationIterationB]:
    x_center, y_center, _, normal_x, normal_y, width_left, width_right, _ = prepare_centerline_data(
        track,
        n_points,
    )
    point_count = len(x_center)
    worker_count = resolve_parallel_workers(n_jobs)

    e_offset = e_init.copy() if e_init is not None else np.zeros(point_count)
    e_offset = np.clip(e_offset, -width_left, width_right)
    lap_time_start, _ = compute_lap_time(e_offset, x_center, y_center, normal_x, normal_y, config)

    x_path = x_center - e_offset * normal_x
    y_path = y_center - e_offset * normal_y
    history: list[PathOptimizationIterationB] = [
        PathOptimizationIterationB(
            iteration=0,
            e_opt=e_offset.copy(),
            x_path=x_path.copy(),
            y_path=y_path.copy(),
            lap_time_s=lap_time_start,
            grad_norm=0.0,
            alpha=0.0,
        )
    ]

    alpha = alpha0
    lap_time_prev = lap_time_start
    for iteration in range(1, max_iters + 1):
        if verbose:
            started = time.perf_counter()
        gradient = finite_diff_gradient_direct(
            e_offset,
            x_center,
            y_center,
            normal_x,
            normal_y,
            width_left,
            width_right,
            config,
            eps=eps,
            n_jobs=worker_count,
        )
        grad_norm = float(np.linalg.norm(gradient))
        if grad_norm < gtol:
            break

        alpha, _ = armijo_backtrack(
            e_offset,
            gradient,
            x_center,
            y_center,
            normal_x,
            normal_y,
            width_left,
            width_right,
            config,
            lap_time_current=lap_time_prev,
            alpha0=alpha * 1.5,
            beta=beta,
        )
        if alpha == 0.0:
            break

        e_offset = np.clip(e_offset - alpha * gradient, -width_left, width_right)
        x_path = x_center - e_offset * normal_x
        y_path = y_center - e_offset * normal_y
        lap_time_new, _ = compute_lap_time(e_offset, x_center, y_center, normal_x, normal_y, config)

        history.append(
            PathOptimizationIterationB(
                iteration=iteration,
                e_opt=e_offset.copy(),
                x_path=x_path.copy(),
                y_path=y_path.copy(),
                lap_time_s=lap_time_new,
                grad_norm=grad_norm,
                alpha=alpha,
            )
        )

        if verbose:
            elapsed = time.perf_counter() - started
            print(f"iter {iteration}: T={lap_time_new:.4f}s |grad|={grad_norm:.3e} alpha={alpha:.3e} ({elapsed:.1f}s)")

        if abs(lap_time_new - lap_time_prev) < ftol:
            break
        lap_time_prev = lap_time_new

    return history