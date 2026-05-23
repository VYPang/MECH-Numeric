from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

from .config import VehicleConfig
from .curvature import periodic_central_difference_curvature
from .data_loader import TrackData
from .path_optimization import build_second_diff_matrix, prepare_centerline_data
from .speed_profile import compute_speed_profile


@dataclass(frozen=True)
class PathOptimizationIterationC:
    iteration: int
    e_opt: np.ndarray
    x_path: np.ndarray
    y_path: np.ndarray
    lap_time_s: float
    total_length_m: float


def solve_qp_apg(
    kappa_center: np.ndarray,
    ds: float,
    width_left: np.ndarray,
    width_right: np.ndarray,
    second_diff: sp.coo_matrix,
    reg_lambda: float = 1e-6,
    max_iters: int = 2000,
    tol: float = 1e-6,
    e_init: np.ndarray | None = None,
) -> np.ndarray:
    point_count = len(kappa_center)
    second_diff_csc = second_diff.tocsc()
    lambda_max_bound = 16.0 / ds**4 + reg_lambda
    lipschitz = 2.0 * lambda_max_bound
    step = 1.0 / lipschitz
    b = second_diff_csc.T @ kappa_center / ds**2

    if e_init is not None:
        e_offset = np.clip(e_init.copy(), -width_left, width_right)
    else:
        e_offset = np.zeros(point_count)

    kappa_e = kappa_center - second_diff_csc @ e_offset / ds**2
    objective_prev = float(np.sum(kappa_e**2) + reg_lambda * np.sum(e_offset**2))
    y = e_offset.copy()
    momentum = 1.0
    best_offset = e_offset.copy()
    best_objective = objective_prev

    for _ in range(max_iters):
        dy = second_diff_csc @ y
        gradient = 2.0 * (second_diff_csc.T @ dy / ds**4 + reg_lambda * y - b)
        e_new = np.clip(y - step * gradient, -width_left, width_right)

        kappa_new = kappa_center - second_diff_csc @ e_new / ds**2
        objective_new = float(np.sum(kappa_new**2) + reg_lambda * np.sum(e_new**2))
        if objective_new < best_objective:
            best_objective = objective_new
            best_offset = e_new.copy()

        if objective_new > objective_prev:
            y = e_new.copy()
            momentum = 1.0
        else:
            momentum_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * momentum * momentum))
            beta = (momentum - 1.0) / momentum_new
            y = e_new + beta * (e_new - e_offset)
            momentum = momentum_new

        if np.max(np.abs(e_new - e_offset)) < tol:
            return e_new

        e_offset = e_new
        objective_prev = objective_new

    return best_offset


def _compute_lap_time_for_e(
    e_offset: np.ndarray,
    x_center: np.ndarray,
    y_center: np.ndarray,
    normal_x: np.ndarray,
    normal_y: np.ndarray,
    config: VehicleConfig,
) -> tuple[float, float]:
    x_path = x_center - e_offset * normal_x
    y_path = y_center - e_offset * normal_y
    total_length = float(
        np.sum(
            np.sqrt(
                np.diff(x_path, append=x_path[0]) ** 2
                + np.diff(y_path, append=y_path[0]) ** 2
            )
        )
    )
    profile = periodic_central_difference_curvature(
        list(x_path),
        list(y_path),
        total_length_m=total_length,
    )
    result = compute_speed_profile(profile, config)
    return float(result.integration.trapezoidal_time_s), total_length


def run_apg_optimization(
    track: TrackData,
    config: VehicleConfig,
    n_points: int | None = None,
    max_iters: int = 3,
    tol: float = 1e-3,
    reg_lambda: float = 1e-6,
    apg_max_iters: int = 2000,
    apg_tol: float = 1e-6,
) -> list[PathOptimizationIterationC]:
    x_center, y_center, kappa_center, normal_x, normal_y, width_left, width_right, ds = prepare_centerline_data(
        track,
        n_points,
    )
    point_count = len(x_center)
    second_diff = build_second_diff_matrix(point_count)

    e_offset = np.zeros(point_count)
    baseline_time, baseline_length = _compute_lap_time_for_e(
        e_offset,
        x_center,
        y_center,
        normal_x,
        normal_y,
        config,
    )
    history: list[PathOptimizationIterationC] = [
        PathOptimizationIterationC(
            iteration=0,
            e_opt=e_offset.copy(),
            x_path=x_center.copy(),
            y_path=y_center.copy(),
            lap_time_s=baseline_time,
            total_length_m=baseline_length,
        )
    ]

    for iteration in range(1, max_iters + 1):
        e_offset = solve_qp_apg(
            kappa_center,
            ds,
            width_left,
            width_right,
            second_diff,
            reg_lambda=reg_lambda,
            max_iters=apg_max_iters,
            tol=apg_tol,
            e_init=e_offset,
        )

        x_path = x_center - e_offset * normal_x
        y_path = y_center - e_offset * normal_y
        lap_time_new, total_length = _compute_lap_time_for_e(
            e_offset,
            x_center,
            y_center,
            normal_x,
            normal_y,
            config,
        )
        history.append(
            PathOptimizationIterationC(
                iteration=iteration,
                e_opt=e_offset.copy(),
                x_path=x_path.copy(),
                y_path=y_path.copy(),
                lap_time_s=lap_time_new,
                total_length_m=total_length,
            )
        )

        ds = total_length / point_count
        if abs(history[-1].lap_time_s - history[-2].lap_time_s) < tol:
            break

    return history