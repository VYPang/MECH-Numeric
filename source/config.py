from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .data_loader import PROJECT_ROOT


DEFAULT_VEHICLE_CONFIG_PATH = PROJECT_ROOT / "config" / "vehicle.json"


@dataclass(frozen=True)
class VehicleConfig:
    ay_max_mps2: float
    ax_engine_max_mps2: float
    brake_max_mps2: float
    mu: float
    F_z_n: float
    v_max_mps: float
    curvature_epsilon: float
    v_min_mps: float
    power_limit_w: float | None = None
    path_optimization_reg_lambda: float = 1e-6
    path_optimization_b_max_iters: int = 8
    path_optimization_b_eps_m: float = 0.01
    path_optimization_b_alpha0: float = 1.0
    path_optimization_b_gtol: float = 1e-4
    path_optimization_b_ftol_s: float = 1e-3
    path_optimization_b_beta: float = 0.5
    path_optimization_b_n_jobs: int = 8
    path_optimization_c_apg_max_iters: int = 2000
    path_optimization_c_apg_tol: float = 1e-6

    def to_vehicle_dict(self) -> dict[str, int | float | None]:
        return {
            "ay_max_mps2": self.ay_max_mps2,
            "ax_engine_max_mps2": self.ax_engine_max_mps2,
            "brake_max_mps2": self.brake_max_mps2,
            "mu": self.mu,
            "F_z_n": self.F_z_n,
            "power_limit_w": self.power_limit_w,
            "v_max_mps": self.v_max_mps,
            "curvature_epsilon": self.curvature_epsilon,
            "v_min_mps": self.v_min_mps,
        }

    def to_optimization_dict(self) -> dict[str, int | float | None]:
        return {
            "path_optimization_reg_lambda": self.path_optimization_reg_lambda,
            "path_optimization_b_max_iters": self.path_optimization_b_max_iters,
            "path_optimization_b_eps_m": self.path_optimization_b_eps_m,
            "path_optimization_b_alpha0": self.path_optimization_b_alpha0,
            "path_optimization_b_gtol": self.path_optimization_b_gtol,
            "path_optimization_b_ftol_s": self.path_optimization_b_ftol_s,
            "path_optimization_b_beta": self.path_optimization_b_beta,
            "path_optimization_b_n_jobs": self.path_optimization_b_n_jobs,
            "path_optimization_c_apg_max_iters": self.path_optimization_c_apg_max_iters,
            "path_optimization_c_apg_tol": self.path_optimization_c_apg_tol,
        }

    def to_dict(self) -> dict[str, dict[str, int | float | None]]:
        return {
            "vehicle": self.to_vehicle_dict(),
            "optimization": self.to_optimization_dict(),
        }


def _read_config_section(values: dict[str, Any], section_name: str) -> dict[str, Any]:
    section = values.get(section_name)
    if section is None:
        return values
    if not isinstance(section, dict):
        raise ValueError(f"Config section '{section_name}' must be a JSON object.")
    return section


def load_vehicle_config(config_path: Path | None = None) -> VehicleConfig:
    path = config_path or DEFAULT_VEHICLE_CONFIG_PATH
    with path.open("r", encoding="utf-8") as handle:
        values = json.load(handle)

    if not isinstance(values, dict):
        raise ValueError("Vehicle config must be a JSON object.")

    vehicle_values = _read_config_section(values, "vehicle")
    optimization_values = _read_config_section(values, "optimization")

    return VehicleConfig(
        ay_max_mps2=float(vehicle_values["ay_max_mps2"]),
        ax_engine_max_mps2=float(vehicle_values.get("ax_engine_max_mps2", vehicle_values.get("ax_max_mps2"))),
        brake_max_mps2=float(vehicle_values["brake_max_mps2"]),
        mu=float(vehicle_values["mu"]),
        F_z_n=float(vehicle_values["F_z_n"]),
        v_max_mps=float(vehicle_values["v_max_mps"]),
        curvature_epsilon=float(vehicle_values["curvature_epsilon"]),
        v_min_mps=float(vehicle_values["v_min_mps"]),
        power_limit_w=None if vehicle_values.get("power_limit_w") is None else float(vehicle_values["power_limit_w"]),
        path_optimization_reg_lambda=float(optimization_values.get("path_optimization_reg_lambda", 1e-6)),
        path_optimization_b_max_iters=int(optimization_values.get("path_optimization_b_max_iters", 8)),
        path_optimization_b_eps_m=float(optimization_values.get("path_optimization_b_eps_m", 0.01)),
        path_optimization_b_alpha0=float(optimization_values.get("path_optimization_b_alpha0", 1.0)),
        path_optimization_b_gtol=float(optimization_values.get("path_optimization_b_gtol", 1e-4)),
        path_optimization_b_ftol_s=float(optimization_values.get("path_optimization_b_ftol_s", 1e-3)),
        path_optimization_b_beta=float(optimization_values.get("path_optimization_b_beta", 0.5)),
        path_optimization_b_n_jobs=int(optimization_values.get("path_optimization_b_n_jobs", 8)),
        path_optimization_c_apg_max_iters=int(optimization_values.get("path_optimization_c_apg_max_iters", 2000)),
        path_optimization_c_apg_tol=float(optimization_values.get("path_optimization_c_apg_tol", 1e-6)),
    )