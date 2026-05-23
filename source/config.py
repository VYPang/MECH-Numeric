from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

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

    def to_dict(self) -> dict[str, float | None]:
        return asdict(self)


def load_vehicle_config(config_path: Path | None = None) -> VehicleConfig:
    path = config_path or DEFAULT_VEHICLE_CONFIG_PATH
    with path.open("r", encoding="utf-8") as handle:
        values = json.load(handle)

    return VehicleConfig(
        ay_max_mps2=float(values["ay_max_mps2"]),
        ax_engine_max_mps2=float(values.get("ax_engine_max_mps2", values.get("ax_max_mps2"))),
        brake_max_mps2=float(values["brake_max_mps2"]),
        mu=float(values["mu"]),
        F_z_n=float(values["F_z_n"]),
        v_max_mps=float(values["v_max_mps"]),
        curvature_epsilon=float(values["curvature_epsilon"]),
        v_min_mps=float(values["v_min_mps"]),
        power_limit_w=None if values.get("power_limit_w") is None else float(values["power_limit_w"]),
        path_optimization_reg_lambda=float(values.get("path_optimization_reg_lambda", 1e-6)),
        path_optimization_b_max_iters=int(values.get("path_optimization_b_max_iters", 8)),
        path_optimization_b_eps_m=float(values.get("path_optimization_b_eps_m", 0.01)),
        path_optimization_b_alpha0=float(values.get("path_optimization_b_alpha0", 1.0)),
        path_optimization_b_gtol=float(values.get("path_optimization_b_gtol", 1e-4)),
        path_optimization_b_ftol_s=float(values.get("path_optimization_b_ftol_s", 1e-3)),
        path_optimization_b_beta=float(values.get("path_optimization_b_beta", 0.5)),
        path_optimization_b_n_jobs=int(values.get("path_optimization_b_n_jobs", 8)),
        path_optimization_c_apg_max_iters=int(values.get("path_optimization_c_apg_max_iters", 2000)),
        path_optimization_c_apg_tol=float(values.get("path_optimization_c_apg_tol", 1e-6)),
    )