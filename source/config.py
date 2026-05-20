from __future__ import annotations

import json
from dataclasses import dataclass
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
    )