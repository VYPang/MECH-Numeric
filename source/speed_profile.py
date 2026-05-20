from __future__ import annotations

from dataclasses import dataclass
import math

from .config import VehicleConfig
from .curvature import CurvatureProfile, closed_segment_lengths, cumulative_values


@dataclass(frozen=True)
class IntegrationComparison:
    kinematic_time_s: float
    left_rule_time_s: float
    trapezoidal_time_s: float
    simpson_time_s: float | None


@dataclass(frozen=True)
class ConstraintResiduals:
    lateral_mps2: float
    acceleration_mps2: float
    braking_mps2: float
    friction_circle_mps2: float


@dataclass(frozen=True)
class SpeedProfileResult:
    geometry_method: str
    x_path_m: list[float]
    y_path_m: list[float]
    s_nodes_m: list[float]
    s_midpoints_m: list[float]
    segment_lengths_m: list[float]
    curvature: list[float]
    speed_cap_mps: list[float]
    friction_total_accel_mps2: float
    forward_longitudinal_limit_mps2: list[float]
    braking_longitudinal_limit_mps2: list[float]
    forward_speed_mps: list[float]
    final_speed_mps: list[float]
    longitudinal_accel_mps2: list[float]
    lateral_accel_mps2: list[float]
    integration: IntegrationComparison
    residuals: ConstraintResiduals

    @property
    def total_length_m(self) -> float:
        return self.s_nodes_m[-1]


def compute_speed_profile(profile: CurvatureProfile, config: VehicleConfig) -> SpeedProfileResult:
    segment_lengths = _segment_lengths_for_profile(profile)
    s_nodes = cumulative_values(segment_lengths)
    s_midpoints = [0.5 * (s_nodes[index] + s_nodes[index + 1]) for index in range(len(segment_lengths))]
    friction_total_accel = compute_total_friction_accel_limit(config)
    speed_cap = compute_lateral_speed_cap(profile.curvature, config, friction_total_accel)
    speed_cap_nodes = speed_cap + [speed_cap[0]]
    forward_speed, forward_limits = _forward_pass(speed_cap_nodes, profile.curvature, segment_lengths, config, friction_total_accel)
    final_speed, braking_limits = _backward_pass(forward_speed, profile.curvature, segment_lengths, config, friction_total_accel)
    longitudinal_accel = _compute_longitudinal_acceleration(final_speed, segment_lengths)
    lateral_accel = [
        final_speed[index] ** 2 * abs(profile.curvature[index % len(profile.curvature)])
        for index in range(len(final_speed))
    ]
    integration = compare_lap_time_integrators(final_speed, segment_lengths, config)
    residuals = compute_constraint_residuals(final_speed, longitudinal_accel, profile.curvature, segment_lengths, config, friction_total_accel)

    return SpeedProfileResult(
        geometry_method=profile.method,
        x_path_m=profile.x,
        y_path_m=profile.y,
        s_nodes_m=s_nodes,
        s_midpoints_m=s_midpoints,
        segment_lengths_m=segment_lengths,
        curvature=profile.curvature,
        speed_cap_mps=speed_cap_nodes,
        friction_total_accel_mps2=friction_total_accel,
        forward_longitudinal_limit_mps2=forward_limits,
        braking_longitudinal_limit_mps2=braking_limits,
        forward_speed_mps=forward_speed,
        final_speed_mps=final_speed,
        longitudinal_accel_mps2=longitudinal_accel,
        lateral_accel_mps2=lateral_accel,
        integration=integration,
        residuals=residuals,
    )


def _segment_lengths_for_profile(profile: CurvatureProfile) -> list[float]:
    if "resampled" in profile.method:
        return [profile.total_length_m / len(profile.curvature) for _ in profile.curvature]
    return closed_segment_lengths(profile.x, profile.y)


def compute_total_friction_accel_limit(config: VehicleConfig) -> float:
    vehicle_mass_kg = compute_vehicle_mass_kg(config)
    friction_force_n = config.mu * config.F_z_n
    return friction_force_n / vehicle_mass_kg


def compute_vehicle_mass_kg(config: VehicleConfig) -> float:
    gravitational_accel = 9.81
    return config.F_z_n / gravitational_accel


def compute_lateral_speed_cap(curvature: list[float], config: VehicleConfig, friction_total_accel: float) -> list[float]:
    lateral_cap = min(config.ay_max_mps2, friction_total_accel)
    return [
        min(config.v_max_mps, math.sqrt(lateral_cap / (abs(value) + config.curvature_epsilon)))
        for value in curvature
    ]


def compute_power_limited_accel(speed_mps: float, config: VehicleConfig) -> float:
    if config.power_limit_w is None:
        return math.inf
    vehicle_mass_kg = compute_vehicle_mass_kg(config)
    effective_speed = max(speed_mps, config.v_min_mps)
    return config.power_limit_w / (vehicle_mass_kg * effective_speed)


def compute_drive_accel_limit(speed_mps: float, config: VehicleConfig) -> float:
    return min(config.ax_engine_max_mps2, compute_power_limited_accel(speed_mps, config))


def compute_available_forward_longitudinal_limit(
    speed_mps: float,
    curvature_abs: float,
    config: VehicleConfig,
    friction_total_accel: float,
) -> float:
    lateral_accel = speed_mps**2 * curvature_abs
    friction_limit = compute_friction_limited_longitudinal_accel(lateral_accel, friction_total_accel)
    return min(compute_drive_accel_limit(speed_mps, config), friction_limit)


def compute_available_braking_limit(
    speed_mps: float,
    curvature_abs: float,
    config: VehicleConfig,
    friction_total_accel: float,
) -> float:
    lateral_accel = speed_mps**2 * curvature_abs
    friction_limit = compute_friction_limited_longitudinal_accel(lateral_accel, friction_total_accel)
    return min(config.brake_max_mps2, friction_limit)


def _forward_pass(
    speed_cap_nodes: list[float],
    curvature: list[float],
    segment_lengths: list[float],
    config: VehicleConfig,
    friction_total_accel: float,
) -> tuple[list[float], list[float]]:
    speeds = [0.0 for _ in speed_cap_nodes]
    accel_limits: list[float] = []
    for index, length in enumerate(segment_lengths):
        upper_speed = speed_cap_nodes[index + 1]
        speeds[index + 1] = _solve_forward_speed_with_friction_circle(
            speeds[index],
            upper_speed,
            length,
            abs(curvature[index % len(curvature)]),
            config,
            friction_total_accel,
        )
        accel_limits.append(
            compute_available_forward_longitudinal_limit(
                speeds[index + 1],
                abs(curvature[index % len(curvature)]),
                config,
                friction_total_accel,
            )
            if not math.isclose(length, 0.0)
            else 0.0
        )
    return speeds, accel_limits


def _backward_pass(
    forward_speed: list[float],
    curvature: list[float],
    segment_lengths: list[float],
    config: VehicleConfig,
    friction_total_accel: float,
) -> tuple[list[float], list[float]]:
    speeds = list(forward_speed)
    brake_limits = [0.0 for _ in segment_lengths]
    for index in range(len(segment_lengths) - 1, -1, -1):
        brake_limited_speed = math.sqrt(speeds[index + 1] ** 2 + 2.0 * config.brake_max_mps2 * segment_lengths[index])
        upper_speed = min(speeds[index], brake_limited_speed)
        speeds[index] = _solve_backward_speed_with_friction_circle(
            speeds[index + 1],
            upper_speed,
            segment_lengths[index],
            abs(curvature[index % len(curvature)]),
            config.brake_max_mps2,
            friction_total_accel,
        )
        brake_limits[index] = (
            compute_available_braking_limit(
                speeds[index],
                abs(curvature[index % len(curvature)]),
                config,
                friction_total_accel,
            )
            if not math.isclose(segment_lengths[index], 0.0)
            else 0.0
        )
    speeds[0] = 0.0
    return speeds, brake_limits


def compute_friction_limited_longitudinal_accel(lateral_accel: float, friction_total_accel: float) -> float:
    clamped_lateral = min(abs(lateral_accel), friction_total_accel)
    return math.sqrt(max(friction_total_accel**2 - clamped_lateral**2, 0.0))


def _solve_forward_speed_with_friction_circle(
    speed_start: float,
    speed_upper: float,
    segment_length: float,
    curvature_abs: float,
    config: VehicleConfig,
    friction_total_accel: float,
) -> float:
    if math.isclose(segment_length, 0.0):
        return speed_start
    if speed_upper <= speed_start:
        return speed_upper
    lower = speed_start
    upper = max(speed_start, speed_upper)
    for _ in range(40):
        trial = 0.5 * (lower + upper)
        longitudinal_accel = (trial**2 - speed_start**2) / (2.0 * segment_length)
        lateral_accel = trial**2 * curvature_abs
        accel_limit = compute_drive_accel_limit(trial, config)
        within_engine = longitudinal_accel <= accel_limit + 1e-12
        within_friction = math.hypot(longitudinal_accel, lateral_accel) <= friction_total_accel + 1e-12
        if within_engine and within_friction:
            lower = trial
        else:
            upper = trial
    return lower


def _solve_backward_speed_with_friction_circle(
    speed_end: float,
    speed_upper: float,
    segment_length: float,
    curvature_abs: float,
    brake_limit: float,
    friction_total_accel: float,
) -> float:
    if math.isclose(segment_length, 0.0):
        return speed_end
    lower = 0.0
    upper = max(0.0, speed_upper)
    for _ in range(40):
        trial = 0.5 * (lower + upper)
        braking_accel = max((trial**2 - speed_end**2) / (2.0 * segment_length), 0.0)
        lateral_accel = trial**2 * curvature_abs
        within_brake = braking_accel <= brake_limit + 1e-12
        within_friction = math.hypot(braking_accel, lateral_accel) <= friction_total_accel + 1e-12
        if within_brake and within_friction:
            lower = trial
        else:
            upper = trial
    return lower


def _compute_longitudinal_acceleration(speed_nodes: list[float], segment_lengths: list[float]) -> list[float]:
    accelerations: list[float] = []
    for index, length in enumerate(segment_lengths):
        if math.isclose(length, 0.0):
            accelerations.append(0.0)
        else:
            accelerations.append((speed_nodes[index + 1] ** 2 - speed_nodes[index] ** 2) / (2.0 * length))
    return accelerations


def compare_lap_time_integrators(speed_nodes: list[float], segment_lengths: list[float], config: VehicleConfig) -> IntegrationComparison:
    left_time = 0.0
    trapezoidal_time = 0.0
    kinematic_time = 0.0
    inverse_speed_nodes: list[float] = []

    for speed in speed_nodes:
        inverse_speed_nodes.append(1.0 / max(speed, config.v_min_mps))

    for index, length in enumerate(segment_lengths):
        left_time += length / max(speed_nodes[index], config.v_min_mps)
        trapezoidal_time += 0.5 * length * (inverse_speed_nodes[index] + inverse_speed_nodes[index + 1])
        speed_sum = speed_nodes[index] + speed_nodes[index + 1]
        if math.isclose(speed_sum, 0.0):
            continue
        kinematic_time += 2.0 * length / speed_sum

    simpson_time = _composite_simpson_time(inverse_speed_nodes, segment_lengths)
    return IntegrationComparison(
        kinematic_time_s=kinematic_time,
        left_rule_time_s=left_time,
        trapezoidal_time_s=trapezoidal_time,
        simpson_time_s=simpson_time,
    )


def _composite_simpson_time(inverse_speed_nodes: list[float], segment_lengths: list[float]) -> float | None:
    panel_count = len(segment_lengths)
    if panel_count % 2 != 0 or panel_count < 2:
        return None
    mean_ds = sum(segment_lengths) / panel_count
    max_deviation = max(abs(length - mean_ds) for length in segment_lengths)
    if max_deviation / mean_ds > 1e-3:
        return None

    odd_sum = sum(inverse_speed_nodes[index] for index in range(1, panel_count, 2))
    even_sum = sum(inverse_speed_nodes[index] for index in range(2, panel_count, 2))
    return mean_ds * (inverse_speed_nodes[0] + inverse_speed_nodes[-1] + 4.0 * odd_sum + 2.0 * even_sum) / 3.0


def compute_constraint_residuals(
    speed_nodes: list[float],
    longitudinal_accel: list[float],
    curvature: list[float],
    segment_lengths: list[float],
    config: VehicleConfig,
    friction_total_accel: float,
) -> ConstraintResiduals:
    lateral_residual = max(
        speed_nodes[index] ** 2 * abs(curvature[index % len(curvature)]) - config.ay_max_mps2
        for index in range(len(speed_nodes))
    )
    acceleration_residual = max(
        longitudinal_accel[index]
        - compute_available_forward_longitudinal_limit(
            speed_nodes[index + 1],
            abs(curvature[index % len(curvature)]),
            config,
            friction_total_accel,
        )
        for index in range(len(segment_lengths))
    )
    braking_residual = max(
        -longitudinal_accel[index]
        - compute_available_braking_limit(
            speed_nodes[index],
            abs(curvature[index % len(curvature)]),
            config,
            friction_total_accel,
        )
        for index in range(len(segment_lengths))
    )
    friction_circle_residual = max(
        math.hypot(abs(longitudinal_accel[index]), speed_nodes[index] ** 2 * abs(curvature[index % len(curvature)])) - friction_total_accel
        for index in range(len(segment_lengths))
    )
    return ConstraintResiduals(
        lateral_mps2=lateral_residual,
        acceleration_mps2=acceleration_residual,
        braking_mps2=braking_residual,
        friction_circle_mps2=friction_circle_residual,
    )