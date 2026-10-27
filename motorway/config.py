"""Simulation parameters, driver profiles and YAML config loading."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np
import yaml

CAR_LENGTH = 4.5
LORRY_LENGTH = 16.5


@dataclass(frozen=True)
class DriverProfile:
    """IDM/MOBIL behavioural parameters for one driver.

    Attributes:
        v0: Desired speed (m/s).
        a: Maximum acceleration (m/s^2).
        b: Comfortable deceleration (m/s^2).
        tau: Desired time headway (s).
        s0: Minimum standstill gap (m).
        politeness: MOBIL politeness factor p in [0, 1].
    """

    v0: float
    a: float
    b: float
    tau: float
    s0: float
    politeness: float


# Desired speed is expressed as a multiple of the speed limit so profiles
# scale with the road. Lorry speed is absolute (UK HGV limiter ~56 mph).
DRIVER_TYPES: dict[str, dict[str, float]] = {
    "aggressive": dict(speed_factor=1.15, a=2.0, b=2.5, tau=0.9, s0=1.5, politeness=0.1),
    "normal": dict(speed_factor=1.00, a=1.4, b=2.0, tau=1.4, s0=2.0, politeness=0.3),
    "cautious": dict(speed_factor=0.90, a=1.0, b=1.5, tau=1.8, s0=2.5, politeness=0.5),
}
LORRY_PROFILE = DriverProfile(v0=25.0, a=0.6, b=1.5, tau=1.8, s0=3.0, politeness=0.5)


def make_profile(
    kind: str, speed_limit: float, rng: np.random.Generator, speed_spread: float = 0.05
) -> DriverProfile:
    """Sample a concrete driver profile of the given type.

    Desired speed gets multiplicative Gaussian noise (``speed_spread`` std)
    so that drivers of the same type are not identical.
    """
    if kind not in DRIVER_TYPES:
        raise ValueError(f"unknown driver type {kind!r}; choose from {list(DRIVER_TYPES)}")
    spec = DRIVER_TYPES[kind]
    noise = max(0.5, 1.0 + speed_spread * rng.standard_normal()) if speed_spread else 1.0
    return DriverProfile(
        v0=spec["speed_factor"] * speed_limit * noise,
        a=spec["a"],
        b=spec["b"],
        tau=spec["tau"],
        s0=spec["s0"],
        politeness=spec["politeness"],
    )


@dataclass
class SimConfig:
    """All tunable simulation parameters (SI units unless noted)."""

    # Time integration
    dt: float = 0.1
    duration: float = 480.0
    record_interval: float = 0.5
    seed: int = 42

    # Road
    road_length: float = 5000.0
    num_lanes: int = 3
    speed_limit: float = 31.3  # 70 mph

    # Demand
    inflow: float = 4800.0  # veh/h, all lanes combined
    profile_mix: dict[str, float] = field(
        default_factory=lambda: {"aggressive": 0.2, "normal": 0.6, "cautious": 0.2}
    )
    lorry_fraction: float = 0.0  # share of slow-lane arrivals that are lorries

    # Bottleneck lorry (set bottleneck_lorry_x to null to disable).
    # Default: broken-down lorry blocking the slow lane at 3 km.
    bottleneck_lorry_x: float | None = 3000.0
    bottleneck_lorry_speed: float = 0.0  # 0 = stationary obstacle, > 0 = moving bottleneck
    bottleneck_lorry_lane: int = 0

    # Aerodynamic wake (slipstream) behind lorries
    wake_length: float = 40.0
    wake_accel_gain: float = 0.15  # fractional boost to max acceleration
    wake_headway_factor: float = 0.8  # multiplier on desired time headway

    # MOBIL lane changing
    lane_change_threshold: float = 0.2  # m/s^2
    keep_left_bias: float = 0.3  # m/s^2, UK keep-left rule
    b_safe: float = 4.0  # m/s^2
    lane_change_cooldown: float = 3.0  # s
    lane_change_interval: float = 1.0  # s between MOBIL evaluations per driver
    obstruction_lookahead: float = 500.0  # m, distance at which drivers react to a blocked lane
    obstruction_bias: float = 3.0  # m/s^2, incentive to leave (and not enter) a blocked lane

    # Safety
    b_max: float = 9.0  # emergency braking, m/s^2

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Raise ``ValueError`` on physically or numerically invalid settings."""
        if not 0 < self.dt <= 0.1:
            raise ValueError("dt must be in (0, 0.1] s for IDM numerical stability")
        if self.record_interval < self.dt:
            raise ValueError("record_interval must be >= dt")
        if self.duration <= 0 or self.road_length <= 0 or self.speed_limit <= 0:
            raise ValueError("duration, road_length and speed_limit must be positive")
        if self.num_lanes < 1:
            raise ValueError("num_lanes must be >= 1")
        if self.inflow < 0:
            raise ValueError("inflow must be >= 0")
        if not 0 <= self.lorry_fraction <= 1:
            raise ValueError("lorry_fraction must be in [0, 1]")
        unknown = set(self.profile_mix) - set(DRIVER_TYPES)
        if unknown:
            raise ValueError(f"unknown driver types in profile_mix: {sorted(unknown)}")
        if any(w < 0 for w in self.profile_mix.values()) or sum(self.profile_mix.values()) <= 0:
            raise ValueError("profile_mix weights must be non-negative with positive sum")
        if self.bottleneck_lorry_x is not None:
            if not 0 <= self.bottleneck_lorry_x <= self.road_length:
                raise ValueError("bottleneck_lorry_x must lie on the road")
            if not 0 <= self.bottleneck_lorry_lane < self.num_lanes:
                raise ValueError("bottleneck_lorry_lane out of range")
            if self.bottleneck_lorry_speed < 0:
                raise ValueError("bottleneck_lorry_speed must be >= 0")
        if self.b_max <= 0 or self.b_safe <= 0:
            raise ValueError("b_max and b_safe must be positive")


def load_config(path: str | Path | None = None, **overrides) -> SimConfig:
    """Build a :class:`SimConfig` from an optional YAML file plus keyword overrides.

    Unknown keys raise ``ValueError`` so typos are not silently ignored.
    """
    data: dict = {}
    if path is not None:
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path}: top level must be a mapping")
    data.update({k: v for k, v in overrides.items() if v is not None})
    known = {f.name for f in fields(SimConfig)}
    unknown = set(data) - known
    if unknown:
        raise ValueError(f"unknown config keys: {sorted(unknown)}")
    return SimConfig(**data)
