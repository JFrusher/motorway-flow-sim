"""Vehicle agent state, Intelligent Driver Model (IDM) and MOBIL lane changing.

The IDM functions are written with NumPy operations so they accept either
scalars or arrays; the simulation evaluates all vehicles in one call.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import DriverProfile

IDM_DELTA = 4.0


@dataclass(eq=False)  # identity equality: list.index/remove must find *this* vehicle
class Vehicle:
    """A single vehicle agent. ``x`` is the front-bumper position (m)."""

    id: int
    lane: int
    x: float
    v: float
    length: float
    profile: DriverProfile
    a: float = 0.0
    is_lorry: bool = False
    fixed: bool = False  # stationary obstacle: never accelerates or moves
    last_lane_change: float = -np.inf

    @property
    def rear(self) -> float:
        """Rear-bumper position (m)."""
        return self.x - self.length


def desired_gap(v, dv, s0, tau, a, b):
    """IDM desired gap s*(v, dv) = s0 + max(0, v*tau + v*dv / (2*sqrt(a*b)))."""
    return s0 + np.maximum(0.0, v * tau + v * dv / (2.0 * np.sqrt(a * b)))


def idm_acceleration(v, v0, a, b, tau, s0, gap, dv, delta: float = IDM_DELTA):
    """IDM acceleration.

    a_IDM = a * [1 - (v/v0)^delta - (s*(v, dv) / s)^2]

    Args:
        v: Own speed (m/s).
        v0: Desired speed (m/s), must be > 0.
        a, b: Max acceleration / comfortable deceleration (m/s^2).
        tau: Desired time headway (s).
        s0: Minimum gap (m).
        gap: Bumper-to-bumper gap to the leader (m); ``np.inf`` for free road.
        dv: Approach rate v - v_leader (m/s); ignored when gap is infinite.
    """
    s_star = desired_gap(v, dv, s0, tau, a, b)
    gap = np.maximum(gap, 1e-3)  # avoid division by zero; safety clamp handles s ~ 0
    return a * (1.0 - (v / v0) ** delta - (s_star / gap) ** 2)


def wake_adjusted(a, tau, strength, accel_gain: float, headway_factor: float):
    """Apply lorry slipstream effects.

    Reduced aerodynamic drag in the wake lets a follower accelerate harder and
    drivers accept a shorter headway while drafting. ``strength`` in [0, 1]
    scales the effect (1 = directly behind the lorry, 0 = no wake).
    """
    a_eff = a * (1.0 + accel_gain * strength)
    tau_eff = tau * (1.0 - (1.0 - headway_factor) * strength)
    return a_eff, tau_eff


def apply_safety_clamp(acc, gap, s0, b_max: float):
    """Hard collision prevention.

    Accelerations are limited to ``-b_max`` and, if the gap has fallen below
    ``0.5 * s0``, overridden to full emergency braking.
    """
    acc = np.maximum(acc, -b_max)
    return np.where(gap < 0.5 * s0, -b_max, acc)


def mobil_incentive(
    a_self_old: float,
    a_self_new: float,
    a_new_follower_old: float,
    a_new_follower_new: float,
    a_old_follower_old: float,
    a_old_follower_new: float,
    politeness: float,
) -> float:
    """MOBIL incentive (acceleration gain, m/s^2).

    gain = (a~_c - a_c) + p * [(a~_n - a_n) + (a~_o - a_o)]
    where c = changing vehicle, n = new follower, o = old follower.
    """
    return (a_self_new - a_self_old) + politeness * (
        (a_new_follower_new - a_new_follower_old) + (a_old_follower_new - a_old_follower_old)
    )


def mobil_is_safe(a_new_follower_new: float, a_self_new: float, b_safe: float) -> bool:
    """MOBIL safety criterion: no one is forced to brake harder than ``b_safe``."""
    return a_new_follower_new >= -b_safe and a_self_new >= -b_safe
