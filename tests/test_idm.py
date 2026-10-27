import numpy as np
import pytest

from motorway import SimConfig, Simulation
from motorway.config import DriverProfile
from motorway.vehicle import apply_safety_clamp, idm_acceleration, wake_adjusted

P = DriverProfile(v0=30.0, a=1.4, b=2.0, tau=1.4, s0=2.0, politeness=0.3)


def empty_sim(**kw) -> Simulation:
    return Simulation(SimConfig(inflow=0, bottleneck_lorry_x=None, **kw))


def idm(v, gap, dv, p=P):
    return idm_acceleration(v, p.v0, p.a, p.b, p.tau, p.s0, gap, dv)


def test_idm_equilibria():
    assert idm(0.0, np.inf, 0.0) == pytest.approx(P.a)  # free start: full acceleration
    assert idm(P.v0, np.inf, 0.0) == pytest.approx(0.0)  # cruising at desired speed
    assert idm(0.0, P.s0, 0.0) == pytest.approx(0.0)  # queued at standstill gap
    assert idm(20.0, 30.0, 10.0) < -P.b  # closing fast on a near leader: hard braking


def test_idm_vectorised_matches_scalar():
    v, gap, dv = np.array([0.0, 15.0, 25.0]), np.array([np.inf, 40.0, 20.0]), np.array([0, 2, -1.0])
    vec = idm(v, gap, dv)
    assert vec == pytest.approx([idm(*args) for args in zip(v, gap, dv)])


def test_accelerates_cleanly_to_desired_speed_on_open_road():
    sim = empty_sim(road_length=1e5)
    car = sim.add_vehicle(0, 0.0, 0.0, profile=P)
    speeds = []
    for _ in range(1200):  # 120 s
        sim.step()
        speeds.append(car.v)
    assert np.all(np.diff(speeds) >= -1e-9), "speed should rise monotonically"
    assert np.max(np.diff(speeds)) / sim.cfg.dt <= P.a + 1e-9
    assert car.v == pytest.approx(P.v0, rel=0.02)


def test_brakes_smoothly_behind_stationary_vehicle():
    sim = empty_sim(num_lanes=1)  # single lane so the car cannot overtake
    obstacle = sim.add_vehicle(0, 600.0, 0.0, fixed=True)
    car = sim.add_vehicle(0, 300.0, 30.0, profile=P)
    decels, gaps = [], []
    for _ in range(900):  # 90 s
        sim.step()
        decels.append(car.a)
        gaps.append(obstacle.rear - car.x)
    assert min(gaps) > 0, "collision"
    assert min(decels) > -sim.cfg.b_max, "emergency clamp should not be needed"
    assert car.v == pytest.approx(0.0, abs=0.05)
    assert gaps[-1] == pytest.approx(P.s0, abs=0.5)


def test_safety_clamp_overrides_idm():
    acc = apply_safety_clamp(np.array([0.5, -20.0, 0.5]), np.array([0.5, 10.0, 10.0]), 2.0, 9.0)
    assert acc.tolist() == [-9.0, -9.0, 0.5]


def test_wake_adjustment():
    assert wake_adjusted(1.4, 1.4, 0.0, 0.15, 0.8) == (1.4, 1.4)
    a, tau = wake_adjusted(1.0, 1.0, 1.0, 0.15, 0.8)
    assert a == pytest.approx(1.15) and tau == pytest.approx(0.8)


def test_slipstream_behind_moving_lorry_only():
    sim = empty_sim()
    lorry = sim.add_vehicle(0, 500.0, 25.0, is_lorry=True)
    car = sim.add_vehicle(0, 500.0 - lorry.length - 20.0, 25.0, profile=P)
    drafting = sim._accelerations([car], [lorry])[0]
    lorry.is_lorry = False  # same geometry, no wake
    plain = sim._accelerations([car], [lorry])[0]
    assert drafting > plain

    lorry.is_lorry, lorry.v, car.v = True, 0.0, 0.0  # parked lorry: no airflow, no wake
    assert sim._accelerations([car], [lorry])[0] == pytest.approx(idm(0.0, lorry.rear - car.x, 0.0))
