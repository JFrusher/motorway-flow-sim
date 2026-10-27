import pytest

from motorway import SimConfig, Simulation
from motorway.vehicle import mobil_incentive, mobil_is_safe


def empty_sim() -> Simulation:
    return Simulation(SimConfig(inflow=0, bottleneck_lorry_x=None))


def test_incentive_formula():
    # own gain 1.0, new follower loses 2.0, old follower gains 0.5, p = 0.5
    assert mobil_incentive(0.0, 1.0, 0.0, -2.0, 0.0, 0.5, 0.5) == pytest.approx(0.25)


def test_safety_criterion():
    assert mobil_is_safe(-3.9, 0.0, 4.0)
    assert not mobil_is_safe(-4.1, 0.0, 4.0)
    assert not mobil_is_safe(0.0, -4.1, 4.0)


def blocked_car_scenario(with_closing_car: bool):
    """Car stuck behind a slow leader in lane 0; optional fast car closing in lane 1."""
    sim = empty_sim()
    sim.add_vehicle(0, 530.0, 10.0)  # slow leader
    car = sim.add_vehicle(0, 500.0, 20.0)
    if with_closing_car:
        sim.add_vehicle(1, 480.0, 35.0)  # 15 m behind car's rear, closing at 15 m/s
    sim.update_accelerations()
    return sim, car


def test_accepts_overtake_into_clear_lane():
    sim, car = blocked_car_scenario(with_closing_car=False)
    assert sim.evaluate_lane_change(car, 1) is not None


def test_refuses_unsafe_change_when_follower_closing_fast():
    sim, car = blocked_car_scenario(with_closing_car=True)
    assert sim.evaluate_lane_change(car, 1) is None


def test_refuses_overlap_and_invalid_lanes():
    sim = empty_sim()
    car = sim.add_vehicle(0, 500.0, 20.0)
    sim.add_vehicle(1, 502.0, 20.0)  # alongside
    sim.update_accelerations()
    assert sim.evaluate_lane_change(car, 1) is None
    assert sim.evaluate_lane_change(car, -1) is None


def test_keep_left_rule_returns_car_to_slow_lane():
    sim = empty_sim()
    car = sim.add_vehicle(1, 100.0, 30.0)
    for _ in range(20):
        sim.step()
    assert car.lane == 0


def test_lorries_never_change_lane():
    sim = empty_sim()
    sim.add_vehicle(0, 530.0, 5.0)
    lorry = sim.add_vehicle(0, 500.0, 20.0, is_lorry=True)
    sim.update_accelerations()
    assert sim.evaluate_lane_change(lorry, 1) is None
