import numpy as np
import pandas as pd
import pytest

from motorway import Road, SimConfig, Simulation, load_config
from motorway.analytics import fundamental_diagram_data
from motorway.config import CAR_LENGTH, LORRY_PROFILE
from motorway.vehicle import Vehicle


def test_road_keeps_lanes_sorted_and_finds_neighbours():
    road = Road(1000, 3, 30)
    vs = [Vehicle(i, 1, x, 20, CAR_LENGTH, LORRY_PROFILE) for i, x in enumerate([300, 100, 200])]
    for v in vs:
        road.insert(v)
    assert [v.x for v in road.lanes[1]] == [100, 200, 300]
    leader, follower = road.neighbours(250, 1)
    assert (leader.x, follower.x) == (300, 200)
    road.change_lane(vs[2], 0)
    assert len(road.lanes[0]) == 1 and len(road) == 3
    with pytest.raises(ValueError):
        road.change_lane(vs[0], 3)


def test_config_validation():
    with pytest.raises(ValueError, match="dt"):
        SimConfig(dt=0.2)
    with pytest.raises(ValueError, match="unknown config keys"):
        load_config(dtt=0.1)
    with pytest.raises(ValueError, match="driver types"):
        SimConfig(profile_mix={"reckless": 1.0})


@pytest.fixture(scope="module")
def sixty_second_run():
    sim = Simulation(SimConfig(duration=60))
    return sim, sim.run()


def test_vehicle_count_conserved(sixty_second_run):
    sim, _ = sixty_second_run
    assert sim.inserted > 50
    assert sim.inserted - sim.exited == sim.vehicle_count


def test_no_nans_and_no_collisions(sixty_second_run):
    sim, df = sixty_second_run
    assert not df[["x", "v", "a"]].isna().any().any()
    assert (df.v >= 0).all()
    for lane in sim.road.lanes:
        for follower, leader in zip(lane, lane[1:]):
            assert leader.rear - follower.x > 0


def test_history_downsampled(sixty_second_run):
    sim, df = sixty_second_run
    times = np.sort(df.t.unique())
    assert np.diff(times) == pytest.approx(sim.cfg.record_interval)


def test_edie_fundamental_diagram_on_uniform_flow():
    # One lane, vehicles every 50 m at 20 m/s => k = 20 veh/km, q = 1440 veh/h.
    t = np.arange(0, 60, 0.5)
    rows = [
        (ti, i, 0, x0 + 20 * ti, 20.0, False)
        for i, x0 in enumerate(np.arange(-1200, 1000, 50.0))
        for ti in t
    ]
    df = pd.DataFrame(rows, columns=["t", "id", "lane", "x", "v", "is_lorry"])
    fd = fundamental_diagram_data(df, 0.5, 1000, 1, cell_length=250, cell_duration=30)
    assert fd.density.to_numpy() == pytest.approx(20, rel=0.05)
    assert fd.flow.to_numpy() == pytest.approx(1440, rel=0.05)
    assert fd.speed.to_numpy() == pytest.approx(72)
