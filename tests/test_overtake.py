import pandas as pd

from motorway import SimConfig
from motorway.overtake import ElephantRace, passing_flow, queue_behind, run_race


def cfg(**kw) -> SimConfig:
    base = dict(road_length=4000, duration=120, bottleneck_lorry_x=None)
    return SimConfig(**{**base, **kw})


def test_lorry_pulls_out_passes_and_returns():
    race = ElephantRace(fast_mph=56, slow_mph=40, start_x=200, overtake_at=1)
    run = run_race(cfg(inflow=0), race)
    assert race.state == "done" and race.t_out is not None and race.t_back is not None
    assert race.fast.lane == 0 and race.fast.rear > race.slow.x
    assert run.history[run.history.id == race.fast.id].lane.max() == 1


def test_baseline_keeps_lorries_in_lane_one():
    race = ElephantRace(overtake=False, overtake_at=1)
    run = run_race(cfg(inflow=0, duration=30), race)
    assert run.history[run.history.id.isin(race.lorry_ids)].lane.max() == 0
    assert race.blocking_time(30) == 0


def test_same_seed_gives_same_demand_with_and_without_overtake():
    base = run_race(cfg(inflow=4200), ElephantRace(overtake=False, overtake_at=20))
    over = run_race(cfg(inflow=4200), ElephantRace(overtake=True, overtake_at=20))
    assert over.race.t_out is not None
    assert base.arrivals == over.arrivals


def test_passing_flow_and_queue_on_synthetic_trajectories():
    # Lorry (id 0) at 20 m/s; car 1 overtakes it at t = 10 s; car 2 sits 30 m behind at 20 m/s.
    rows = []
    for t in range(0, 20):
        rows += [
            (t, 0, 0, 1000 + 20 * t, 20.0, True),
            (t, 1, 1, 800 + 40 * t, 40.0, False),
            (t, 2, 0, 1000 + 20 * t - 30, 20.0, False),
        ]
    df = pd.DataFrame(rows, columns=["t", "id", "lane", "x", "v", "is_lorry"])
    flow = passing_flow(df, [0], bin_s=20)
    assert flow.sum() == 3600 / 20  # exactly one pass in one 20 s bin
    q = queue_behind(df, [0], hold_speed=21.0)
    assert (q.vehicles == 1).all() and (q.length_m == 30).all()
