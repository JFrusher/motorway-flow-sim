"""Road geometry and per-lane vehicle bookkeeping."""

from __future__ import annotations

import bisect

from .vehicle import Vehicle


def _pos(v: Vehicle) -> float:
    return v.x


class Road:
    """A straight multi-lane carriageway.

    Lane 0 is the slow (nearside) lane. Each lane keeps its vehicles sorted
    by ascending ``x`` so leaders/followers are list neighbours.
    """

    def __init__(self, length: float, num_lanes: int, speed_limit: float) -> None:
        if length <= 0 or num_lanes < 1 or speed_limit <= 0:
            raise ValueError("length, num_lanes and speed_limit must be positive")
        self.length = length
        self.num_lanes = num_lanes
        self.speed_limit = speed_limit
        self.lanes: list[list[Vehicle]] = [[] for _ in range(num_lanes)]
        self.obstacles: list[Vehicle] = []

    @property
    def vehicles(self) -> list[Vehicle]:
        """All vehicles on the road, lane by lane."""
        return [v for lane in self.lanes for v in lane]

    def __len__(self) -> int:
        return sum(len(lane) for lane in self.lanes)

    def _check_lane(self, lane: int) -> None:
        if not 0 <= lane < self.num_lanes:
            raise ValueError(f"lane {lane} out of range 0..{self.num_lanes - 1}")

    def insert(self, veh: Vehicle) -> None:
        """Insert a vehicle into its lane, keeping order."""
        self._check_lane(veh.lane)
        bisect.insort(self.lanes[veh.lane], veh, key=_pos)

    def place_obstacle(self, veh: Vehicle) -> None:
        """Insert a vehicle and register it as an obstacle (e.g. the lorry)."""
        self.insert(veh)
        self.obstacles.append(veh)

    def remove(self, veh: Vehicle) -> None:
        """Remove a vehicle from the road."""
        self.lanes[veh.lane].remove(veh)
        if veh in self.obstacles:
            self.obstacles.remove(veh)

    def change_lane(self, veh: Vehicle, new_lane: int) -> None:
        """Move a vehicle to ``new_lane`` at its current position."""
        self._check_lane(new_lane)
        self.lanes[veh.lane].remove(veh)
        veh.lane = new_lane
        bisect.insort(self.lanes[new_lane], veh, key=_pos)

    def resort(self) -> None:
        """Restore ordering after positions update (near-sorted, so O(n))."""
        for lane in self.lanes:
            lane.sort(key=_pos)

    def neighbours(self, x: float, lane: int):
        """Return ``(leader, follower)`` around position ``x`` in ``lane``.

        The leader is the nearest vehicle with front ahead of ``x``; the
        follower the nearest at or behind it. Either may be ``None``.
        """
        vehs = self.lanes[lane]
        i = bisect.bisect_right(vehs, x, key=_pos)
        leader = vehs[i] if i < len(vehs) else None
        follower = vehs[i - 1] if i > 0 else None
        return leader, follower

    def leader(self, veh: Vehicle) -> Vehicle | None:
        """Vehicle directly ahead in the same lane, or ``None``."""
        lane = self.lanes[veh.lane]
        i = lane.index(veh)
        return lane[i + 1] if i + 1 < len(lane) else None

    def follower(self, veh: Vehicle) -> Vehicle | None:
        """Vehicle directly behind in the same lane, or ``None``."""
        lane = self.lanes[veh.lane]
        i = lane.index(veh)
        return lane[i - 1] if i > 0 else None
