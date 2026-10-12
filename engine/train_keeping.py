"""On-board formation keeping for a train of drones (the figure-8 plan),
or a formation flying timed tracks (the star plan).

A train is a line of drones flying one shared path, one spacing apart. The
ground station (`services.thread_backend`) runs a single PACE along that
path - where the front of the train should be, and how fast that point is
moving - and broadcasts it, time-stamped, to every drone. Each drone's own
thread (its companion computer) then does the formation keeping itself,
every tick:

    where should I be at the next tick?   pace, extrapolated from its time
                                          stamp, minus my place in the train
    where am I?                           my own position, projected onto my
                                          own route (`RouteTracker`)
    -> the speed that closes the gap      (`train_speed`), sent straight to
                                          my own autopilot

Doing this on board, rather than in the ground station, is what makes it
hold: the drone knows its own position exactly and acts on it the same
tick, while a ground-station loop sees every drone through telemetry and
commands it over the network, both a poll or two late - at the
simulator's time scale that is many metres of flight, and a train of
drones each reacting late to the one in front oscillates.

A formation that changes shape (the star plan) uses the same machinery
with a SCHEDULE instead of a fixed place in a line: each drone is given
keyframes "at pace time t, be s metres along your own route", and the pace
is a shared clock (in nominal seconds) rather than a distance. Every drone
flying its own schedule against the one clock is what makes every shape
change start and finish together.

The neighbours still matter. The ground station includes the drone in
front's last known position in each drone's pace message - a drone that
seems too close to it does not speed up to catch up into it - and slows the
pace itself when a drone falls behind its place (so the train waits for a
straggler rather than stretching). The front report is deliberately only a
cap, never a hard brake: it is a poll or two old, and a drone slamming on
the brakes over stale news is exactly how the one behind runs into it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

METRES_PER_DEG_LAT = 111_320.0
# Design speed of timed tracks (schedule mode): one unit of the pace clock
# is this many metres at full speed. The ground station runs the clock
# faster or slower to suit the actual swarm.
SCHEDULE_UNIT_MPS = 10.0
# Seconds (simulated) a drone takes to close most of the gap to its place.
# A fixed time constant, not "close it by the next tick": drone threads
# don't tick on a precise clock, and a speed sized for one short tick but
# flown for a long one overshoots.
CATCH_UP_S = 2.5
# How far ahead along its route a drone is looked for each time - so where
# the route passes close to itself (the 8's crossing, the next lap, the trip
# back over the same ground) the projection can't jump to the wrong part.
SEARCH_SEGMENTS = 12
# Don't speed up past the pace if the drone in front is reported closer than
# this many spacings ahead.
FRONT_CAUTION_SPACINGS = 0.7


def local_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """(east, north) metres of a point from an origin - locally flat."""
    north = (lat - origin_lat) * METRES_PER_DEG_LAT
    east = (lon - origin_lon) * METRES_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    return east, north


class RouteTracker:
    """A route written out in full, and how far along it a drone has got."""

    # A drone's progress never reads more than this behind the furthest it
    # has got - which is what tells "flying back along the same line" (a
    # show's star -> circle after circle -> star) apart from "still flying
    # out along it": both are equally close to the drone.
    BACKSTEP_M = 2.0
    # Candidates this close (in metres) to the nearest one count as a tie,
    # broken in favour of the one nearest the drone's last progress.
    TIE_M = 0.75

    def __init__(self, points: list[tuple[float, float]]):
        self.points = points
        self.cumulative_m = [0.0]
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            self.cumulative_m.append(self.cumulative_m[-1] + math.hypot(bx - ax, by - ay))
        self.segment = 0
        self.last_m = 0.0
        self.furthest_m = 0.0

    @property
    def length_m(self) -> float:
        return self.cumulative_m[-1]

    def progress_m(self, xy: tuple[float, float]) -> float:
        """Metres flown along the route, for a drone at `xy`."""
        last = len(self.points) - 2
        if last < 0:
            return 0.0
        candidates = []
        for step in range(-1, SEARCH_SEGMENTS):
            seg = self.segment + step
            if seg < 0 or seg > last:
                continue
            (ax, ay), (bx, by) = self.points[seg], self.points[seg + 1]
            dx, dy = bx - ax, by - ay
            length_sq = dx * dx + dy * dy
            t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((xy[0] - ax) * dx + (xy[1] - ay) * dy) / length_sq))
            along = self.cumulative_m[seg] + t * (self.cumulative_m[seg + 1] - self.cumulative_m[seg])
            if along < self.furthest_m - self.BACKSTEP_M:
                continue
            dist = math.hypot(ax + t * dx - xy[0], ay + t * dy - xy[1])
            candidates.append((dist, abs(along - self.last_m), seg, along))
        if not candidates:
            return self.last_m
        nearest = min(c[0] for c in candidates)
        _dist, _gap, seg, along = min((c for c in candidates if c[0] <= nearest + self.TIE_M), key=lambda c: c[1])
        self.segment = seg
        self.last_m = along
        self.furthest_m = max(self.furthest_m, along)
        return along


@dataclass(frozen=True)
class TrainSlot:
    """A drone's place in the train. Positions on the SHARED path are its
    own route progress minus `shared_from_m` (its private way from its pad
    to the base); it belongs `slot_m` behind the front of the train."""

    shared_from_m: float
    slot_m: float
    spacing_m: float
    max_mps: float
    min_mps: float = 0.1
    # Keyframes ((pace_time, route_metres), ...) - when given, the drone
    # follows this timetable instead of a place in a line (see module doc).
    schedule: tuple[tuple[float, float], ...] = ()


def schedule_at(schedule, t: float) -> tuple[float, float]:
    """(route metres, metres per unit of pace time) a timetable asks for
    at pace time `t` - linear between keyframes, holding still outside them."""
    if t <= schedule[0][0]:
        return schedule[0][1], 0.0
    for (t0, s0), (t1, s1) in zip(schedule, schedule[1:]):
        if t <= t1:
            if t1 <= t0:
                return s1, 0.0
            slope = (s1 - s0) / (t1 - t0)
            return s0 + slope * (t - t0), slope
    return schedule[-1][1], 0.0


@dataclass(frozen=True)
class Pace:
    """The ground station's pace: the front of the train is `at_m` along
    the shared path at wall time `stamp_s` (time.time()), moving at `mps`.
    `front_at_m` is where the drone directly in front of the receiver was
    at that moment, if it is flying."""

    at_m: float
    mps: float
    stamp_s: float
    front_at_m: Optional[float] = None


def train_speed(
    slot: TrainSlot,
    pace: Pace,
    at_m: float,
    end_m: float,
    now_s: float,
    time_scale: float,
    tick_sim_s: float = 0.0,
) -> float:
    """The speed (m/s) to fly to stay in place: the pace's own speed, plus
    whatever closes the gap to this drone's place over `CATCH_UP_S`.

    `at_m` is where the drone is on the shared path and `end_m` where its
    own route ends there (it can't be asked to go past its landing pad)."""
    since_s = max(0.0, now_s - pace.stamp_s) * time_scale
    if slot.schedule:
        clock = pace.at_m + pace.mps * since_s
        target, slope = schedule_at(slot.schedule, clock)
        target = min(target, end_m)
        speed = slope * pace.mps + (target - at_m) / CATCH_UP_S
        if tick_sim_s > 0:
            # Never plan to be past where the timetable wants this drone one
            # tick from now: the autopilot keeps flying at this speed until
            # the next tick, so a move that ends mid-tick would otherwise
            # carry it metres past its place - and a timetable drone holding
            # a shape can't fly back.
            ahead = min(schedule_at(slot.schedule, clock + pace.mps * tick_sim_s)[0], end_m)
            speed = min(speed, (ahead - at_m) / tick_sim_s)
        return min(slot.max_mps, max(slot.min_mps, speed))
    target = min(pace.at_m + pace.mps * since_s - slot.slot_m, end_m)
    speed = pace.mps + (target - at_m) / CATCH_UP_S
    if pace.front_at_m is not None and 0.0 <= at_m <= end_m:
        front_now = pace.front_at_m + pace.mps * since_s
        if front_now - at_m < FRONT_CAUTION_SPACINGS * slot.spacing_m:
            speed = min(speed, pace.mps)
    return min(slot.max_mps, max(slot.min_mps, speed))
