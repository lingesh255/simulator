# Multi-drone Renode fleets

Fly one emulated Pixhawk 6C (ArduCopter in Renode) per drone, each with its
own MAVLink connection and mission, from the GUI.

## Flying a fleet

1. In **Drone Management**, check two or more drone profiles. Each needs its
   own SYSID between 1 and 32: the SYSID is also that drone's Renode
   instance number.
2. In **Mission Planner**, check **Fly via real MAVLink** and leave
   **Connect** empty (empty means Renode is the vehicle).
3. Pick a plan, click **Plan Mission**, then click the points on the map as
   usual.

With exactly one drone checked, the single-drone path is used unchanged
(Renode instance 0 on `tcp:127.0.0.1:5762`). With a mock vehicle or your own
address typed into Connect, no Renode is launched.

What happens with two or more drones (`services/fleet_mission.py`):

- **Boot**: `kill_all_renode_processes()` runs once, then every instance
  boots at the same time, each spawned at its own route's start. The
  missions start only once every drone is armable (about 150-200 s). If
  any drone fails to boot, the others are stopped, one fleet failure is
  reported and no mission starts.
- **Routes**: plans with per-drone routes (Search lanes, V-formation and
  grid slots) use them. A plan with one shared route (travell) is flown by
  every drone, each copy shifted 18 m further east than the last.
- **Separation**: if a drone's spawn or landing point is closer than **5 m**
  to an earlier drone's, it is moved east in 18 m steps (only that end
  point; the rest of the route is unchanged) and the move is logged. This
  catches shared endpoints (the Search base, a shared destination; ~0 m
  apart) and leaves V and grid slots at their designed spacing (20 m and
  10 m). 5 m because the project spec's formation row requires at least 2 m
  between drones, and 5 m leaves margin for the 0-2.4 m landing error seen
  in testing.
- **Flight**: one `MavlinkFlightService` per drone; their telemetry reaches
  the map as one `SwarmTelemetryBatch` with every drone's SYSID. A drone
  completes only when it disarms within 10 m of its LAND point
  (`engine/mavlink_mission.py`); landing anywhere else is a failure. A
  drone that fails is marked failed and the others keep flying; a
  once-a-second watchdog fails a drone whose Renode process dies and stops
  its instance (physics sidecar included) at once.
- **End**: once every drone has ended, a per-drone summary and a fleet
  result are logged and every instance is stopped. **Stop** stops every
  drone. The next Plan Mission boots a fresh fleet.

Plan minimums: Search needs 2 drones, V-formation an odd number from 3, grid
formation 4.

## Per-instance isolation (`engine/renode_launcher.py`)

`RenodeLauncher(..., instance=N)`:

| | Instance 0 (single drone) | Instance N >= 1 (fleet drone N) |
|---|---|---|
| MAVLink (TCP) | 5762 | 5762 + N |
| Physics sidecar | 9002 | 9002 + N |
| Vehicle sysid | left at 1 | `MAV_SYSID` = N (this firmware's name for `SYSID_THISMAV`) |
| FRAM, persistent flash | the standalone folder's own | private copies in `renode_instances/instance-N/`, made once and kept, so parameters survive restarts without leaking between drones |
| SD card | fresh copy of the golden image, every launch | fresh copy of the golden image, every launch |
| Console log | `pixhawk6c_renode_standalone/renode-console.log` | `renode_instances/instance-N/renode-console.log` |

The standalone folder's files are only ever read. Instance 0's launch
command differs from main's only in the SD card path.

**Fresh SD per launch**: `renode_instances/golden-sdcard.img` is built once
from the standalone card: a copy is read with mtools, a new FAT32 image of
the same size, cluster size and label is formatted with `mkfs.fat`, only the
original's real files (111 entries, 2013 clusters) are copied in with
`mcopy`, and `fsck.fat -n` must pass. Every launch copies it into the
instance's work dir. Needs `mtools` and `dosfstools`.

**Cleanup**: `kill_all_renode_processes()` (called once before a fleet or
single launch) and each instance's own stale-process cleanup match only the
real executables: the kernel process name and argv[0] must both be exactly
`renode` or `renode-physics`. An instance's own cleanup only hits the
processes carrying its physics port.

**Port conflicts fail fast**: `start()` checks that the MAVLink and physics
ports are free before launching, and watches Renode's log for
`AddressAlreadyInUse` while booting - either fails that drone within
seconds instead of after the 240 s GPS wait.

## Memory

Renode runs with `DOTNET_GCConserveMemory=7` (set only in Renode's own
environment). Measured on one instance, RSS booted / flying:

| .NET GC setting | Booted (MB) | Flying (MB) |
|---|---|---|
| default | 3484 | 3531 |
| `DOTNET_gcServer=0` | 3548 | 3596 |
| **`DOTNET_GCConserveMemory=7` (used)** | **2276** | **2323** |
| `DOTNET_GCHeapHardLimit=0x60000000` | 3413 | 3462 |

Boot and climb times were unchanged. Renode's memory does not track the SD
image size (a 64 MiB image instead of 512 MiB saved ~100 MB).

In fleets each Renode instance used 2.1-2.35 GB (the physics sidecar ~6 MB).
On this 14 GiB machine: 3 drones flying left about 3 GiB available with no
swap; 4 drones left about 2 GiB available and started using swap.

## Degraded SD cards and EKF failsafes

The per-instance SD cards used to persist between launches. Every Renode
killed while ArduPilot was logging left orphaned FAT clusters behind; one
card had lost ~400 MB that way and stalled the firmware badly enough that
first-boot parameter sets went unacknowledged (proven by swapping that card
in and out). With persistent cards, 7 of 8 fleet flights on instances 1-2
ended in an EKF failsafe landing. Since every launch boots from a fresh
copy of the golden image: 0 of 16 flights in Steps 7-8 and 0 of 20 in Step 9
(17 completed on target, 3 stopped by the Stop test) ended in an EKF
failsafe or an early landing.

## Known limits

- **Memory**: ~2.2 GB per drone. On a 14 GiB machine, 3 drones fit
  comfortably; 4 need about 10 GiB available before launching; more than
  4 won't fit.
- **Grid formation needs 4 drones**; V-formation needs an odd number from 3.
- **SYSIDs 1-32** only (keeps the port ranges clear of VNC's 5900 and X11's
  6000); two checked profiles may not share one.
- **Speed**: emulation runs at roughly a quarter of real time; a boot takes
  about 2.5-3 minutes and a 50 m climb about 75 s.
- **Link loss**: a drone whose Renode dies is caught by the watchdog at
  once, but a link that goes silent while Renode keeps running is only
  given up on after `upload_and_fly`'s 600 s telemetry timeout.
- **App killed without closing**: if the GUI dies without its window
  closing (e.g. the desktop session crashing), its Renode processes are
  left running. The next launch clears them (`kill_all_renode_processes()`
  runs first), or run `tests/harness/rclean.sh`.

## Verifying

The scripts used to verify all of this are in `tests/harness/` (see its
README), e.g. `tests/harness/gui_drive.py fleet_travell3`.
