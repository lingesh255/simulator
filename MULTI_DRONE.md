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
- **Flight Log**: the Table view shows one row per drone (State, current
  mission step, position, battery, link); **Logs** switches to the text log.

Plan minimums: Search needs 2 drones, V-formation an odd number from 3, grid
formation 4.

## Fleet emulation: one Renode per drone, or one shared Renode

### Choosing a mode

**Mission Planner > Fleet emulation** (saved as `fleet_emulation` in
`data/app_settings.json`) chooses how a fleet of two or more drones is
emulated. It is enabled once **Fly via real MAVLink** is checked. A single
drone always uses its own Renode, whatever it says.

- **One Renode per drone** (the default): each drone gets its own Renode
  process, about 2.2 GB each. A Renode that dies takes one drone with it.
- **Shared Renode (low memory)**: every drone is a machine inside ONE
  Renode process, about 2.4-2.7 GB in all. About 10-15 % slower end to end.
  If that Renode dies, every drone fails.

The fleet log says which is in use ("Booting a fleet of 4 drones in one
shared Renode (low memory) ...").

### Measured (8 Oct 2026, GUI regression, both modes on the same routes)

| Scenario | Result (both modes) | Renode RSS in flight: per drone / shared | Fleet ready: per drone / shared | Fleet finished: per drone / shared |
|---|---|---|---|---|
| Travell, 3 drones | 3 of 3 completed | 6717 / 2379 MB | 177 / 202 s | 551 / 631 s |
| Search, 3 drones | 3 of 3 completed | 6583 / 2399 MB | 178 / 198 s | 609 / 672 s |
| V-formation, 3 drones | 3 of 3 completed | 6480 / 2322 MB | 181 / 202 s | 570 / 631 s |
| **Grid, 4 drones** | 4 of 4 completed | **8536 / 2673 MB** | 198 / 234 s | 644 / 733 s |
| Stop pressed mid-flight, 3 drones | 0 of 3, all "stopped by the user" | - | 186 / 205 s | 233 / 257 s |
| Last drone's MAVLink port taken | fleet refused in under a second, nothing started | - | - | - |
| One drone's emulation killed mid-flight | 2 of 3 completed, that drone failed | 6865 / 2525 MB | 181 / 203 s | 528 / 608 s |

Every run exited cleanly with no Renode or sidecar left and no EKF
failsafe. The grid's landing spread is the same in both modes (10.0 m
sides, 14.1 m diagonals). "One drone's emulation killed" is that drone's
Renode in the per-drone mode and its physics sidecar in the shared mode.
Shared mode only: the one Renode killed mid-flight fails all three drones
("the shared Renode exited mid-flight") and leaves nothing running; Stop
pressed 60 s into the boot stops everything within about a second.

### How the shared mode works (`engine/shared_renode.py`)

- `generate_fleet_script` writes one Renode script: every peripheral source
  included once, then one machine per drone (`drone<SYSID>`) with that
  drone's own platform files, MAVLink socket (`serial_d<N>`, port 5762+N),
  CAN and GPS hubs, SD card and physics connection (9002+N), then one
  `start`. Ports, SYSIDs, SD / FRAM / flash copies and the physics sidecars
  are exactly the per-drone ones described below.
- Each machine is created with its own time source, through the monitor's
  Python because the monitor has no command for it:
  `AddMachine(Machine(True), 'droneN')`.
- The machines run in parallel, one host thread each. Each keeps the 10 ms
  quantum of the standalone scripts; the master time source brings them
  back in step every 100 ms (they never talk to each other inside Renode).
- `SharedRenodeFleet.start()` checks every port first, starts the N
  sidecars and the one Renode, then takes every drone through the same GPS
  fix, provisioning and armable steps as the per-drone launcher, in
  parallel. Those waits are stretched by 25 % per extra drone. If any drone
  fails to boot, everything is stopped and the error names it.
- Routes, separation, the landing check, the Flight Log table and Stop are
  the same code as in the per-drone mode.
- The script and Renode's console log are in `renode_instances/shared/`.

### Why `mach create` does not work, and the fix

With the monitor's `mach create`, every machine shares the emulation's one
master time source. One machine's timers are then advanced from another
machine's CPU thread while its own CPU is running, and interrupts are lost
or delivered with nothing pending: with two of these machines a CPU aborts
(`CPU abort [PC=0xF092D004]`) or the emulation freezes within a second.
Creating each machine with its own time source removes that; nothing else
had to change. The full investigation is in
`experiments/single_renode/RESULTS.md`.

### The watchdog, and stopping one drone

- If the one Renode dies, every flying drone is failed at once.
- If one drone's physics sidecar dies, only that drone is failed and
  stopped; the others keep flying. A drone whose sidecar has died keeps
  sending MAVLink from frozen sensor values, so only the sidecar process
  shows it - MAVLink does not.
- Stopping one drone (`SharedRenodeFleet.stop_drone`): through Renode's
  monitor, `mach set "droneN"`, `cpu IsHalted true`, `physics Disconnect`;
  then its sidecar is stopped. The machine stays halted until the fleet
  ends.

### Limits and risks of the shared mode

- It relies on a Renode internal (`Machine(createLocalTimeSource: true)`)
  reached through the monitor's Python.
- One process is one point of failure.
- Never `machine Pause` (it stops every machine) and never un-halt a
  stopped drone (without its physics it crashes and stalls the others).
- The standalone folder's Renode is a modified build: official Renode
  1.17.0 and the 7 Oct 2026 nightly cannot load this platform, so the mode
  has only ever run on this one build.
- Renode's monitor listens on a local TCP port for the length of the run.

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

- **Memory**: each Renode instance uses 1.8-2.3 GB, so 4 drones use about
  8-9 GB. In testing on this 14 GiB machine, 3 flying left about 2-3 GiB
  available; 4 flying left 2.2 GiB available with 2.9 GiB of swap in use.
  With other apps open, expect heavier swapping.
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
