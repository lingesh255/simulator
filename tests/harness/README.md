# Test harness

Scripts used to verify the Renode single-drone and fleet work end to end.
They need the standalone Renode package at `pixhawk6c_renode_standalone/`
(see the repo's `.gitignore`), ENHSP under `tools/enhsp/` for the GUI plans,
`fsck.fat` (dosfstools) for the golden SD image, and the repo's `.venv`.
Run everything from the repo root; logs from `gui_drive.py` land in
`tests/harness/out/` (gitignored).

Before and after every Renode run, clear stray processes:

```bash
tests/harness/rclean.sh
```

It kills and then lists only the real `renode-bin/renode` and
`renode-physics` executables. Don't put the bare word "renode" in a command
line that runs next to it - `pgrep -f`/`pkill -f` patterns match any
process whose command line contains the text, including your own shell.

## Unit tests

```bash
.venv/bin/python -m unittest tests.test_mavlink_mission tests.test_mission_steps tests.test_shared_renode -v
```

## GUI scenarios - `gui_drive.py <scenario>`

Builds the real `MainWindow`, operates its widgets and the map's own click
handler (`MapViewer._on_map_clicked`), timestamps every event, and closes
the window at the end. Needs a display.

```bash
.venv/bin/python tests/harness/gui_drive.py baseline
```

| Scenario | What it does |
|---|---|
| `baseline` | D1, Launch Renode by hand, then travell Canberra -> 330 m north (the Phase 0 reference flight) |
| `step4_mock` | D1 against the built-in mock vehicle (`scripts/mock_sitl.py`) |
| `fleet_travell`, `fleet_travell3` | 2 / 3 drones, travell, routes offset 18 m east per drone |
| `fleet_search`, `fleet_search3` | 2 / 3 drones, Search lanes over a ~45 m square |
| `fleet_stop`, `fleet_stop3` | 2 / 3 drones, Stop pressed once all are above 15 m |
| `fleet_bootfail`, `fleet_bootfail3` | the last drone's MAVLink port pre-occupied by `port_squatter.py` |
| `fleet_kill2` | 3 drones, drone 2's Renode killed (SIGKILL) once all are above 20 m |
| `fleet_dupsysid` | adds a temporary profile sharing SYSID 1, expects the fleet to be refused, removes it |
| `fleet_sidecar3` | 3 drones, drone 2's physics sidecar killed (SIGKILL) once all are above 20 m - meant for the shared mode, where only the sidecar shows the failure |
| `fleet_stopboot3` | 3 drones, Stop pressed 60 s into the fleet boot |
| `fleet_twice3` | two fleet missions back to back in one app session |
| `fleet_switch3` | per-drone, then shared, then per-drone fleets in one app session |
| `fleet_stopthen3` | Stop mid-flight, then a new mission straight away |
| `fleet_travell5`, `fleet_travell6` | 5 / 6 drones (adds temporary `harness_d<N>.json` profiles, removes them afterwards) |
| `fleet_sidecar4` | 4 drones, drone 2's physics sidecar killed mid-flight |
| `fleet_long3` | 3 drones on a ~3 km route (about 30 minutes of flying) |
| `fleet_close3`, `fleet_sigterm3`, `fleet_sigkill3` | the app window closed / SIGTERM / SIGKILL to the app with 3 drones airborne (run `fleet_sigkill3` on its own with `gui_drive.py`, then another fleet scenario without cleaning up, to see the next launch clear the orphans) |
| `fleet_single1` | one drone checked: must use the single-drone path whatever the fleet emulation says |
| `fleet_norenode1` | a typed address, then the mock vehicle: the control is off and no Renode starts |
| `fleet_renodekill3` | 3 drones, every Renode process killed (SIGKILL) once all are above 20 m - in the shared mode that is the one Renode |
| `table_demo` | D1-D3 on the local preview: the Flight Log table, Table/Logs toggle and Logs badge, with screenshots (prefix from `TABLE_DEMO_PREFIX`, default `s13`) |

Fleet scenarios live in `fleet_scenarios.py`. `fleet_vform3` flies a
3-drone V-formation and `fleet_grid4` a 4-drone grid. For the fourth drone
it adds a temporary profile, `data/profiles/harness_grid_d4.json` (SYSID 4),
checks only that one plus D1-D3, and removes only that file afterwards; it
refuses to start if the file already exists, and never touches a real D4.

To run several scenarios back to back - clearing Renode before each,
printing the exit code and pgrep after each (and `free -m` before
`fleet_grid4`, for information):

```bash
tests/harness/regression.sh s9e baseline fleet_travell3 fleet_search3 fleet_stop3 fleet_bootfail3
```

Any fleet scenario flies in the shared-Renode mode with
`FLEET_EMULATION=shared` in the environment (`per_drone` forces the default;
unset uses the saved setting). The choice is not written to the settings
file:

```bash
FLEET_EMULATION=shared tests/harness/regression.sh s18sh fleet_travell3 fleet_sidecar3
```

Each scenario's log lands in `tests/harness/out/<prefix>_<scenario>.out`.
`summarize.py <prefix>` then prints one line per scenario: exit code,
processes left, the fleet result, failsafe count, Renode RSS in flight, and
the seconds to fleet ready and finished.
It can run detached (`nohup setsid ... &`) so it survives the terminal.

## Headless checks

| Script | Checks |
|---|---|
| `step1_test.py ctrlc kill close` | Ctrl+C, `kill`, and window close all stop Renode and leave the terminal echoing (runs `step1_app.py` on a pseudo-terminal) |
| `multi_check.py` | two instances at once: ports, files, sysids, param isolation across a restart, simultaneous 10 m flights |
| `kill_match_check.py` | cleanup kills the real Renode executables but none of four look-alike dummies |
| `mock_fly_check.py` | a 3-point route against `scripts/mock_sitl.py` completes by itself on the LAND point |
| `mem_check.py <label> [VAR=VALUE ...]` | one instance's RSS idle and during a 50 m flight, with optional extra Renode environment |
| `mem_sd.py <instance> normal\|small64` | RSS with the normal vs a 64 MiB SD image |
| `sd_check.py` | golden SD image passes `fsck.fat -n`, original untouched, instance 0's command vs main, fresh SD copy on relaunch |
| `port_conflict_check.py` | a taken MAVLink port fails the instance within seconds, both via the pre-boot check and via Renode's log |
| `flight_check.py <instance>` | one headless boot + 110 m flight on an instance; reports EKF failsafes and where it landed |
| `boot_check.py [instance]` | one instance to armable and stopped again, no flight |
| `launch_command_check.py` | prints the launch command for instances 0, 1 and 3 without starting anything; diff it between two checkouts |
| `shared_fleet_check.py [n]` | `SharedRenodeFleet`: n drones in one Renode to armable, pids and memory, `stop_drone()` on the last one while the others keep sending heartbeats, `stop_all()` |

```bash
.venv/bin/python tests/harness/sd_check.py
```

`port_squatter.py <port>` just listens on a port and never answers.
