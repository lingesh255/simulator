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
.venv/bin/python -m unittest tests.test_mavlink_mission -v
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

Each scenario's log lands in `tests/harness/out/<prefix>_<scenario>.out`.
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

```bash
.venv/bin/python tests/harness/sd_check.py
```

`port_squatter.py <port>` just listens on a port and never answers.
