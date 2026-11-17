# Drone Swarm Simulator

A desktop ground-control station (PySide6) that plans drone missions with the
[ENHSP](https://github.com/hstairs/enhsp) PDDL planner and flies them on one
drone or a whole swarm. The same plan can be flown four ways:

| Vehicle | What it is | Needs |
|---|---|---|
| **In-app simulation** | A built-in kinematic model, one thread per drone. Fast, nothing to install beyond Python. | Python only |
| **Mock MAVLink vehicle** | `scripts/mock_sitl.py`, a small fake autopilot that speaks real MAVLink. Tests the real mission upload / arm / fly code without firmware. | Python only |
| **Renode (real firmware)** | Unmodified ArduCopter firmware running on an emulated Pixhawk 6C in [Renode](https://renode.io). One Renode per drone, or one shared Renode for a fleet. | The Renode package (see [Setup](#setup)) |
| **Your own vehicle** | Any MAVLink endpoint you type in: ArduPilot SITL, or real hardware. | That vehicle |

---

## Contents

1. [Setup](#setup)
2. [Run the app](#run-the-app)
3. [Your first flights](#your-first-flights)
4. [Plans](#plans)
5. [Drone profiles](#drone-profiles)
6. [Project layout](#project-layout)
7. [Testing](#testing)
8. [Troubleshooting](#troubleshooting)
9. [Contributing](#contributing)
10. [Further reading](#further-reading)

---

## Setup

Tested on Ubuntu (native and WSL2) with Python 3.14. Any Python 3.10 or newer
should work.

### 1. Get the code and the Python dependencies

```bash
git clone https://github.com/lingesh255/simulator.git
cd simulator
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Every command in this README is run from the repository root and uses the
`.venv` interpreter.

### 2. Install the ENHSP planner (needed for every plan)

```bash
bash scripts/setup_enhsp.sh
```

This installs OpenJDK 17 if Java is missing (it asks for `sudo`), clones ENHSP
into `tools/enhsp/` and builds `tools/enhsp/enhsp-dist/enhsp.jar`. It is safe
to re-run.

### 3. Set up Renode (only for real-firmware flights)

Skip this step if you only want the in-app simulation and the mock vehicle.

1. **Get the Renode package.** `pixhawk6c_renode_standalone/` (about 750 MB:
   Renode, the physics sidecar, the ArduCopter firmware and its SD card
   image) is **not in git**. Ask a team member for a copy and place it in the
   repository root, so you have `simulator/pixhawk6c_renode_standalone/`.
   Never commit it, and never edit the files inside it: the app only reads
   them and makes per-drone copies under `renode_instances/`.
2. **Install the SD-card tools.** Each launch builds a fresh SD card image:
   ```bash
   sudo apt install mtools dosfstools
   ```
3. **Check your memory.** Each Renode drone uses about 2.3 GB of RAM. On a
   14 GB machine, 3 drones in "One Renode per drone" mode is comfortable. For
   more drones, use **Shared Renode (low memory)** mode, where 3 or 4 drones
   use about 2.5-2.7 GB in total.

### 4. Check the install

```bash
.venv/bin/python -m unittest discover -s tests
```

All tests should pass (`OK`). They need neither Renode nor a display.

---

## Run the app

```bash
.venv/bin/python main.py
```

Close the window (or press Ctrl+C in the terminal) to quit. Closing the app
stops every Renode process and mock vehicle it started.

The window's main areas:

- **Drone Management** (top left): your drone profiles. Check the drones
  that should fly.
- **Map** (top centre): drones, routes and the points you click. Map tiles
  come from OpenStreetMap, so the first view of an area needs internet.
  **Download / Cache This Region** saves tiles to `data/tiles/` for offline
  use.
- **Fault Injection** (top right): GPS loss, comms loss or battery failure
  for chosen drones during an in-app simulation.
- **Flight Log** (bottom left): **Table** shows one row per drone (state,
  mission step, position, battery, link). **Logs** shows the full text log.
- **Mission Planner** (bottom right): choose a plan, how to fly it, and
  press **Plan Mission**.
- **Toolbar** (top): the in-app simulation's **Flight source** and the
  dark/light theme.

---

## Your first flights

Do these in order: each one adds a piece of the system.

### A. In-app simulation (no Renode)

1. In **Drone Management**, check **D1**.
2. In **Mission Planner**, pick **Travell** and leave **Fly via real
   MAVLink** unchecked.
3. Press **Plan Mission**, then click a **Start** point and a **Destination**
   on the map.

ENHSP solves the plan (the action list appears in the Mission Planner) and
the drone flies it on the map. Check D1, D2 and D3 to fly a swarm.

### B. Mock MAVLink vehicles

Same as A, but in **Mission Planner** also check **Fly via real MAVLink** and
**Use built-in mock vehicle**. The app starts one mock vehicle per drone on
`udp:127.0.0.1:14550`, `14551`, and so on. It then uploads each mission, arms
the vehicles, and flies them with the same code a real autopilot gets.

### C. Real firmware in Renode

1. In **Mission Planner**, check **Fly via real MAVLink**. Leave **Use
   built-in mock vehicle** unchecked and the **Connect** box empty: empty
   means "Renode is the vehicle".
2. Check one drone (D1) and plan a Travell mission as in A.

Renode boots at your Start point. This takes about 2.5-3 minutes; the status
line shows progress. The mission then starts by itself. Emulation runs at
roughly a quarter of real time, so a 50 m climb takes over a minute.

**Fleets:** check two or more drones and the app boots one emulated
Pixhawk per drone. Each drone's SYSID is its Renode instance number, with
MAVLink on `tcp:127.0.0.1:5762+SYSID`. **Fleet emulation** chooses:

- **One Renode per drone**: the most isolated option. Each drone uses about
  2.3 GB of RAM.
- **Shared Renode (low memory)**: every drone in one Renode process.

[MULTI_DRONE.md](MULTI_DRONE.md) explains both modes, their measured
performance and their limits.

### D. Your own SITL or hardware

Check **Fly via real MAVLink** and type the vehicle's address into
**Connect**, e.g. `udp:127.0.0.1:14550`. No Renode is started.

- A Travell plan flies one vehicle on that address (`udp:` or `tcp:`).
- Search, V-Formation and Grid-Formation fly one vehicle per drone and need
  a `udp:host:port` address: drone *i* (counting from 0) is flown on port
  + *i*, so `udp:127.0.0.1:14550` means 14550, 14551, 14552 and so on.

---

## Plans

Each folder in `plans/` is a mission template: a PDDL `domain.pddl` plus a
problem file. When you press **Plan Mission**, the template is moved onto the
points you clicked, expanded to the number of checked drones, and solved by
ENHSP. Each solve is saved under `plans/<name>/runs/`, which git ignores.

| Plan | Folder | What you click | Drones |
|---|---|---|---|
| **Travell** | `plans/travell/` | Start, Destination. Optionally, a restricted area to fly around (tick **Mark a restricted area**) | 1 or more, all on the same route |
| **Search Forest** | `plans/Search/` | Base point, then the 4 corners of the search area | 2 or more, one search lane each |
| **V-Formation** | `plans/vformation/` | Start, Destination | An odd number, from 3 |
| **Grid-Formation** | `plans/gridformation/` | Start, Destination | 4 |

[plans/PLANS_STATUS.txt](plans/PLANS_STATUS.txt) describes each plan in more
detail. Some plans can also be flown from the command line, without the GUI:
see the `scripts/pddl_*_to_mavlink.py` files. Each script's `--help`
explains its options.

---

## Drone profiles

A profile is one drone's configuration: name, SYSID, mass, maximum speed,
battery and cruise altitude. Profiles are JSON files in `data/profiles/`.
Create, edit and delete them with **New / Edit / Delete** in Drone
Management.

- Every drone flown in a Renode fleet needs a **unique SYSID from 1 to 32**.
- Only the three shared profiles (`d1.json`-`d3.json`) are tracked by git.
  New profiles you create stay on your machine.
- App settings (theme, fleet emulation mode) are saved in
  `data/app_settings.json`.

---

## Project layout

| Path | What is there |
|---|---|
| `main.py` | Entry point |
| `gui/` | The window and its panels (`main_window.py` wires everything together), plus `qml/Map.qml` for the map |
| `services/` | What the GUI runs: planning (`plan_service.py`), the in-app simulation (`thread_backend.py`, `local_flight.py`), MAVLink flights (`mavlink_flight_service.py` for one drone, `mavlink_swarm_flight_service.py` for mock or typed-address swarms), Renode fleets (`fleet_mission.py`) and profile storage |
| `engine/` | The core logic: the MAVLink mission upload and flight (`mavlink_mission.py`), Renode launch (`renode_launcher.py`) and the shared-Renode fleet (`shared_renode.py`), the PDDL planner wrapper (`pddl_planner.py`), drone-to-drone messaging for formations (`peer_bus.py`), terrain and geometry |
| `contracts/` | Shared Pydantic data models (drone profiles, telemetry, commands) |
| `plans/` | PDDL mission templates, one folder per plan |
| `scripts/` | The mock vehicle (`mock_sitl.py`), the ENHSP installer and the command-line plan flyers |
| `data/` | Profiles, settings and the cached map tiles |
| `tests/` | Unit tests (`test_*.py`) and the GUI regression harness (`tests/harness/`) |
| `experiments/` | Investigation scripts and their results; not used by the app |
| `backend/`, `firmware/`, `platforms/`, `eeprom.bin` | From the project's earlier, simulated-hardware design. The app's Renode flights do not use them, and the toolbar's **Connect to Telemetry / Inject Hardware ISR / Restore State** buttons expect a UDP orchestrator that is not in this repository |

Not in git, but created or needed locally: `pixhawk6c_renode_standalone/`
(the Renode package), `tools/enhsp/` (the planner), `renode_instances/`
(per-drone working copies and logs, safe to delete when no Renode is
running) and `.venv/`.

---

## Testing

**Unit tests**: fast, no Renode or display needed. Run them before every
commit:

```bash
.venv/bin/python -m unittest discover -s tests
```

**GUI regression harness**: drives the real window through complete flights
and records what happened. It needs a display, and Renode for most scenarios.
For example:

```bash
.venv/bin/python tests/harness/gui_drive.py step4_mock      # one drone, mock vehicle (about 1 min)
.venv/bin/python tests/harness/gui_drive.py mock_vform3     # V-formation, 3 mock vehicles (about 1.5 min)
.venv/bin/python tests/harness/gui_drive.py fleet_travell3  # 3 Renode drones (about 10 min)
```

To run several scenarios in a row, with Renode cleaned up between them and
logs in `tests/harness/out/`:

```bash
tests/harness/regression.sh mycheck baseline step4_mock mock_search2 mock_vform3 fleet_travell3 | tee tests/harness/out/mycheck_chain.txt
```

Then summarise the results (this reads the `_chain.txt` file saved above):

```bash
.venv/bin/python tests/harness/summarize.py mycheck
```

[tests/harness/README.md](tests/harness/README.md) lists every scenario.
If you change flight, planning or Renode code, run the scenarios that cover
it before you push.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| A Renode launch fails with "port already in use", or old Renode processes are still running after a crash | `tests/harness/rclean.sh` stops every Renode and physics-sidecar process (and nothing else). The app also clears leftovers itself before each launch. |
| "Could not load the Qt platform plugin xcb" | `sudo apt install libxcb-cursor0` |
| The map is grey | No internet for OpenStreetMap tiles. Connect once and use **Download / Cache This Region**. |
| Planning fails with an ENHSP or Java error | Re-run `bash scripts/setup_enhsp.sh` and check that `java -version` works. |
| "No solution" for a plan | The clicked points may be infeasible (for example, a restricted area that blocks every path). Try other points. Search needs at least 2 drones, V-Formation an odd number from 3, and Grid-Formation 4. |
| The machine slows down or swaps during a fleet | Fly fewer drones, close other apps, or switch **Fleet emulation** to **Shared Renode (low memory)**. |
| The map is slow under WSL | The app already switches the map to software rendering when there is no GPU. Make sure nothing sets `QT_QUICK_BACKEND`. |
| A Renode drone fails to boot | Read its console log: `renode_instances/instance-<N>/renode-console.log` for fleet drone N, `pixhawk6c_renode_standalone/renode-console.log` for a single drone, or `renode_instances/shared/renode-console.log` in shared mode. |

### Known issues

- **V-Formation on real ArduPilot over a typed address** (and
  `scripts/pddl_vformation_to_mavlink.py --connect`): the drones take off one
  after another and reach their slots, but the formation never sets off on
  its route. Each drone's climb-out mission ends in a "loiter forever" item,
  and `engine/mavlink_mission.py` waits for a `MISSION_ITEM_REACHED` that
  ArduCopter never sends for that item. Mock vehicles do send it, so mock
  flights are not affected. Renode fleets are not affected either: they fly
  each drone's slot route directly, without this climb-out step.

---

## Contributing

- Work on a branch, not directly on `main`.
- Before you push: the unit tests pass, and for flight, planning or Renode
  changes, the harness scenarios that cover your change pass too.
- Don't commit `pixhawk6c_renode_standalone/`, `tools/`, `renode_instances/`,
  `tests/harness/out/`, `__pycache__/`, or personal profiles and settings.
  `.gitignore` already covers most of these.
- If you add a feature, add a unit test or a harness scenario for it, and
  update this README if the setup or the way people use the app changes.

---

## Further reading

- [MULTI_DRONE.md](MULTI_DRONE.md): Renode fleets in depth (boot, routes,
  separation, the two emulation modes, memory, known limits).
- [tests/harness/README.md](tests/harness/README.md): every regression
  scenario and the headless check scripts.
- [plans/PLANS_STATUS.txt](plans/PLANS_STATUS.txt): the four plans and their
  status.
- [experiments/single_renode/RESULTS.md](experiments/single_renode/RESULTS.md):
  how the shared-Renode mode was developed and measured.
