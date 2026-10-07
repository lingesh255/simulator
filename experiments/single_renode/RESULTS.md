# Single-Renode fleet: feasibility experiments (Tasks 16 and 17)

Question: can all fleet drones run as machines inside ONE Renode process,
instead of one process (about 2.2 GB) per drone?

**Answer (Task 17): yes, in parallel, at close to the speed of separate
processes.** Four drones boot, fly and land in one Renode process using
2.3-2.7 GB instead of 8.8-9.0 GB, about 20 % slower end to end. The one
change that makes it work: each machine is created with its **own time
source** instead of with `mach create`.

Measured on 7-8 Oct 2026: the standalone folder's Renode (reports 1.16.1,
build 2a060779-202608220408), ArduCopter 4.8.0-dev on the Pixhawk6C
platform, 16 cores, 15 GiB RAM.

## Verdict (Task 17)

**GO: integrate "one Renode, parallel machines with their own time
sources" as the app's optional low-memory fleet mode.**

4 drones, same route (start -> 60 m north at 50 m), all completed with their
own sysid in every run:

| Option | Renode RSS: after boot / airborne / end (MB) | GPS fix (s) | Armable (s) | Arm -> 50 m (s) | Flight (s) | Launch to all landed (s) | Renode CPU (cores) |
|---|---|---|---|---|---|---|---|
| 4 separate processes (today) | 8758 / 8862 / 9023 | 63-65 | 192-195 | 102-109 | 367-390 | 587 | 4.46 |
| **1 process, parallel, own time sources** (recommended; master quantum 100 ms) | **2339 / 2454 / 2609** | 71 | 230-231 | 129 | 462-467 | **703** | 3.91 |
| the same, master quantum 10 ms (3 runs) | 2422-2466 / 2534-2583 / 2675-2727 | 74-75 | 239-244 | 133-135 | 484-490 | 729, 739, 736 | 3.68-3.71 |
| 1 process, serial, 100 us quantum (Task 16) | 2238 / 2397 / 2530 | 159-208 | 669-723 | 433-436 | 1568-1573 | 2300 | 1.09 |

- 2 drones in one process, parallel: 2187 / 2251 / 2326 MB, GPS fix 55 s,
  armable 174 s, arm -> 50 m 94 s, 519 s in all, 2.17 cores.
- With 4 separate processes airborne the machine had 1457 MiB available;
  with one process it had over 7 GiB.
- A 2x2 hybrid (17c) was not built: it was the fallback for parallel not
  working, and it would use about twice the memory of the recommended
  option for at most the 20 % it is behind.

Risks and things the integration must handle:

1. **It depends on a Renode internal.** `Machine(createLocalTimeSource:
   true)` is reachable only through the monitor's Python
   (`python "... AddMachine(Machine(True), 'droneN')"`); there is no
   monitor command for it. It worked in every run here (2 drones twice, 4
   drones four times, plus the fault runs).
2. **The standalone's Renode is not a stock build.** Official 1.17.0 and
   the 7 Oct 2026 nightly cannot load this platform (see lead 5), so the
   fix could not be cross-checked on another Renode, and the mechanism below
   is read from the public source, which is newer than this binary.
3. **Boot is a little slower** (armable about 230-240 s against about 195 s
   for 4). The launcher's 180 s armable timeout is measured from the end of
   provisioning; the margin is smaller here. The runs used 400 s.
4. **One process is one point of failure**: killing Renode drops all drones.
5. **A dead physics sidecar is silent on MAVLink** (as in Task 16): watch
   the sidecar processes.
6. **Stopping one drone**: `cpu IsHalted true` or `mach rem` (below);
   never `machine Pause`, and never un-halt a drone whose physics is gone.

## Task 17a: the cause of the parallel-mode fault

**Cause: `mach create` puts every machine on the emulation's single shared
time source.** In Renode's `Machine` constructor a machine only gets its
own `SlaveTimeSource` when `createLocalTimeSource` is true; `mach create`
does not pass it, so `machine.LocalTimeSource` is the shared master and
every machine's CPU is a direct sink of it. Each machine also subscribes
its timer handling (`HandleTimeProgress`, which advances its clock source
and fires its timer interrupts) to that shared source's `TimePassed` event.
With two machines, one machine's timers are therefore advanced when the
other machine's CPU thread reports time, on that other thread, while the
first machine's own CPU is running. Renode sets CPU interrupt flags without
locking, so interrupts get lost or delivered when nothing is pending.

That accounts for every Task 16 symptom: the spurious interrupt (exception
0x210) and the freezes in parallel mode; "Couldn't synchronize time before
scheduling action" (a machine's scheduled action running on a thread that
is not its CPU's); "An interrupt from the unsynchronized thread"; and, in
serial mode, only the last-created machine working unless the quantum is
tiny.

Evidence:

- Same script, same build, only the way the machines are created differs
  (`partest.sh`, 2 machines, parallel): `mach create` -> `ABORT`, virtual
  time stuck at 0.71 s; own time sources -> `RUNNING`, 13.9 s of virtual
  time after 25 s. Reproduced before and after making it the default.
- With own time sources: 2 drones flew once and 4 drones four times, all
  completed, 3.7-3.9 cores used (one busy thread per machine).

The leads, in the order given:

| Lead | What I tried | Result |
|---|---|---|
| 1. Reset macro, monitor-global names | Generated the script with no macro at all (its commands inline) | Still aborts. The macro only ever ran once per machine, at setup; nothing resets at run time. Not the cause. |
| 2. Command scoping | Checked the generated script: every per-machine line follows that machine's `mach create` / `mach set`, so it runs with that machine selected. Also ran with drone1 as the monitor's current machine at `start` (Task 16) | No unscoped line; no change. Not the cause. |
| 3. Python hooks | Ran without the three `SetHookBeforePeripheralWrite` lines | Still aborts (0.67 s). Not the cause. |
| 4. Peripherals on host threads | Dropped peripheral groups from patched platform copies: actuators / CAN / GPIO stimulus / timer-update DMA; DMA fixups and the circular ADC DMA -> still aborts. Counted "unsynchronized thread" interrupts on ONE machine: 103 in 20 s, 72-92 without IOMCU / DMA fixups / ADC DMA / GPIO stimulus, 0 without the SD-card controller | The SD controller (stock base class and `AP_STM32H7_SDMMC`) defers its transfers to "the next synced state" of `machine.LocalTimeSource`. Following that call into Renode's time code is what showed that `LocalTimeSource` is the shared master. The peripherals themselves are not at fault. |
| 5. Newer Renode | Downloaded official 1.17.0 and the 7 Oct 2026 nightly into `experiments/renode-new/` (gitignored) | Neither can load the platform: `AP_STM32_OTG.cs` does not compile (`IUSBDevice.ConnectUSB`, `USBEndpoint.NonBlocking` ... missing); with USB left out, `Miscellaneous.STM32F4_RNG` does not exist; with that left out, `STM32_DMAMUX` has a different constructor. The standalone's Renode is a modified build. Stopped there. |

Also done: a reflection dump of every static field in the standalone's
Renode assemblies (`list_statics.py`). Nothing in the CPU, bus, time or
interrupt-controller classes holds per-machine state.

## Task 17b: parallel, measured

See the verdict table. Three identical 4-drone runs (master quantum 10 ms):
4 of 4 completed each time, armable 239-244 s, arm -> 50 m 133-135 s,
landing error 0.01-0.12 m, no failsafes, 729 / 739 / 736 s. A fourth run
with the master time source's quantum raised to 100 ms (each machine keeps
its 10 ms quantum): 703 s, armable 230 s, arm -> 50 m 129 s, 3.91 cores.
The drones do not talk to each other inside Renode, so the master only has
to keep them roughly together; 100 ms is the generator's default.

Against the targets (armable about 210 s, arm -> 50 m about 99 s): 2
drones beat them (174 s, 94 s); 4 drones are at 230 s and 129 s, where 4
separate processes measured 192-195 s and 102-109 s on the same night.

### Failure behaviour in this mode (2 drones, on the ground after GPS fix)

1. Kill one sidecar: `drone2/physics: physics disconnected: ... Broken
   pipe`; Renode stays up, drone 1 unaffected, drone 2 keeps sending
   telemetry.
2. Stop one drone:
   - `cpu IsHalted true`: works. Only that drone's telemetry stops.
     Un-halting it after its physics is gone gives `CPU abort
     [PC=0xE000FFFE]` and stalls the other drone for a while.
   - `mach rem "drone2"`: **works in this mode** (it crashed Renode in
     Task 16's). The machine is gone, drone 1 carries on. Its sidecar is
     left running and its MAVLink port still accepts connections.
   - `machine Pause`: still stops both drones; `machine Start` resumes both.
3. Kill Renode: both stop at once; the two sidecars remain until stopped.

## Task 16 (earlier): what was tried before the cause was known

The rest of this file is the Task 16 record. Its verdict ("conditional GO,
serial execution with a 100 us quantum, three times slower") is superseded
by the above; its measurements stand.

## 16a: static-state audit

`ardupilot_h743.resc` and the board script include 61 `.cs` files. They
contain 31 `static` fields and **none is mutable per-device state**:

| Field(s) | Kind | Verdict |
|---|---|---|
| `AP_Physics.cs:344` `ConditionalWeakTable<IMachine, AP_PhysicsState> states` | readonly table keyed by machine (`ForMachine(machine)`) | safe, per machine |
| `AP_Physics.cs:245` `Stationary` | property returning a new object each time | safe |
| `AP_Physics.cs:740` `Magic`, `AP_RAMTRON.cs:151` `DeviceId`, `AP_UBlox.cs:1181` `RequiredRecords`, `AP_AdditionalCompasses.cs:764` `TemperatureRaw`, `AP_AdditionalBarometers.cs:795-796`, `AP_PowerMonitors.cs:798-879` (9 arrays), `AP_SerialRangefinders.cs:379-1503` (10 arrays), the three `*Beacon.cs` `AnchorsNedM` | readonly constant tables, never written (checked for element writes and `Add`/`Remove`) | safe |
| `AP_STM32_OTG.cs:974` `PropertyInfo` | readonly reflection handle | safe |

Other process-wide resources looked at:

- `AP_CANMcast.cs` (fixed UDP port 57732): it only opens sockets once its
  `Bus` is set; it defaults to -1 and nothing here sets it.
- `AP_STM32_OTG.cs:776` and `AP_Hotplug.cs:22` use the emulation-wide host
  machine / externals. OTG only does so if a USB/IP server named "usb"
  exists (none here); `AP_Hotplug` is not in the Pixhawk6C platform.
- FRAM, SD card and persistent flash each write a file: every drone has
  its own copy, as with separate processes.

So nothing in the custom peripherals explains the fault below.

## 16b: the generator

`make_fleet_resc.py` writes one script: variables and every `include *.cs`
once, then per drone `mach create "droneN"`, its own platform files, ELF,
vector table, reset macro, MAVLink server socket (`serial_dN`, port
5762+N), CAN hubs (`can1Hub_dN`, `can2Hub_dN`), SD card, bus hooks, physics
connection (9002+N) and GPS hub (`gpsHub_dN`), then the emulation-wide
settings and one `start`. `run_fleet.py` prepares each drone with
`RenodeLauncher(instance=N)` (same files, ports and sysid as today), starts
the sidecars, runs the one Renode, and reuses the launcher's own GPS-fix
wait, provisioning and armable wait.

What worked first time: including the `.cs` files before any `mach
create`; per-drone names for the emulation-level objects; `mach create` /
per-machine commands; AP_Physics with one sidecar per machine.

## 16c: measurements

Every drone flew start -> 60 m north at 50 m with `upload_and_fly` and had
to disarm within 10 m of its LAND point.

| Setup | Drones completed | Own sysid | Renode RSS: after boot / airborne / end (MB) | GPS fix (s) | Armable (s) | Arm -> 50 m (s) | Whole flight (s) | Landing error (m) | Failsafes | Renode CPU (cores) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 drone, one process, generated script (10 ms quantum) | 1 of 1 | 1 | 2127 / 2157 / 2192 | 64 | 193 | 104 | 373 | 0.08 | 0 | 1.12 |
| 2 drones, separate processes (existing launcher) | 2 of 2 | 1, 2 | 4395 / 4447 / 4450 | 74-75 | 211-215 | 99 | 334-341 | 0.04-0.11 | 0 | 2.27 |
| **2 drones, ONE process** (serial, 100 us) | 2 of 2 | 1, 2 | 2102 / 2172 / 2242 | 85-105 | 305-325 | 179 | 667-669 | 0.04-0.07 | 0 | 1.14 |
| **4 drones, ONE process** (serial, 100 us) | 4 of 4 | 1, 2, 3, 4 | 2238 / 2397 / 2530 | 159-208 | 669-723 | 433-436 | 1568-1573 | 0.00-0.09 | 0 | 1.09 |

- Each extra machine costs about 50-100 MB. The process itself is about
  2.1 GB whatever it holds.
- Memory only, 4 machines loaded, at 150 s: 2279 MB with
  `DOTNET_GCConserveMemory=7`, 3592 MB with the default GC.
- CPU: each machine has its own host thread (`droneN.cpu[0]`), but with
  serial execution they take turns: 4 drones used 1.09 cores in total
  (30 / 22 / 21 / 20 % per thread). The 16 cores are not used.
- Whole run, launch to all landed: 558 s for 2 separate processes, 998 s
  for 2 in one process, 2300 s for 4 in one process. For comparison, 4
  separate processes in Task 15 finished a longer route in 719 s.
- `free -m` with all airborne: 4 in one process left 8449 MiB available
  and no swap in use. The first two rows were measured earlier in the day
  with other apps holding memory (2.8-2.9 GiB available, 2-3.7 GiB swap in
  use), so their times are not a like-for-like speed baseline.

## 16d: failure behaviour

Done with 2 drones in one process **on the ground after the GPS fix**, not
in flight (a flight in this mode takes 15-25 minutes), watching each
drone's telemetry.

1. **Kill one drone's physics sidecar**: Renode logs `drone2/physics:
   physics disconnected: ... Broken pipe` and stays up. Drone 1 is
   unaffected. Drone 2 **keeps sending telemetry** (last message about 1 s
   old 15 s and 45 s later), so MAVLink alone does not show the failure.
2. **Stop one drone without killing the process**:
   - `cpu IsHalted true` on that machine: **works.** Its telemetry stops,
     the other drone carries on, Renode stays up. `physics Disconnect` and
     stopping its sidecar afterwards are fine. Its MAVLink port stays open.
     Do not un-halt it: `cpu IsHalted false` (with physics gone) gave `CPU
     abort [PC=0xE000FFFE]` and the other drone's telemetry stalled too.
   - `machine Pause`: stops **both** drones' telemetry. (`machine Resume`
     does not exist.)
   - `mach rem "drone2"`: **crashes the whole Renode** with an unhandled
     `ObjectDisposedException` in `AP_UARTFrameDevice.TransmitFrame`.
3. **Kill the whole Renode process**: both drones' telemetry stops at
   once. The two sidecars are left running until the driver stops them;
   after that nothing is left (`pgrep` empty).

## Task 16: attempts that failed

All with 2 machines created with `mach create` unless noted. "Abort" is `CPU abort [PC=0xF092D004]:
Trying to execute code outside RAM or ROM`, on one machine, a different one
from run to run.

| Attempt | Result |
|---|---|
| Default (parallel) execution, 10 ms quantum, one `start` | abort about 2.4 s after start (1.0 s virtual), 5 of 5 runs |
| `machine Start` per machine instead of `start` | no abort, but no heartbeat from either drone in 4 minutes |
| Without the three `SetHookBeforePeripheralWrite` lines | abort |
| Without the GPS hub lines | abort |
| Without `physics Connect` and the GPS hub | abort |
| No MAVLink client, no sidecar | abort |
| Renode's default quantum (100 us), parallel | abort (0.12 s virtual) |
| Default GC settings instead of `GCConserveMemory=7` | no abort; the emulation freezes at 1.03 s virtual |
| Everything logged (`logLevel -1`) | freeze at 1.11 s virtual |
| `emulation Mode SynchronizedTimers`, parallel | abort |
| Boot in serial mode, switch to parallel after 20 s | freeze 0.5 s later |
| Second machine's CPU halted | the first runs normally at 1.0x real time |
| Serial execution, 10 ms quantum (2 and 3 machines) | no crash, but only the LAST-created machine works; the others never reach a GPS fix |
| the same with drone1 as the monitor's current machine | still only the last-created machine works |
| the same with `SynchronizedTimers` | the same |
| the same with `SetGlobalAdvanceImmediately true` | the same |
| Serial execution, 1 ms quantum | the first machine still starved |
| **Serial execution, 100 us quantum** | **every machine works** (2 and 4 flown) |

What is known about the fault:

- The aborting CPU takes exception 0x210, which is Renode's own "no
  interrupt pending" value: the CPU was told an interrupt was pending when
  its interrupt controller had none. The vector it then reads
  (0x08020840) is firmware code, hence the constant bogus PC.
- In the frozen runs both CPUs are blocked inside a peripheral access
  (`ChibiOS::UARTDriver::write_pending_bytes_DMA` writing a DMA stream
  register) and every Renode thread is waiting.
- In serial mode with a 10 ms quantum the starved machines sit in the
  idle thread with their tick interrupt (TIM2) pending and untaken; their
  CPU wakes only every few seconds.
- With two machines Renode logs "Couldn't synchronize time before
  scheduling action" (47 times in 1.1 s; never with one machine) and, with
  `cpu ThreadSentinelEnabled true`, "An interrupt from the unsynchronized
  thread" about ten times as often for the first machine as for a single
  machine.
- Each CPU does get its own copy of the native translation library.

Not tried: another Renode version, and a managed stack dump of the frozen
process (`gdb` cannot attach here; `dotnet-stack` is not installed).

## Files

- `make_fleet_resc.py` - the generator. Defaults: own time source per
  machine, parallel, 10 ms machine quantum, 100 ms master quantum.
  `SINGLE_RENODE_SHARED_TIME=1` reproduces the Task 16 setup.
- `run_fleet.py` - boot, fly, measure, inject faults; `--mode separate`
  uses the existing launcher for comparison.
- `mon.py` - send commands to the Renode monitor of a `--telnet` run.
- `probe.sh`, `raw_probe.sh` - start a run detached and show its errors
  (the raw one runs the script with no MAVLink client and no sidecar).
- `partest.sh` - 30-second "does an N-machine script survive" check
  (ABORT / FROZEN / RUNNING); `sentinel.sh` counts unsynchronized-thread
  interrupts; `list_statics.py` dumps a Renode build's static fields;
  `nightly_try.sh` runs the script on the downloaded nightly.
- `experiments/renode-new/` (not committed, about 500 MB) - the two official
  Renode builds downloaded for lead 5, and the Renode source files read.
  Safe to delete.
- `out/` (not committed) - each run's log, Renode console, script and JSON.

Run `tests/harness/rclean.sh` before each run. Never put the cleanup and a
command that mentions the Renode binary or `run_fleet.py` in one shell
line: `pkill -f` then matches that shell too.
