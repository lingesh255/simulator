# Single-Renode fleet: feasibility experiment (Task 16)

Question: can all fleet drones run as machines inside ONE Renode process,
instead of one process (about 2.2 GB) per drone?

Short answer: **yes, with one specific setting, and at a large speed cost.**
Four drones booted, flew and landed in one process using 2.2-2.5 GB in
total (against about 9 GB today), but the run took about three times as
long. Renode's normal multi-machine execution does not work with this
platform at all; the cause was not found.

Measured on 7 Oct 2026: Renode 1.16.1 (2a060779-202608220408), ArduCopter
4.8.0-dev on the Pixhawk6C platform, 16 cores, 15 GiB RAM.

## Verdict

**Conditional GO**, as an optional low-memory fleet mode, not as a
replacement for one process per drone.

- Expected memory for 4 drones: **about 2.2 GB after boot, 2.4 GB in
  flight, 2.5 GB at the end** (one Renode), plus 6 MB per physics sidecar.
- Cost: boot to armable about 12 minutes for 4 drones (about 4 minutes
  today), and a short flight about 26 minutes (about 8 today).
- It only works with `emulation SetGlobalSerialExecution true` **and** a
  100 us global quantum. Nothing else tried works (see Attempts).

Risks:

1. **The multi-machine fault is not understood.** The working setting was
   found by trial. It flew 2 drones once and 4 drones once; that is all the
   evidence there is that it is stable.
2. **Timeouts.** Everything that waits in wall-clock time has to be scaled.
   The launcher's defaults (GPS 240 s, provisioning 45 s, armable 180 s)
   are too short: with 4 drones the GPS fix alone came at 159-208 s and
   armable at 669-723 s. `upload_and_fly`'s own fixed timeouts (10 s per
   item, 5 x 1 s to confirm AUTO) held in these two runs, with no margin
   measured.
3. **One process is one point of failure.** Killing Renode drops every
   drone. `mach rem` on one machine crashes the whole process.
4. **A dead physics sidecar is silent.** The drone keeps sending telemetry
   from frozen sensor values; only Renode's console log says so.
5. **Speed falls with every drone**: all machines share about one host core.

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

## Attempts that failed

All with 2 machines unless noted. "Abort" is `CPU abort [PC=0xF092D004]:
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

- `make_fleet_resc.py` - the generator (defaults: serial execution, 100 us).
- `run_fleet.py` - boot, fly, measure, inject faults; `--mode separate`
  uses the existing launcher for comparison.
- `mon.py` - send commands to the Renode monitor of a `--telnet` run.
- `probe.sh`, `raw_probe.sh` - start a run detached and show its errors
  (the raw one runs the script with no MAVLink client and no sidecar).
- `out/` (not committed) - each run's log, Renode console, script and JSON.

Run `tests/harness/rclean.sh` before each run. Never put the cleanup and a
command that mentions the Renode binary or `run_fleet.py` in one shell
line: `pkill -f` then matches that shell too.
