# Speeding up the shared-Renode mode, and splitting a fleet across several Renodes (Task 21)

Experiments only; nothing in the app changed. Measured 8-9 Oct 2026 on
Paris's laptop: i7-12650H, 15 GiB RAM, the standalone folder's Renode.
Every drone flew start -> 60 m north at 50 m with `upload_and_fly`. A run
counts as OK only if every drone completed with its own sysid, with no EKF
failsafe, and nothing was left running.

## Short answers

- **Is there a way to speed up shared execution?** A little. The only knob
  that helps is CPU affinity, and only for small fleets: pinning Renode to
  the P-cores makes 4 drones about 7 % faster (landed in 653 s instead of
  703 s). At 6 and 8 drones the same pinning makes things slower.
  Everything else tried (advance immediately, bigger quanta, default GC,
  sidecar placement) changes nothing or costs.
- **Does splitting the fleet over several Renodes reduce the time?** Yes.
  8 drones as 2 Renodes x 4 landed in 962 s against 1094 s in one Renode
  (12 % faster) for 2.2 GB more RAM. 6 drones as 3 x 2 landed in 741 s
  against 894 s in one (17 % faster) for 3.7 GB more.
- **Why it is slow at all:** one emulated flight controller runs at about
  a third of real time on one host thread, flat out. That is the ceiling;
  nothing here raises it without changing emulation fidelity.

## Recommendation: the fastest correct configuration found

| Fleet | Configuration | Launch to all landed | Armable | Renode RAM in flight | Against one shared Renode today |
|---|---|---|---|---|---|
| 4 drones | 1 Renode, pinned to the P-cores | 653 s | 217 s | 2.6 GB | 703 s, 2.6 GB (7 % faster, same RAM) |
| 6 drones | 3 Renodes x 2, not pinned | 741 s | 243-246 s | 6.6 GB | 894-900 s, 2.9 GB (17 % faster, +3.7 GB) |
| 6 drones, less RAM | 2 Renodes x 3, pinned to the P-cores | 771 s | 249-253 s | 4.9 GB | 14 % faster, +2.1 GB |
| 8 drones | 2 Renodes x 4, not pinned | 962 s | 313-316 s | 5.1 GB | 1094 s, 3.0 GB (12 % faster, +2.2 GB) |

Rules of thumb from the numbers: each extra Renode process costs about
2.2-2.4 GB and buys about 8-12 %; groups of 2-4 drones per process are the
useful range; pin to the P-cores only when the whole fleet has about 4 or
5 machine threads, never when it has more than the 6 P-cores.

Not measured: 8 drones as 4 x 2 (about 9.6 GB of Renode, did not fit), and
6 drones as 3 x 2 pinned (skipped by the memory guard, 1310 MB would have
been left).

## 21a: where the time goes

CPU topology (`lscpu -e`): logical CPUs 0-11 are the 6 P-cores with
hyper-threading (4.6-4.7 GHz), CPUs 12-15 are the 4 E-cores (3.5 GHz).

Real-time factor (RTF) = virtual seconds per wall second, per machine, from
the monitor's `machine ElapsedVirtualTime`, sampled every 30 s.

| | RTF booting | RTF flying | Machine threads' CPU | Where the hot threads ran |
|---|---|---|---|---|
| 1 drone, one Renode | 0.32 (0.35 pinned to P-cores) | - | 96 % of one core | a P-core |
| 4 drones, one Renode | 0.20-0.21 | 0.195 | about 85 % each | 3 of the 4 on E-cores (CPUs 12, 14, 15) |
| 6 drones, one Renode | 0.16 | 0.152 | about 78 % each | 2 on E-cores (13, 15), 4 on P-cores |
| 8 drones, one Renode | 0.13-0.16 | 0.125 | 6.6 cores in all | - |

What limits it:

1. **Single-thread emulation speed is the ceiling.** One machine alone runs
   at RTF 0.32-0.35 with its thread at 96 % of a core. The physics sidecars
   use almost nothing (0.01-0.02 cores each).
2. **Every machine in one Renode moves at the pace of the slowest.** The
   RTF of all machines in a process is identical to three decimals in every
   sample: they are brought back in step at each master sync, so one slow
   thread slows them all, and the others idle (85 % then 78 % busy).
3. **The slow threads are the ones on E-cores or sharing a P-core.**
   Renode pinned entirely to the E-cores takes 397 s to boot 4 drones
   against 236 s unpinned and 217-220 s on the P-cores: an E-core is worth
   a bit over half a P-core here. Linux puts hot machine threads on E-cores
   even when P-cores are free.
4. **Why 6 drones are so much slower than 5** (ready 379 s against 265 s in
   Task 20): there are 6 P-cores. Up to about 5 machine threads can each
   have a P-core while Renode's other threads (one socket thread per drone
   at about 9-12 %, the monitor), the sidecars and the app use the rest.
   From 6 on, at least one machine thread lives on an E-core or shares a
   P-core with something busy, and point 2 drags the whole fleet down to
   that thread's pace.

It is not the quantum barrier as such (bigger quanta change nothing) and
not memory or GC.

## 21b: speed knobs on one shared Renode

One change at a time against today's settings; 4 drones, boot only (time
until all are armable, and the RTF while booting). Today's setting was run
5 times: 232-236 s.

| Knob | All armable (s) | RTF booting | Verdict |
|---|---|---|---|
| Today's settings (5 runs) | 232, 233, 235, 235, 236 | 0.213-0.220 | baseline, spread under 2 % |
| Renode on the P-cores (CPUs 0-11), sidecars on the E-cores | 219 | 0.224 | **helps, about 7 %** |
| Renode on the P-cores, sidecars free | 220 | 0.221 | same |
| Renode on the P-cores, sidecars on the P-cores too | 224 | 0.222 | same |
| Renode on one CPU per P-core (0,2,..,10), sidecars on the E-cores | 217 | 0.238 | same |
| Renode on CPUs 0,2,..,10, sidecars on 1,3,..,11 | 222 | 0.221 | same |
| Renode on the E-cores (control) | 397 | 0.146 | much slower, as expected |
| `SetGlobalAdvanceImmediately true` | 247 | 0.213 (min 0.170) | slower; not worth a flight |
| Machine quantum 0.02 | 237 | 0.216 | no change |
| Machine quantum 0.05 | 240 | 0.220 | no change (and MAVLink bytes would wait up to 50 ms of virtual time) |
| Master quantum 0.2 | 236 | 0.213 | no change |
| Default GC (no `DOTNET_GCConserveMemory=7`) | 235 | 0.217 | no faster; **RSS 3740 MB instead of about 2400** |

6 drones, boot only: today 295-297 s; Renode on the P-cores 279 s; one CPU
per P-core 288 s.

Combined best = "Renode on the P-cores" alone, since nothing else helped.
Full flights with it:

| | Today | Pinned to the P-cores |
|---|---|---|
| 4 drones, launch to all landed | 703 s | **653 s** |
| 6 drones | 900 s, 894 s (two runs) | 960 s, 986 s (two runs; the second with little free memory) |
| 8 drones | 1094 s | 1146 s |

So pinning helps at 4 and hurts at 6 and 8, although the 6-drone boot-only
run looked better pinned. In flight six pinned machine threads have to
share the six P-cores with everything else Renode does; unpinned, the
E-cores absorb that.

## 21c: several Renode processes

Each process is one shared Renode holding a group of drones; the groups
boot in parallel and the flights start when every drone is armable. Drones
keep their sysid-based ports. "P" = Renode on the P-cores, sidecars on the
E-cores.

| Split | Launch to all landed (s) | Armable (s) | Arm -> 50 m (s) | RSS in flight (MB) | RTF flying | Renode CPU (cores) |
|---|---|---|---|---|---|---|
| 6 drones, 1 x 6 | 894, 900 | 294-297 | 164-166 | 2812-2859 | 0.152 | 5.3 |
| 6 drones, 1 x 6, P | 960 | 302-304 | 181-182 | 2751 | 0.139 | 5.5 |
| 6 drones, 2 x 3 | 808 | 262-267 | 148-150 | 4931 | 0.169 | 5.9 |
| 6 drones, 2 x 3, P | 771 | 249-253 | 138-142 | 4942 | 0.180 | 6.1 |
| 6 drones, 3 x 2 | **741** | 243-246 | 134-136 | 6558 | 0.186 | 6.1 |
| 6 drones, 3 x 2, P | skipped (memory) | | | about 7200 est. | | |
| 8 drones, 1 x 8 | 1094 | 354-357 | 202 | 2974 | 0.125 | 6.6 |
| 8 drones, 1 x 8, P | 1146 | 372-375 | 211-212 | 3056 | 0.119 | 6.7 |
| 8 drones, 2 x 4 | **962** | 313-316 | 177-178 | 5147 | 0.142 | 7.5 |
| 8 drones, 2 x 4, P | 1000 | 319-328 | 179-185 | 5128 | 0.139 | 7.5 |
| 8 drones, 4 x 2 | skipped (memory: about 9.6 GB) | | | | | |
| 8 drones, 8 x 1 | not run: about 17.6 GB | | | | | |

- Splitting helps because a slow thread now only holds back its own group,
  and more of the 10 cores get used (7.5 cores for 2 x 4 against 6.6 for
  1 x 8).
- It does not come close to "half the time": the machine has 6 fast cores
  for 8 machine threads however they are grouped.
- All 8 drones completed in every 8-drone run, own sysids, no failsafes.
- Run-to-run spread: 1 x 6 today's settings 894 and 900 s (0.7 %); the
  4-drone boot 232-236 s over 5 runs. The splits were run once each: the
  memory guard skipped them on the first evening (other apps had left about
  5.6 GB) and there was no room for repeats the next day either.

## 21d: boot snapshot probe

Question: can Renode's `Save` / `Load` skip the 150-380 s boot?
**No, not without changing peripheral sources** (which are off-limits).

1. One drone booted to armable in a shared Renode (147 s), `pause`, then
   `Save @file`: refused after 3 s, no file.
   `Error encountered during saving: Pointer or ThreadLocal or SpinLock encountered during serialization.`
2. With Renode's detailed serializer (`serialization-mode = Reflection` in
   a private config file) the path is:
   `Emulation => Action => CortexM => Machine => BaseClockSource => List => ClockEntry => AP_Physics => TcpClient => Socket => SafeSocketHandle => IntPtr`
   - the physics peripheral's live TCP connection to its sidecar.
3. `physics Disconnect` through the monitor first (allowed, no source
   change): the next blocker appears:
   `Emulation => BackendManager => ... => STM32F7_USART => DoubleWordRegisterCollection => ... => Action => <>c__DisplayClass0_1 => RuntimeMethodInfo => IntPtr`
   - a register hook on the UARTs that captures a reflection handle. It
   comes from the UART helper peripheral's source and cannot be removed
   from outside.

Stopped there (about 25 minutes of the 90). Even past these two there
would be more of the same (the other reflection-based helpers), and after a
`Load` the physics sidecar would start from its initial state while the
firmware believes it has been flying its EKF for two minutes.

## Proposals not tested (they change fidelity)

- `cpu PerformanceInMips` below 300: the emulated CPU would do fewer
  instructions per virtual second, so each virtual second would cost less
  host time. It changes the firmware's timing margins.
- A lower physics rate than 1200 Hz.
- Both would raise the single-thread ceiling, which is the only thing that
  would make every configuration faster.

## What integrating this would take

- A "drones per Renode" choice for the shared mode (all in one, 4, 3 or
  2), ideally with a memory check that refuses a split that will not fit.
- `services/fleet_mission.py`: several `SharedRenodeFleet` objects instead
  of one, booted in parallel; the watchdog per group (a Renode that dies
  fails its own group only, which also softens the single point of
  failure); stop and wrap-up over all of them.
- `engine/shared_renode.py`: a per-group work directory (today it is the
  fixed `renode_instances/shared/`), and optionally a CPU set for Renode
  at launch.
- P-core pinning for fleets of up to 4: detect the P-cores from
  `/sys/devices/cpu_core/cpus` (absent on non-hybrid CPUs, where nothing
  should be pinned).
- The experiment's `SpeedFleet` subclass in `speed_run.py` already does the
  per-group directory and the affinity; it is about 40 lines.

## All runs

| Run | Drones x Renodes | Settings | OK | GPS fix (s) | Armable (s) | Arm->50 m (s) | Flight (s) | Launch to all landed (s) | RSS after boot (MB) | RSS airborne (MB) | RSS end (MB) | Renode CPU (cores) | Sidecar CPU (cores) | RTF boot mean / min | RTF flight mean / min | Min MemAvailable (MB) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| a1 | 1 x 1 | boot only | yes | 51 | 155 | - | - | - | 2133 | - | - | 0.93 | 0.02 | 0.321 / 0.317 | - | 8631 |
| b1p | 1 x 1 | Renode on P-cores, boot only | yes | 47 | 144 | - | - | - | 2051 | - | - | 0.88 | 0.01 | 0.350 / 0.331 | - | 8710 |
| a4 | 4 x 1 | today | yes | 73 | 232-233 | 128 | 470 | 702.7 | 2483 | 2596 | 2744 | 3.92 | 0.04 | 0.215 / 0.200 | 0.195 / 0.190 | 7988 |
| b4ai | 4 x 1 | advance immediately, boot only | yes | 81 | 247-248 | - | - | - | 2369 | - | - | 3.36 | 0.04 | 0.213 / 0.170 | - | 4019 |
| b4base | 4 x 1 | boot only | yes | 74 | 236 | - | - | - | 2429 | - | - | 3.35 | 0.04 | 0.213 / 0.198 | - | 4602 |
| b4e | 4 x 1 | Renode on E-cores, boot only | yes | 124 | 396-398 | - | - | - | 2450 | - | - | 3.44 | 0.04 | 0.146 / 0.117 | - | 8347 |
| b4gc | 4 x 1 | GC default, boot only | yes | 74 | 235-236 | - | - | - | 3740 | - | - | 3.34 | 0.04 | 0.217 / 0.199 | - | 3211 |
| b4mq02 | 4 x 1 | master quantum 0.2, boot only | yes | 75 | 235-237 | - | - | - | 2284 | - | - | 3.34 | 0.04 | 0.213 / 0.200 | - | 4760 |
| b4p | 4 x 1 | Renode on P-cores, boot only | yes | 69 | 219-220 | - | - | - | 2389 | - | - | 3.4 | 0.04 | 0.224 / 0.215 | - | 8378 |
| b4p1 | 4 x 1 | Renode on one CPU per P-core, boot only | yes | 71 | 216-218 | - | - | - | 2332 | - | - | 3.32 | 0.04 | 0.238 / 0.221 | - | 8449 |
| b4q02 | 4 x 1 | machine quantum 0.02, boot only | yes | 76 | 236-238 | - | - | - | 2332 | - | - | 3.3 | 0.04 | 0.216 / 0.198 | - | 4537 |
| b4q05 | 4 x 1 | machine quantum 0.05, boot only | yes | 77 | 239-241 | - | - | - | 2393 | - | - | 3.28 | 0.04 | 0.220 / 0.196 | - | 4626 |
| c4_even_sideodd | 4 x 1 | Renode on CPUs 0,2,4,6,8,10, sidecars on CPUs 1,3,5,7,9,11, boot only | yes | 70 | 222 | - | - | - | 2327 | - | - | 3.3 | 0.04 | 0.221 / 0.213 | - | 4494 |
| c4_p_sideP | 4 x 1 | Renode on P-cores, sidecars on P-cores, boot only | yes | 71 | 224 | - | - | - | 2388 | - | - | 3.4 | 0.04 | 0.222 / 0.211 | - | 4537 |
| c4_p_sidefree | 4 x 1 | Renode on P-cores, boot only | yes | 69 | 220 | - | - | - | 2472 | - | - | 3.42 | 0.04 | 0.221 / 0.213 | - | 4417 |
| d4_1x4_p | 4 x 1 | Renode on P-cores, sidecars on E-cores | yes | 69 | 217 | 120 | 431-436 | 652.9 | 2452 | 2568 | 2705 | 4.01 | 0.07 | 0.227 / 0.218 | 0.210 / 0.207 | 4197 |
| unpinned_repeat_c4_even_sideodd | 4 x 1 | boot only | yes | 74 | 235 | - | - | - | 2465 | - | - | 3.36 | 0.04 | 0.218 / 0.200 | - | 4451 |
| unpinned_repeat_c4_p_sideP | 4 x 1 | boot only | yes | 74 | 233-235 | - | - | - | 2485 | - | - | 3.36 | 0.04 | 0.220 / 0.202 | - | 4427 |
| unpinned_repeat_c4_p_sidefree | 4 x 1 | boot only | yes | 73 | 231-232 | - | - | - | 2363 | - | - | 3.35 | 0.04 | 0.216 / 0.205 | - | 4660 |
| a6 | 6 x 1 | today | yes | 92 | 295-297 | 165-166 | 596-603 | 899.5 | 2641 | 2812 | 3036 | 5.24 | 0.06 | 0.184 / 0.156 | 0.152 / 0.150 | 7747 |
| b6p | 6 x 1 | Renode on P-cores, boot only | yes | 85 | 278-280 | - | - | - | 2641 | - | - | 4.87 | 0.06 | 0.191 / 0.165 | - | 8094 |
| b6p1 | 6 x 1 | Renode on one CPU per P-core, boot only | yes | 86 | 287-290 | - | - | - | 2501 | - | - | 4.31 | 0.06 | 0.187 / 0.140 | - | 4277 |
| d6_1x6 | 6 x 1 | today | yes | 92 | 294-295 | 164 | 592-599 | 893.9 | 2693 | 2859 | 3062 | 5.28 | 0.06 | 0.185 / 0.156 | 0.153 / 0.151 | 5352 |
| d6_1x6_p | 6 x 1 | Renode on P-cores, sidecars on E-cores | yes | 88 | 302-304 | 181-182 | 644-657 | 960.4 | 2577 | 2751 | 2963 | 5.47 | 0.06 | 0.174 / 0.142 | 0.139 / 0.122 | 3241 |
| oct8_lowmem_d6_1x6_p | 6 x 1 | Renode on P-cores, sidecars on E-cores | yes | 90 | 295-298 | 172-174 | 680-688 | 986.4 | 2643 | 2816 | 3029 | 5.43 | 0.06 | 0.185 / 0.138 | 0.134 / 0.117 | 2705 |
| d6_2x3 | 6 x 2 (3+3) | today | yes | 81-82 | 262-267 | 148-150 | 533-541 | 808.1 | 4751 (2382/2369) | 4931 (2482/2449) | 5154 (2596/2558) | 5.88 | 0.06 | 0.197 / 0.175 | 0.169 / 0.162 | 3454 |
| d6_2x3_p | 6 x 2 (3+3) | Renode on P-cores, sidecars on E-cores | yes | 79 | 249-253 | 138-142 | 497-518 | 771.2 | 4745 (2422/2323) | 4942 (2521/2421) | 5165 (2637/2528) | 6.06 | 0.06 | 0.211 / 0.185 | 0.180 / 0.175 | 3472 |
| d6_3x2 | 6 x 3 (2+2+2) | today | yes | 76-77 | 243-246 | 134-136 | 486-496 | 741.3 | 6416 (2086/2163/2167) | 6558 (2136/2202/2220) | 6696 (2182/2241/2273) | 6.14 | 0.06 | 0.219 / 0.181 | 0.186 / 0.178 | 1573 |
| d6_3x2_p | 6 x 3 (2+2+2) | Renode on P-cores | SKIPPED: would leave 1310 MB available (estimate 7219 MB) | | | | | | | | | | | | |
| d8_1x8 | 8 x 1 | today | yes | 110 | 354-357 | 202 | 729-736 | 1093.7 | 2751 | 2974 | 3245 | 6.58 | 0.08 | 0.158 / 0.127 | 0.125 / 0.121 | 5133 |
| d8_1x8_p | 8 x 1 | Renode on P-cores, sidecars on E-cores | yes | 114-115 | 372-375 | 211-212 | 762-770 | 1145.9 | 2832 | 3056 | 3336 | 6.71 | 0.08 | 0.149 / 0.121 | 0.119 / 0.118 | 5188 |
| d8_2x4 | 8 x 2 (4+4) | today | yes | 96-99 | 313-316 | 177-178 | 639-646 | 962.2 | 4917 (2442/2475) | 5147 (2554/2593) | 5431 (2692/2739) | 7.49 | 0.08 | 0.172 / 0.137 | 0.142 / 0.139 | 3355 |
| d8_2x4_p | 8 x 2 (4+4) | Renode on P-cores, sidecars on E-cores | yes | 101-102 | 319-328 | 179-185 | 643-672 | 1000.4 | 4898 (2496/2402) | 5128 (2609/2519) | 5427 (2756/2671) | 7.52 | 0.08 | 0.172 / 0.135 | 0.139 / 0.135 | 3292 |
| d8_4x2 | 8 x 4 (2+2+2+2) | today | SKIPPED: would leave -3227 MB available (estimate 9625 MB) | | | | | | | | | | | | |

"Boot only" runs stop 35 s after every drone is armable. `oct8_lowmem_*`
ran with only 2.7 GB of memory free. RSS in brackets is per process.

## Files

- `speed_run.py` - the runner (groups, knobs, sampling). `run.sh <tag> ...`
  wraps it with cleanup and a capped log.
- `snapshot_probe.py` - the 21d probe.
- `make_table.py` - the table above from `out/*.json`.
- `out/` (not committed) - every run's log and JSON.
