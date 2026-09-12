"""Launches a pre-built, standalone Renode + Pixhawk6C/6X package as a
managed subprocess and exposes the resulting MAVLink connection string.

This deliberately does NOT depend on an ArduPilot checkout, a build step,
or any patches being applied at runtime - all of that was done once,
already, to produce the standalone folder this class launches. See
renode_firmware_guide.md ("Part 4 - Freezing a Working Result Into a
Standalone Copy") for how that folder was produced and what it contains.

Treats Renode exactly like engine/renode_backend.py (an earlier, now-
obsolete prototype in this project's history) intended to: an external
process this app starts and talks to over MAVLink, nothing more - the
same pattern already used for real SITL and mock_sitl.py via
services/mavlink_flight_service.py's plain connection_string interface.

Physics and GPS are not optional extras here. Without them the emulated
vehicle never gets a real gyro/compass/GPS fix and never arms - that is the
broken state this whole thing exists to avoid, not a legitimate lighter
boot mode - so `start()` always brings both up as part of a normal launch.

Two independently-confirmed findings (both verified by hand against a real
boot, not inferred) drive the sequencing and constants below:

1. Wiring the GPS peripheral to its UART (and, it turns out, connecting
   physics at all) must happen BEFORE the machine's `start` runs, not after
   on an already-running machine. Attaching it live to a running machine
   produces a clean boot with no errors, yet GPS_RAW_INT.fix_type never
   leaves 0 - the same class of problem already seen once with the SD card.
   So the launch command below replicates launch.resc's own content up to
   (not including) its trailing `start`, inserts `physics Connect`, then the
   GPS hub wiring, and only then issues `start` itself, last.
2. The physics sidecar (renode-physics) is a TCP *server* - Renode's own
   `physics` peripheral dials out to it as the client - so the sidecar must
   already be listening before Renode's `physics Connect` runs. Renode
   itself is otherwise silent about the handshake at normal log verbosity;
   the sidecar's own stdout `PHYSICS_PORT <port>` line is the only reliable
   readiness signal.
"""
from __future__ import annotations

import os
import select
import signal
import socket
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2


class RenodeLauncherError(Exception):
    pass


class RenodeLauncher:
    # Confirmed, not placeholder, values - see the module docstring and
    # renode_firmware_guide.md Part 4.4 for how each was pinned down:
    #   - lat/lon/alt/heading: AP_PhysicsTruth.Stationary's own defaults
    #     (peripherals/common/AP_Physics.cs), matching launch.py's own
    #     defaults and the "Home: ..." line renode-physics prints back.
    #   - rate_hz=1200: the "quad" SITL model's native rate - the `Aircraft`
    #     base class default in libraries/SITL/SIM_Aircraft.h, which
    #     SIM_Multicopter (what frame name "quad" selects) does not
    #     override. Requesting anything above this is a real, documented
    #     rejection case in renode_physics.cpp's CONFIGURE handling.
    #   - GPS_UART_HOST="usart1Host": Pixhawk6C's hwdef.dat maps GPS1 to
    #     USART1, and gen_board.py appends "Host" for the h743 family - the
    #     same derivation extract_pixhawk6x_renode_standalone.sh's
    #     apply_gps_patch() performs from source. Hardcoded here (rather
    #     than re-derived from hwdef.dat at runtime) because the frozen
    #     standalone folder carries no hwdef.dat to derive it from - only
    #     Tools/renode, firmware, and the already-generated board files.
    PHYSICS_PORT = 9002
    PHYSICS_MODEL = "quad"
    PHYSICS_LATITUDE_DEG = -35.363261
    PHYSICS_LONGITUDE_DEG = 149.165230
    PHYSICS_ALTITUDE_M = 584.0
    PHYSICS_HEADING_DEG = 353.0
    PHYSICS_RATE_HZ = 1200
    GPS_UART_HOST = "usart1Host"
    GPS_MIN_FIX_TYPE = 3  # mavutil GPS_FIX_TYPE_3D_FIX

    # Known first-boot provisioning, all confirmed from source (not
    # guessed), all reapplied live once per fresh boot via
    # `provision_first_boot_params()` (a separate call from `start()` -
    # see its own docstring for why they're split). Deliberately NOT
    # followed by a reboot: a mid-session firmware reboot has been
    # observed to leave the GPS peripheral stuck renegotiating
    # configuration indefinitely (renode_firmware_guide.md Part 4.6) -
    # these are set in the same session the vehicle already booted
    # healthy in.
    #
    # FRAME_CLASS/FRAME_TYPE: fresh firmware boots with these unset,
    # which blocks arming outright ("PreArm: Motors: Check frame class
    # and type") independent of physics/GPS health. Confirmed correct
    # for this board (both AP_Int8, see ArduCopter/Parameters.h). NOT
    # persisted firmware-side - baking them into defaults.parm was
    # attempted and is blocked by a real ArduPilot build-system gap
    # (embedded_defaults.find() returning falsy - "No param area found";
    # see Part 4.6).
    FRAME_CLASS = 1  # Quad
    FRAME_TYPE = 1  # X

    # COMPASS_USE/COMPASS_ENABLE: left at their real ArduPilot defaults (1).
    # A prior version of this class force-disabled both, on the belief that
    # gen_board.py's generated board genuinely had no compass hardware to
    # model at all (true - it has no auto-generation code path for an
    # external I2C compass like Pixhawk6C's real IST8310/RM3100). That
    # workaround was wrong, not just incomplete: AP_Arming's own
    # compass_checks() is happily bypassed by COMPASS_USE=0, but
    # EK3_SRC1_YAW still defaults to COMPASS (AP_NavEKF_Source.cpp:60), and
    # NavEKF3_core::readyToUseGPS() (AP_NavEKF3_Control.cpp:599) requires
    # yawAlignComplete before the EKF will EVER produce a position estimate
    # (not even the disarmed-lenient PRED_HORIZ_POS_ABS case - both flags
    # are computed identically, AP_NavEKF3_Control.cpp:794,799). Per
    # AP_NavEKF3.cpp:277's own EK3_MAG_CAL description, going compass-less
    # only realigns yaw via GPS-velocity GSF "when flight commences and
    # there is sufficient movement" - i.e. never while grounded and
    # stationary, which is exactly when arming's own position check runs.
    # Confirmed by direct test: with COMPASS_USE=0 (and even COMPASS_ENABLE=0
    # + a persisted reboot to apply it), "PreArm: Need Position Estimate"
    # never cleared across a 250+s window. A compass-less copter genuinely
    # cannot arm - this isn't a Renode gap, it's how EKF3 works.
    #
    # The actual fix: this harness already ships a complete, physics-driven
    # IST8310 peripheral (peripherals/sensors/AP_IST8310.cs, reading real
    # physics.Current.MagneticFieldBodyMgauss - already provided by the
    # physics sidecar) that gen_board.py simply never wires in for boards
    # whose real compass is external I2C rather than IMU-embedded. It's
    # manually added to the generated Pixhawk6C.repl (compass0 @ i2c4 0x0C,
    # matching hwdef.dat's real "COMPASS IST8310 I2C:0:0x0C ROTATION_NONE"),
    # the same class of hand-added fix as imu2Accel/imu2Gyro for the BMI055
    # above it in that file. With a real compass now present, COMPASS_USE/
    # COMPASS_ENABLE need no override at all.

    # "Compass not calibrated": Compass::configured() (AP_Compass.cpp:2078)
    # requires, for every compass instance actually used for yaw, BOTH a
    # non-zero saved offset (COMPASS_OFS_*, same "exactly 0.0 is
    # unconfigured" logic as the accel case below) AND a saved COMPASS_DEV_ID
    # matching the live-detected device id (AP_Compass.cpp:2111 - the same
    # register-time id_ok gate as accel/gyro, set in RAM at detection but
    # only persisted by a real calibration completing). The real onboard 3D
    # compass calibration needs physical rotation through many orientations
    # (infeasible here, same reason full 3D accel cal was skipped), so this
    # uses the two narrower real commands instead: a direct COMPASS_OFS_*
    # param_set for the offset requirement, then MAV_CMD_PREFLIGHT_CALIBRATION
    # with param2 = PREFLIGHT_CALIBRATION_MAGNETOMETER_FORCE_SAVE (76,
    # common.xml) -> Compass::force_save_calibration() (AP_Compass.cpp:2266),
    # which persists the already-detected dev_id - the compass-specific
    # equivalent of the accel case's simple_accel_cal(), documented for
    # exactly this "re-validate an existing/known-good reading" situation.
    COMPASS_OFFSET = 0.01
    COMPASS_CAL_TIMEOUT_S = 15.0
    PREFLIGHT_CALIBRATION_MAGNETOMETER_FORCE_SAVE = 76

    # "3D Accel calibration needed": AP_InertialSensor::accel_calibrated_ok_all()
    # (AP_InertialSensor.cpp ~line 1672) requires, for EVERY registered accel
    # instance, that `_accel_id_ok[i]` be true AND offset/scale be non-zero.
    # `_accel_id_ok[i]` is only ever set true by register_accel() finding a
    # previously-SAVED INS_ACC*_ID matching the live-detected device id (never
    # true on a truly first boot, since nothing has been saved yet), or by a
    # real calibration routine running and setting it directly. Directly
    # param-setting INS_ACCOFFS_*/INS_ACC2OFFS_* to a non-zero value was tried
    # first and does NOT work: it never touches `_accel_id_ok`, so the check
    # still fails, and fixing it that way would need a reboot after saving
    # INS_ACC*_ID - which reopens the GPS-stuck-on-reboot issue (Part 4.6).
    # The real fix is MAV_CMD_PREFLIGHT_CALIBRATION with param5 =
    # PREFLIGHT_CALIBRATION_ACCELEROMETER_SIMPLE (4), which calls
    # AP_InertialSensor::simple_accel_cal() (GCS_Common.cpp:4978) - a
    # single-orientation calibration that only needs the vehicle level and
    # stationary (true here: physics starts it in a level hover) and, on
    # success, sets offset/scale/id AND `_accel_id_ok[k] = true` directly, live,
    # with no reboot (AP_InertialSensor.cpp ~line 2682-2692).
    ACCEL_CAL_TIMEOUT_S = 30.0

    # ARMING_SKIPCHK: "RC not found" (ArduCopter/AP_Arming_Copter.cpp:114,
    # rc_throttle_failsafe_checks() - true when
    # !rc().has_had_rc_receiver() && !rc().has_had_rc_override()) and
    # "Safety Switch" (AP_Arming.cpp:812, hardware_safety_check() - true
    # when hal.util->safety_switch_state() == SAFETY_DISARMED) both require
    # real hardware this generated board has no peripheral for at all -
    # confirmed by grepping the generated Pixhawk6C.repl, which declares
    # imu0/imu2Accel/imu2Gyro/gps/fram and nothing RC- or safety-switch-
    # related; gen_board.py has no code path that ever emits either. This is
    # the same class of gap as COMPASS_USE above (real absent hardware, not
    # a timing or config issue), and ARMING_SKIPCHK (formerly ARMING_CHECK -
    # AP_Arming.cpp:205, AP_Int32, default 0/"skip nothing") is ArduPilot's
    # own real mechanism for exactly this: disabling checks for equipment a
    # given vehicle doesn't carry. RC = Check::RC = 1<<6 = 64, SWITCH =
    # Check::SWITCH = 1<<11 = 2048 (AP_Arming.h).
    #
    # GPS_CONFIG (1<<12 = 4096) is added for a different, narrower reason:
    # "GPS %d still configuring this GPS" (AP_Arming.cpp:780) comes from
    # AP_GPS::first_unconfigured_gps() -> driver->is_configured(), which for
    # AP_GPS_UBLOX tracks whether specific UBX config messages it sent have
    # each been ack'd. The emulated GPS here already proves a real, healthy
    # 3D fix (GPS_RAW_INT.fix_type, confirmed in start()) - this is not a
    # "no fix" problem, only the UBX config-ack handshake never completing
    # against this peripheral's emulation, a known incomplete-emulation gap
    # (renode_firmware_guide.md Part 4.6), not a data-integrity concern.
    # RC = 64, SWITCH = 2048, GPS_CONFIG = 4096 -> 6208.
    #
    # ARMING_SKIPCHK's SWITCH bit only bypasses the generic
    # AP_Arming::hardware_safety_check() (AP_Arming.cpp:808). ArduCopter's
    # own AP_Arming_Copter::arm_checks() (AP_Arming_Copter.cpp:641) has a
    # SECOND, separate, unconditional safety-switch gate that calls
    # check_failed(true, "Safety Switch") - the bool-only overload, not the
    # Check::SWITCH-typed one - meaning it is NOT gated by checks_to_skip at
    # all and stays live at actual arm time no matter what ARMING_SKIPCHK
    # is set to (confirmed by direct test: it disappeared from "PreArm:"
    # output once ARMING_SKIPCHK included SWITCH, but still failed the real
    # "Arm:"-time attempt). BRD_SAFETY_DEFLT (AP_BoardConfig.cpp:237) is the
    # real param for a compass-less-style default-state fix, but it's
    # @RebootRequired: True, reopening the reboot/GPS-stuck problem. The
    # real no-reboot fix is the live MAV_CMD_DO_SET_SAFETY_SWITCH_STATE
    # (5300) command - hal.rcout->force_safety_off() (GCS_Common.cpp:5643-
    # 5658) - genuinely the same call a real safety-switch press makes.
    ARMING_SKIPCHK = 64 | 2048 | 4096

    # FS_THR_ENABLE: real run evidence - the vehicle genuinely armed this
    # time (first real arm of the night) and then immediately printed
    # "Radio Failsafe - Disarming" and disarmed again. This is
    # ArduCopter/radio.cpp's own throttle/radio failsafe monitor
    # (read_radio() -> failsafe_radio_on_event(), events.cpp), a
    # continuously-running IN-FLIGHT safety check - separate from, and not
    # affected by, ARMING_SKIPCHK's RC bit (which only covers the PreArm/
    # Arm-time "RC not found" gate, not this ongoing monitor). Since this
    # board has no RC receiver peripheral at all (same absent-hardware class
    # as ARMING_SKIPCHK's RC/SWITCH bits above), FS_THR_ENABLE=0 (DISABLED,
    # ArduCopter/Parameters.h's FS_THR_Action enum - default is
    # ALWAYS_RTL=1) is the real ArduPilot mechanism for a vehicle that
    # genuinely has no throttle failsafe input to monitor. Not
    # @RebootRequired (Parameters.cpp:130) - takes effect live.
    FS_THR_ENABLE = 0

    # AUTO_OPTIONS: real run evidence - the vehicle genuinely armed AND
    # entered AUTO mode AND started the takeoff command, but motors never
    # spun up past MOT_SPIN_ARM (AP_MotorsMulticopter.h's
    # AP_MOTORS_SPIN_ARM_DEFAULT = 0.10 -> exactly PWM 1100 with
    # MOT_PWM_MIN/MAX defaults of 1000/2000 - confirmed via live
    # SERVO_OUTPUT_RAW telemetry matching that value exactly, ruling out a
    # physics-actuator-forwarding bug: the flight controller itself never
    # asked for more). Root cause traced to Copter::update_auto_armed()
    # (system.cpp:309): for a normal multirotor (not using_interlock),
    # `ap.auto_armed` - which gates the spool-state progression past
    # GROUND_IDLE (takeoff.cpp:124) - only becomes true once
    # `!ap.throttle_zero`, i.e. a real RC throttle stick raise. This board
    # has no RC receiver at all (same absent-hardware class as
    # ARMING_SKIPCHK's RC bit above), so throttle_zero never clears and
    # auto_armed never sets through that path. ArduCopter has a real,
    # documented mechanism for exactly this GCS-driven/no-RC scenario:
    # ModeAuto::takeoff_run() (mode_auto.cpp:1115) directly calls
    # set_auto_armed(true) when AUTO_OPTIONS bit 1 ("Allow Takeoff Without
    # Raising Throttle", Parameters.cpp:806-810) is set. Not
    # @RebootRequired - takes effect live.
    AUTO_OPTIONS = 2

    def __init__(self, standalone_dir: str, port: int = 5762):
        self.standalone_dir = Path(standalone_dir).expanduser().resolve()
        self.port = port

        self.renode_bin = self.standalone_dir / "renode-bin" / "renode"
        self.launch_script = self.standalone_dir / "launch.resc"
        self.physics_bin = self.standalone_dir / "renode-physics"

        if not self.renode_bin.is_file():
            raise RenodeLauncherError(
                f"No renode executable at {self.renode_bin} - is this a real "
                "standalone extraction? See renode_firmware_guide.md Part 4."
            )
        if not self.launch_script.is_file():
            raise RenodeLauncherError(f"No launch.resc at {self.launch_script}")
        if not self.physics_bin.is_file():
            raise RenodeLauncherError(
                f"No renode-physics executable at {self.physics_bin} - the "
                "standalone extraction is missing the physics sidecar. See "
                "renode_firmware_guide.md Part 4.4."
            )

        self._proc: subprocess.Popen | None = None
        self._physics_proc: subprocess.Popen | None = None
        self._renode_log = None
        self._renode_log_path: Path | None = None

    @property
    def connection_string(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(
        self,
        physics_ready_timeout_s: float = 30.0,
        gps_ready_timeout_s: float = 150.0,
    ) -> str:
        """Two-stage start: renode-physics first, confirmed listening, then
        Renode itself with physics and the GPS UART wired in before `start`
        runs. Only returns once a real MAVLink heartbeat has been seen AND
        GPS_RAW_INT.fix_type has reached a 3D fix - a clean boot with no
        errors is NOT sufficient evidence on its own (confirmed the hard
        way: that exact combination happened once tonight while GPS stayed
        dead the whole time).

        This does NOT provision first-boot params (FRAME_CLASS/FRAME_TYPE/
        COMPASS_USE) - call `provision_first_boot_params()` afterward for
        that, on its own short-lived connection, closed before the real
        mission connection opens. Frame params used to be folded into this
        same connection/method, but that was undone: a control test
        (disabling only that step, changing nothing else) showed the
        vehicle can still fail to reach healthy gyro state even without
        it, proving the combined step was never actually the cause of that
        failure - the gyro-health cause is still open (see
        renode_firmware_guide.md Part 4.6) and isn't chased further here.
        Splitting the two keeps `start()` itself to the one thing it has
        real, positive evidence for (GPS-fix boot health), rather than
        bundling in a step whose own connection-handling was never
        actually implicated.

        `gps_ready_timeout_s` covers Renode's own boot, the MAVLink
        handshake, and GPS settling once connected - real data from tonight
        put the fix at ~18.6s after the first heartbeat, so the default
        carries generous margin rather than a tight bound.
        """
        if self.is_running:
            raise RenodeLauncherError("already running - call stop() first")

        self._kill_any_stale_processes()

        try:
            self._start_physics(physics_ready_timeout_s)
            self._start_renode()
            self._wait_for_gps_fix(gps_ready_timeout_s)
        except Exception:
            self.stop()
            raise

        return self.connection_string

    # ---- Stage 1: physics sidecar ----

    def _start_physics(self, ready_timeout_s: float) -> None:
        # New process group, same as Renode below, so stop() can reliably
        # reach every child - not just this one PID.
        self._physics_proc = subprocess.Popen(
            [str(self.physics_bin), "--model", self.PHYSICS_MODEL,
             "--physics-port", str(self.PHYSICS_PORT)],
            cwd=self.standalone_dir,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self._wait_for_physics_ready(ready_timeout_s)

    def _wait_for_physics_ready(self, timeout_s: float) -> None:
        """Reads the sidecar's real stdout for its own `PHYSICS_PORT <port>`
        readiness line - not a generic port-open probe. Deliberately never
        connects to the port itself to check it: the sidecar's socket
        serves exactly one client, and that client must be Renode's own
        `physics` peripheral, not a readiness probe."""
        proc = self._physics_proc
        deadline = time.monotonic() + timeout_s
        buffer = b""
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RenodeLauncherError(
                    f"renode-physics exited early (code {proc.returncode}): "
                    f"{buffer.decode(errors='replace')}"
                )
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([proc.stdout], [], [], min(0.5, remaining))
            if ready:
                chunk = os.read(proc.stdout.fileno(), 4096)
                if not chunk:
                    raise RenodeLauncherError(
                        "renode-physics closed stdout before printing PHYSICS_PORT"
                    )
                buffer += chunk
                if b"PHYSICS_PORT" in buffer:
                    return
        raise TimeoutError(
            f"renode-physics never printed PHYSICS_PORT within {timeout_s}s"
        )

    # ---- Stage 2: Renode itself ----

    def _start_renode(self) -> None:
        command = self._build_launch_command()
        # Captured to a file, not DEVNULL: an unhandled exception inside a
        # Renode peripheral crashes the whole process with no other signal
        # than an early exit code - this is the only way to see why.
        self._renode_log_path = self.standalone_dir / "renode-console.log"
        self._renode_log = open(self._renode_log_path, "wb")
        self._proc = subprocess.Popen(
            [str(self.renode_bin), "--disable-xwt", "--console", "-e", command],
            cwd=self.standalone_dir,
            preexec_fn=os.setsid,
            stdout=self._renode_log,
            stderr=subprocess.STDOUT,
        )

    def _build_launch_command(self) -> str:
        """launch.resc's own content, up to (not including) its trailing
        `start`, with physics Connect and the GPS UART wiring inserted
        before a final `start` we issue ourselves - see the module
        docstring for why this order is load-bearing, confirmed by hand."""
        lines = [
            line for line in self.launch_script.read_text().splitlines()
            if line.strip()
        ]
        if not lines or lines[-1].strip() != "start":
            raise RenodeLauncherError(
                f"{self.launch_script} does not end with a bare 'start' line "
                "as expected - cannot safely reorder it to wire physics/GPS "
                "before boot without risking a wrong assumption about its "
                "structure"
            )
        boot_commands = lines[:-1]

        physics_connect = 'physics Connect %d "%s" %.6f %.6f %.1f %.1f %d' % (
            self.PHYSICS_PORT, self.PHYSICS_MODEL,
            self.PHYSICS_LATITUDE_DEG, self.PHYSICS_LONGITUDE_DEG,
            self.PHYSICS_ALTITUDE_M, self.PHYSICS_HEADING_DEG,
            self.PHYSICS_RATE_HZ,
        )
        gps_wiring = [
            'emulation CreateUARTHub "gpsHub"',
            'connector Connect sysbus.%s gpsHub' % self.GPS_UART_HOST,
            'connector Connect sysbus.gps gpsHub',
        ]
        return "; ".join([*boot_commands, physics_connect, *gps_wiring, "start"])

    def _connect_mavlink(self, deadline: float):
        """`mavutil.mavlink_connection` connects eagerly for a tcp: URL and
        raises immediately if nothing is listening yet - which is normal
        for several seconds while Renode is still booting. Retries until
        something is actually listening, rather than treating an early
        connection refusal as failure."""
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                raise RenodeLauncherError(
                    f"renode exited early (code {self._proc.returncode})"
                )
            try:
                return mavutil.mavlink_connection(self.connection_string)
            except OSError as exc:
                last_error = exc
                time.sleep(0.5)
        raise TimeoutError(
            f"could not connect to {self.connection_string} within "
            f"{deadline - time.monotonic():.0f}s of the deadline: {last_error}"
        )

    def _wait_for_gps_fix(self, timeout_s: float) -> None:
        """A clean boot with no errors is not success on its own (confirmed
        the hard way). Real success is a MAVLink heartbeat followed by
        GPS_RAW_INT.fix_type reaching a 3D fix. The verification connection
        is closed again before returning - the FMU's serial terminal serves
        one client at a time, and the real client is whatever calls
        connection_string next (services.mavlink_flight_service)."""
        deadline = time.monotonic() + timeout_s
        connection = self._connect_mavlink(deadline)
        try:
            heartbeat_timeout = max(0.0, deadline - time.monotonic())
            heartbeat = connection.wait_heartbeat(timeout=heartbeat_timeout)
            if heartbeat is None:
                raise TimeoutError(
                    f"no MAVLink heartbeat on {self.connection_string} "
                    f"within {timeout_s}s"
                )
            # ArduPilot does not stream GPS_RAW_INT (or anything else)
            # unheard - a plain tcp: connection with no stream request never
            # sees one, which is exactly why the first automated run of this
            # method waited the full timeout with zero GPS_RAW_INT messages
            # despite the manual terminal tests (which always sent this)
            # reaching a fix in ~18.6s.
            connection.mav.request_data_stream_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
            )
            while time.monotonic() < deadline:
                if self._proc.poll() is not None:
                    raise RenodeLauncherError(
                        f"renode exited early (code {self._proc.returncode})"
                    )
                message = connection.recv_match(
                    type="GPS_RAW_INT", blocking=True, timeout=2
                )
                if message is not None and message.fix_type >= self.GPS_MIN_FIX_TYPE:
                    return
            raise TimeoutError(
                f"GPS_RAW_INT.fix_type never reached {self.GPS_MIN_FIX_TYPE} "
                f"within {timeout_s}s"
            )
        finally:
            connection.close()

    def provision_first_boot_params(self, timeout_s: float = 45.0) -> None:
        """FRAME_CLASS/FRAME_TYPE/COMPASS_USE (each confirmed via a real
        PARAM_VALUE ack) plus a real simple accelerometer calibration
        (confirmed via a real COMMAND_ACK) - not fire-and-assume for any of
        them. No reboot follows this - see the class-level comments above
        these constants for why.

        Call this AFTER `start()` returns, not as part of it: it opens its
        own short-lived connection and fully closes it again before
        returning, mirroring the only pattern proven safe tonight - one
        connection, do the one thing it's for, close it completely, then
        the next caller (a real mission flight, typically) connects fresh.
        Frame params used to run inside `start()`'s own connection;
        splitting them out was a documented fallback, not a fix for a
        diagnosed cause - see `start()`'s docstring.
        """
        if not self.is_running:
            raise RenodeLauncherError("not running - call start() first")
        deadline = time.monotonic() + timeout_s
        connection = self._connect_mavlink(deadline)
        try:
            heartbeat_timeout = max(0.0, deadline - time.monotonic())
            if connection.wait_heartbeat(timeout=heartbeat_timeout) is None:
                raise TimeoutError(
                    f"no MAVLink heartbeat on {self.connection_string} "
                    f"within {timeout_s}s"
                )
            self._confirm_param_set(
                connection, "FRAME_CLASS", self.FRAME_CLASS, mav2.MAV_PARAM_TYPE_INT8,
                max(1.0, deadline - time.monotonic()),
            )
            self._confirm_param_set(
                connection, "FRAME_TYPE", self.FRAME_TYPE, mav2.MAV_PARAM_TYPE_INT8,
                max(1.0, deadline - time.monotonic()),
            )
            self._confirm_param_set(
                connection, "ARMING_SKIPCHK", self.ARMING_SKIPCHK, mav2.MAV_PARAM_TYPE_INT32,
                max(1.0, deadline - time.monotonic()),
            )
            self._confirm_param_set(
                connection, "FS_THR_ENABLE", self.FS_THR_ENABLE, mav2.MAV_PARAM_TYPE_INT8,
                max(1.0, deadline - time.monotonic()),
            )
            self._confirm_param_set(
                connection, "AUTO_OPTIONS", self.AUTO_OPTIONS, mav2.MAV_PARAM_TYPE_INT32,
                max(1.0, deadline - time.monotonic()),
            )
            self._run_simple_accel_cal(connection, max(1.0, deadline - time.monotonic()))
            self._run_compass_force_save(connection, max(1.0, deadline - time.monotonic()))
            self._force_safety_off(connection, max(1.0, deadline - time.monotonic()))
        finally:
            connection.close()

    @staticmethod
    def _force_safety_off(connection, timeout_s: float) -> None:
        """MAV_CMD_DO_SET_SAFETY_SWITCH_STATE(DANGEROUS) - see the class-level
        comment above ARMING_SKIPCHK for why this, not ARMING_SKIPCHK's
        SWITCH bit, is the real fix for the ArduCopter-specific mandatory
        "Arm: Safety Switch" gate."""
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mav2.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, 0,
            mav2.SAFETY_SWITCH_STATE_DANGEROUS, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
            if message is None:
                continue
            if message.command != mav2.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE:
                continue
            if message.result != mav2.MAV_RESULT_ACCEPTED:
                raise RenodeLauncherError(
                    f"force safety off not accepted: result={message.result}"
                )
            return
        raise TimeoutError(f"no safety-off COMMAND_ACK within {timeout_s}s")

    def wait_until_armable(
        self, timeout_s: float = 120.0, stable_window_s: float = 5.0,
    ) -> None:
        """Wait for EKF_STATUS_REPORT.flags to hold EKF_POS_HORIZ_ABS
        continuously for `stable_window_s`, with no "PreArm:"/"Arm:"
        STATUSTEXT during that same window - i.e. genuinely, durably
        armable, not just observed passing at one instant.

        This exists because of a real, confirmed sequencing bug in engine/
        mavlink_mission.py (off-limits to edit per this project's own
        standing rule - the connection_string producer here doesn't touch
        the flight code that consumes it): upload_and_fly() sends
        MAV_CMD_COMPONENT_ARM_DISARM exactly ONCE, immediately after
        mission upload, with no retry - and the very next line reads
        "Arm ack: <the first COMMAND_ACK to arrive>", which is actually the
        ack for the PRECEDING MAV_CMD_DO_SET_MODE (command 176) call, not
        the arm command (command 400) at all. Since arming is attempted
        only once, an early rejection is final for the whole flight -
        position never changes for the rest of the run because the vehicle
        stays disarmed throughout.

        Two earlier designs were tried and both proved unsound on real
        runs, which is why this checks actual continuous vehicle state
        instead of inferring health from an absence of complaints:
        - A single rolling quiet-timer (return once N seconds passed with
          no failure text) can start counting before the vehicle-side
          check has even been evaluated once, declaring victory on pure
          timing luck.
        - Sending MAV_CMD_RUN_PREARM_CHECKS (401, side-effect-free -
          GCS_Common.cpp:5016 - always ACKs ACCEPTED and just triggers
          AP_Arming::pre_arm_checks(true)) repeatedly and requiring N
          consecutive clean responses is closer, but still only samples
          discrete instants: a real run passed 8 consecutive clean probes
          (20s) and the real arm attempt still failed moments later with
          "Need Position Estimate", because only one of the two EKF3
          cores had actually reached GPS-aided position by then - the
          probes simply hadn't caught the other core's lag.

        EKF_STATUS_REPORT.flags is what Copter::ekf_has_absolute_position()
        (system.cpp:223) itself is built from - EKF_POS_HORIZ_ABS is the
        live, continuously-updated version of the exact same
        status.flags.horiz_pos_abs bit (AP_NavEKF3_Control.cpp:794), not a
        one-shot announcement or an inference from silence. Requiring it
        held for a continuous stretch (not just seen once) is what
        actually matches "durably converged" rather than "passed an
        instant ago".
        """
        if not self.is_running:
            raise RenodeLauncherError("not running - call start() first")
        deadline = time.monotonic() + timeout_s
        connection = self._connect_mavlink(deadline)
        try:
            heartbeat_timeout = max(0.0, deadline - time.monotonic())
            if connection.wait_heartbeat(timeout=heartbeat_timeout) is None:
                raise TimeoutError(
                    f"no MAVLink heartbeat on {self.connection_string} "
                    f"within {timeout_s}s"
                )
            connection.mav.request_data_stream_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
            )
            last_bad_at = time.monotonic()
            seen_ekf_status = False
            while True:
                now = time.monotonic()
                if now >= deadline:
                    raise TimeoutError(
                        f"vehicle never held EKF_POS_HORIZ_ABS with no "
                        f"PreArm/Arm failures for {stable_window_s}s within "
                        f"{timeout_s}s (seen_ekf_status={seen_ekf_status})"
                    )
                if seen_ekf_status and now - last_bad_at >= stable_window_s:
                    return
                message = connection.recv_match(
                    type=["EKF_STATUS_REPORT", "STATUSTEXT"],
                    blocking=True, timeout=0.5,
                )
                if message is None:
                    continue
                if message.get_type() == "EKF_STATUS_REPORT":
                    seen_ekf_status = True
                    if not (message.flags & mav2.EKF_POS_HORIZ_ABS):
                        last_bad_at = time.monotonic()
                elif "PreArm:" in message.text or "Arm:" in message.text:
                    last_bad_at = time.monotonic()
        finally:
            connection.close()

    @staticmethod
    def _run_compass_force_save(connection, timeout_s: float) -> None:
        """COMPASS_OFS_* param_set (non-zero offset) + MAV_CMD_PREFLIGHT_
        CALIBRATION param2=FORCE_SAVE (persists the already-detected
        COMPASS_DEV_ID) - see the class-level comment above COMPASS_OFFSET
        for why this, not a full 3D onboard mag calibration, is the real fix
        for "Compass not calibrated"."""
        deadline = time.monotonic() + timeout_s
        for name in ("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z"):
            RenodeLauncher._confirm_param_set(
                connection, name, RenodeLauncher.COMPASS_OFFSET,
                mav2.MAV_PARAM_TYPE_REAL32, max(1.0, deadline - time.monotonic()),
            )
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mav2.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            0, RenodeLauncher.PREFLIGHT_CALIBRATION_MAGNETOMETER_FORCE_SAVE,
            0, 0, 0, 0, 0,
        )
        while time.monotonic() < deadline:
            message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
            if message is None:
                continue
            if message.command != mav2.MAV_CMD_PREFLIGHT_CALIBRATION:
                continue
            if message.result != mav2.MAV_RESULT_ACCEPTED:
                raise RenodeLauncherError(
                    f"compass force-save not accepted: result={message.result}"
                )
            return
        raise TimeoutError(f"no compass force-save COMMAND_ACK within {timeout_s}s")

    @staticmethod
    def _run_simple_accel_cal(connection, timeout_s: float) -> None:
        """Send MAV_CMD_PREFLIGHT_CALIBRATION (param5=SIMPLE) and confirm a
        real MAV_RESULT_ACCEPTED ack - see the class-level comment above
        ACCEL_CAL_TIMEOUT_S for why this, not a direct param_set, is the
        real fix for "3D Accel calibration needed". Vehicle-side this takes
        up to ~10s (AP_InertialSensor.cpp's simple_accel_cal() convergence
        loop), so the timeout here is generous, not tight.
        """
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mav2.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            0, 0, 0, 0,
            4,  # PREFLIGHT_CALIBRATION_ACCELEROMETER_SIMPLE
            0, 0,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
            if message is None:
                continue
            if message.command != mav2.MAV_CMD_PREFLIGHT_CALIBRATION:
                continue
            if message.result != mav2.MAV_RESULT_ACCEPTED:
                raise RenodeLauncherError(
                    f"simple accel cal not accepted: result={message.result}"
                )
            return
        raise TimeoutError(f"no accel-cal COMMAND_ACK within {timeout_s}s")

    @staticmethod
    def _confirm_param_set(
        connection, name: str, value: float, param_type: int, timeout_s: float
    ) -> None:
        connection.mav.param_set_send(
            connection.target_system, connection.target_component,
            name.encode("ascii"), float(value), param_type,
        )
        deadline = time.monotonic() + timeout_s
        last_seen = None
        while time.monotonic() < deadline:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
            if message is None:
                continue
            if message.param_id.rstrip("\x00") != name:
                continue
            # A stale PARAM_VALUE for this same name (echoing the value from
            # BEFORE this set, e.g. a leftover startup param-stream broadcast
            # already queued in the socket buffer) can arrive before the real
            # ack triggered by param_set_send above - so a single mismatch is
            # not proof of failure; keep listening until timeout instead of
            # raising immediately. REAL32 params also round-trip through a
            # 32-bit float on the vehicle side (e.g. 0.01 comes back as
            # 0.009999999776482582), so use a tolerance, not exact equality.
            last_seen = message.param_value
            if abs(float(message.param_value) - float(value)) <= 1e-4:
                return
        raise RenodeLauncherError(
            f"{name} never acked as {value} within {timeout_s}s (last seen: {last_seen})"
        )

    # ---- Lifecycle ----

    def _kill_any_stale_processes(self) -> None:
        """Best-effort cleanup of any previously-leaked renode or
        renode-physics process before starting new ones, mirroring the
        `pkill -9 -f renode` step that turned out to matter every single
        time tonight. One pattern covers both: "renode-physics" contains
        "renode" as a substring."""
        try:
            subprocess.run(
                ["pkill", "-9", "-f", "renode"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            time.sleep(1.0)
        except FileNotFoundError:
            pass  # pkill not available on this platform; not fatal

    def stop(self) -> None:
        """Tears down both processes - neither is left orphaned if the
        other fails to stop first, since each is torn down independently
        in its own try/except."""
        self._terminate(self._proc)
        self._proc = None
        if self._renode_log is not None:
            self._renode_log.close()
            self._renode_log = None
        self._terminate(self._physics_proc)
        self._physics_proc = None

    @staticmethod
    def _terminate(proc: subprocess.Popen | None) -> None:
        if proc is None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
