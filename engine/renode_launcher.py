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
"""
from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from pathlib import Path


class RenodeLauncherError(Exception):
    pass


class RenodeLauncher:
    def __init__(self, standalone_dir: str, port: int = 5762):
        self.standalone_dir = Path(standalone_dir).expanduser().resolve()
        self.port = port

        self.renode_bin = self.standalone_dir / "renode-bin" / "renode"
        self.launch_script = self.standalone_dir / "launch.resc"

        if not self.renode_bin.is_file():
            raise RenodeLauncherError(
                f"No renode executable at {self.renode_bin} - is this a real "
                "standalone extraction? See renode_firmware_guide.md Part 4."
            )
        if not self.launch_script.is_file():
            raise RenodeLauncherError(f"No launch.resc at {self.launch_script}")

        self._proc: subprocess.Popen | None = None

    @property
    def connection_string(self) -> str:
        return f"tcp:127.0.0.1:{self.port}"

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, ready_timeout_s: float = 60.0) -> str:
        """Starts Renode fresh and waits for the MAVLink port to come up.
        Returns the connection string on success.

        IMPORTANT (learned the hard way): the MAVLink socket this exposes
        serves its first client only. Do NOT probe it with a throwaway
        connection to "test readiness" - that consumes the one real slot.
        This waits for the port to be listening (a plain TCP accept-check,
        which does not itself complete a MAVLink handshake) rather than
        connecting and disconnecting.
        """
        if self.is_running:
            raise RenodeLauncherError("already running - call stop() first")

        self._kill_any_stale_renode()

        # New process group so stop() can cleanly kill Renode's own child
        # processes too, not just this one PID - a bare terminate() here
        # was the exact cause of repeated leaked-process/"address already
        # in use" problems during development.
        self._proc = subprocess.Popen(
            [str(self.renode_bin), "--console", str(self.launch_script)],
            cwd=self.standalone_dir,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        try:
            self._wait_for_port(ready_timeout_s)
        except Exception:
            self.stop()
            raise

        return self.connection_string

    def _kill_any_stale_renode(self) -> None:
        """Best-effort cleanup of any previously-leaked renode process
        before starting a new one, mirroring the `pkill -9 -f renode`
        step that turned out to matter every single time tonight."""
        try:
            subprocess.run(
                ["pkill", "-9", "-f", "renode"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            time.sleep(1.0)
        except FileNotFoundError:
            pass  # pkill not available on this platform; not fatal

    def _wait_for_port(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                raise RenodeLauncherError(
                    f"renode exited early (code {self._proc.returncode})"
                )
            try:
                with socket.create_connection(
                    ("127.0.0.1", self.port), timeout=0.5
                ) as s:
                    s.close()
                    return
            except OSError:
                time.sleep(0.5)
        raise TimeoutError(
            f"MAVLink port {self.port} never came up within {timeout_s}s"
        )

    def stop(self) -> None:
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            self._proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._proc = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
