"""Step 1: shutdown paths and the terminal, on a real pseudo-terminal.

    python tests/harness/step1_test.py ctrlc kill close

ctrlc: writes ^C (0x03) to the terminal, so the tty driver sends SIGINT.
kill : SIGTERM to the app pid, like `kill <pid>` from another terminal.
close: out/close_now -> window.close() (the title-bar X path).
After each: wait for the app to exit, show pgrep, show the terminal's echo.
"""
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
PY = sys.executable


def ts(t0):
    return f"[{time.monotonic() - t0:6.1f}s]"


def pgrep():
    out = subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True).stdout
    return out.strip() or "(nothing)"


def stty_echo(slave_fd):
    full = subprocess.run(["stty", "-a"], stdin=slave_fd, capture_output=True, text=True).stdout
    grep = subprocess.run(["bash", "-c", "stty -a | grep -o -- '-\\?echo '"], stdin=slave_fd,
                          capture_output=True, text=True).stdout
    flags = [w for w in full.split() if w.lstrip("-") in ("echo", "icanon", "isig")]
    return grep.strip(), flags


def drain(master, seconds):
    out = b""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        r, _, _ = select.select([master], [], [], 0.3)
        if not r:
            continue
        try:
            chunk = os.read(master, 65536)
        except OSError:
            break
        if not chunk:
            break
        out += chunk
    return out


def run(case):
    t0 = time.monotonic()
    (HARNESS / "out").mkdir(exist_ok=True)
    master, slave = os.openpty()
    print(f"{ts(t0)} case={case} tty={os.ttyname(slave)} before: echo={stty_echo(slave)}", flush=True)

    def child_setup():
        import fcntl
        import termios
        os.setsid()
        fcntl.ioctl(0, termios.TIOCSCTTY, 0)

    proc = subprocess.Popen([PY, str(HARNESS / "step1_app.py")], stdin=slave, stdout=slave, stderr=slave,
                            preexec_fn=child_setup, close_fds=True)
    buf = b""
    deadline = time.monotonic() + 600
    while time.monotonic() < deadline and b"READY" not in buf and b"LAUNCH FAILED" not in buf:
        buf += drain(master, 0.5)
    lines = [l for l in buf.decode(errors="replace").splitlines() if any(k in l for k in ("READY", "APP PID", "FAILED"))]
    print(f"{ts(t0)} app output so far: {lines}", flush=True)
    print(f"{ts(t0)} pgrep while running:\n{pgrep()}", flush=True)
    print(f"{ts(t0)} terminal flags while running: {stty_echo(slave)}", flush=True)

    if case == "ctrlc":
        print(f"{ts(t0)} writing ^C (0x03) to the terminal", flush=True)
        os.write(master, b"\x03")
    elif case == "kill":
        print(f"{ts(t0)} kill {proc.pid}  (SIGTERM)", flush=True)
        os.kill(proc.pid, signal.SIGTERM)
    elif case == "close":
        print(f"{ts(t0)} requesting window.close()", flush=True)
        (HARNESS / "out" / "close_now").write_text("x")

    try:
        rc = proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        rc = "STILL RUNNING after 30s"
    tail = drain(master, 1.0)
    last = [l for l in tail.decode(errors="replace").splitlines() if l.strip()][-4:]
    print(f"{ts(t0)} app exit code: {rc}; last app lines: {last}", flush=True)
    time.sleep(2)
    print(f"{ts(t0)} pgrep after exit (+2s): {pgrep()}", flush=True)
    print(f"{ts(t0)} stty -a | grep -o -- '-\\?echo ' -> {stty_echo(slave)}", flush=True)
    os.close(master)
    os.close(slave)


if __name__ == "__main__":
    for c in sys.argv[1:]:
        run(c)
        print("-" * 60, flush=True)
