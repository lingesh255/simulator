"""Send commands to the single Renode's telnet monitor (run_fleet.py --telnet) and print the replies.

    python experiments/single_renode/mon.py 'mach set "drone1"' 'cpu PC' 'machine ElapsedVirtualTime'
"""
import re
import socket
import sys
import time

PORT = 4455


def mon(commands, wait=1.5):
    sock = socket.create_connection(("127.0.0.1", PORT), timeout=5)
    sock.settimeout(0.4)
    time.sleep(0.4)
    try:
        sock.recv(65536)
    except OSError:
        pass
    for command in commands:
        sock.sendall(command.encode() + b"\n")
        end, buf = time.time() + wait, b""
        while time.time() < end:
            try:
                buf += sock.recv(65536)
            except OSError:
                pass
        text = re.sub(r"\x1b\[[0-9;]*m", "", buf.decode(errors="replace").replace("\r", ""))
        print(">>", command, "\n  " + "\n  ".join(line for line in text.split("\n") if line.strip()))
    sock.close()


if __name__ == "__main__":
    mon(sys.argv[1:])
