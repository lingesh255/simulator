"""Listen on a TCP port and never say anything - occupies a MAVLink port."""
import socket
import sys
import time

sock = socket.socket()
# Like Renode's own listener: a connection from an earlier run still in
# TIME_WAIT on this port must not stop the squatter from taking it.
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", int(sys.argv[1])))
sock.listen(8)
held = []
sock.settimeout(1)
while True:
    try:
        conn, _ = sock.accept()
        held.append(conn)
    except socket.timeout:
        time.sleep(0.1)
