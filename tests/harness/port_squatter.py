"""Listen on a TCP port and never say anything - occupies a MAVLink port."""
import socket
import sys
import time

sock = socket.socket()
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
