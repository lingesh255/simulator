"""GCS <-> drone wire protocol.

JSON over UDP, one datagram per message - except a command too big for one
datagram (a formation's whole timed track), which `fragment` splits into
numbered FRAGMENT datagrams that the drone's `FragmentAssembler` joins back
together. Deliberately simple and human readable: this is the transport that later gets swapped for MAVLink, and
keeping the schema explicit makes that substitution a translation exercise
rather than a rewrite.

    GCS ----- Command -----> Drone      (COMMAND)
    GCS <---- Telemetry ---- Drone      (TELEMETRY, HEARTBEAT, ACK)

Every message carries `drone_id`, which is the drone's MAVLink SYSID, so a
single GCS socket can address and demultiplex a whole swarm.
"""
from __future__ import annotations

import base64
import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

# Each drone listens on its own port so the swarm needs no demultiplexing on
# the way down; SYSID 1 -> 14701, SYSID 2 -> 14702, and so on.
DRONE_BASE_PORT = 14700
GCS_PORT = 14690
MAX_DATAGRAM = 65535
# Largest message sent as one datagram; anything bigger goes as fragments.
# UDP over IPv4 carries at most 65507 bytes, so keep well clear of it.
MAX_SINGLE_DATAGRAM = 60_000
# Raw bytes per fragment - base64 makes it 4/3 bigger, plus a small header.
FRAGMENT_CHUNK = 42_000


def drone_port(sysid: int, base_port: int = DRONE_BASE_PORT) -> int:
    return base_port + sysid


class MessageType(str, Enum):
    COMMAND = "COMMAND"
    FRAGMENT = "FRAGMENT"  # one piece of a command too big for one datagram
    TELEMETRY = "TELEMETRY"
    HEARTBEAT = "HEARTBEAT"
    ACK = "ACK"


class CommandName(str, Enum):
    """Commands the GCS can issue. NAVIGATE is the one mission operation."""

    NAVIGATE = "NAVIGATE"
    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    RESUME = "RESUME"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"
    ABORT = "ABORT"
    SET_GPS = "SET_GPS"
    SET_SPEED = "SET_SPEED"   # change cruise speed in flight
    TRAIN_PACE = "TRAIN_PACE"  # formation keeping: where the front of the train is (engine.train_keeping)
    CHARGE = "CHARGE"         # landed at a charging station: charge the battery (engine.battery_mission)
    TASK_DATA = "TASK_DATA"   # mission state relayed to a replacement drone
    PING = "PING"
    SHUTDOWN = "SHUTDOWN"


class ProtocolError(ValueError):
    """A datagram was not a message we understand."""


@dataclass
class CommandMessage:
    """GCS -> drone.

    Wire form, matching the agreed schema:

        {"drone_id": 1, "command": "NAVIGATE",
         "source": [lat, lon, alt], "destination": [lat, lon, alt]}
    """

    drone_id: int
    command: CommandName
    source: Optional[list[float]] = None
    destination: Optional[list[float]] = None
    params: dict[str, Any] = field(default_factory=dict)
    seq: int = 0
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        body: dict[str, Any] = {
            "type": MessageType.COMMAND.value,
            "drone_id": self.drone_id,
            "command": self.command.value,
            "seq": self.seq,
            "timestamp": self.timestamp,
        }
        if self.source is not None:
            body["source"] = self.source
        if self.destination is not None:
            body["destination"] = self.destination
        if self.params:
            body["params"] = self.params
        return json.dumps(body).encode("utf-8")

    @classmethod
    def from_json(cls, raw: bytes) -> "CommandMessage":
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"undecodable datagram: {exc}") from exc
        if not isinstance(body, dict):
            raise ProtocolError("message body is not an object")
        try:
            command = CommandName(body["command"])
        except KeyError as exc:
            raise ProtocolError("command message has no 'command' field") from exc
        except ValueError as exc:
            raise ProtocolError(f"unknown command {body.get('command')!r}") from exc
        try:
            drone_id = int(body["drone_id"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError("command message has no usable 'drone_id'") from exc

        return cls(
            drone_id=drone_id,
            command=command,
            source=body.get("source"),
            destination=body.get("destination"),
            params=body.get("params", {}),
            seq=int(body.get("seq", 0)),
            timestamp=float(body.get("timestamp", time.time())),
        )


@dataclass
class TelemetryMessage:
    """Drone -> GCS: position, battery, GPS, health, mission state."""

    drone_id: int
    position: list[float]
    velocity: dict[str, float]
    battery: float
    gps: dict[str, Any]
    health: str
    mission_state: str
    armed: bool
    operation: str
    communication: str
    destination: Optional[list[float]] = None
    elapsed_s: float = 0.0
    note: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.TELEMETRY.value,
                "drone_id": self.drone_id,
                "position": self.position,
                "velocity": self.velocity,
                "battery": self.battery,
                "gps": self.gps,
                "health": self.health,
                "mission_state": self.mission_state,
                "armed": self.armed,
                "operation": self.operation,
                "communication": self.communication,
                "destination": self.destination,
                "elapsed_s": self.elapsed_s,
                "note": self.note,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


@dataclass
class HeartbeatMessage:
    """Drone -> GCS: proof of life, independent of mission activity."""

    drone_id: int
    seq: int
    uptime_s: float
    mission_state: str
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.HEARTBEAT.value,
                "drone_id": self.drone_id,
                "seq": self.seq,
                "uptime_s": self.uptime_s,
                "mission_state": self.mission_state,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


@dataclass
class AckMessage:
    """Drone -> GCS: this command was received and accepted (or refused)."""

    drone_id: int
    command: str
    seq: int
    accepted: bool
    detail: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.ACK.value,
                "drone_id": self.drone_id,
                "command": self.command,
                "seq": self.seq,
                "accepted": self.accepted,
                "detail": self.detail,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


def fragment(raw: bytes, drone_id: int, message_id: int) -> list[bytes]:
    """`raw` (one encoded message) as datagrams no bigger than one UDP
    datagram can carry: itself if it fits, otherwise FRAGMENT messages
    numbered 0 .. count-1, each carrying a base64 slice of it."""
    if len(raw) <= MAX_SINGLE_DATAGRAM:
        return [raw]
    chunks = [raw[i:i + FRAGMENT_CHUNK] for i in range(0, len(raw), FRAGMENT_CHUNK)]
    return [
        json.dumps({
            "type": MessageType.FRAGMENT.value,
            "drone_id": drone_id,
            "message_id": message_id,
            "index": index,
            "count": len(chunks),
            "data": base64.b64encode(chunk).decode("ascii"),
        }).encode("utf-8")
        for index, chunk in enumerate(chunks)
    ]


class FragmentAssembler:
    """Joins FRAGMENT datagrams back into the message they were cut from."""

    MAX_PENDING = 8  # messages being assembled at once; older ones are dropped

    def __init__(self) -> None:
        self._pending: dict[int, dict[int, bytes]] = {}

    @staticmethod
    def is_fragment(raw: bytes) -> bool:
        return b'"FRAGMENT"' in raw[:64]

    def add(self, raw: bytes) -> Optional[bytes]:
        """Take one FRAGMENT datagram; the whole message once its last piece
        has arrived, otherwise None."""
        try:
            body = json.loads(raw.decode("utf-8"))
            message_id, index, count = int(body["message_id"]), int(body["index"]), int(body["count"])
            data = base64.b64decode(body["data"])
        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"malformed fragment: {exc}") from exc
        pieces = self._pending.setdefault(message_id, {})
        pieces[index] = data
        while len(self._pending) > self.MAX_PENDING:
            self._pending.pop(next(iter(self._pending)))
        if len(pieces) < count:
            return None
        del self._pending[message_id]
        return b"".join(pieces[i] for i in range(count))


def decode_uplink(raw: bytes) -> dict[str, Any]:
    """Parse anything a drone sends to the GCS."""
    try:
        body = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"undecodable datagram: {exc}") from exc
    if not isinstance(body, dict) or "type" not in body:
        raise ProtocolError("uplink message has no 'type'")
    return body
