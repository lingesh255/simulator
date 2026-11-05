"""Drone-to-drone messaging for a formation: each drone's agent talks to its
peers directly, so the launch order and the "everyone is up, go" release are
decided by the drones' own messages instead of by a shared Python flag.

    drone-lead agent --+
    drone-left agent --+-- one UDP datagram per peer, no hub
    drone-right agent -+

Each drone's agent owns one `PeerBus`: a UDP socket plus two small threads
(receive, and a once-a-second re-send). A message is one JSON datagram,
`{"from": name, "topic": ..., "data": {...}}`, sent to every other agent.

Two kinds of message:

  * `announce(topic, **data)` - a *state*: "I am in my slot", "I am ready".
    It is remembered and re-sent every `resend_s`, so a peer that was not
    listening yet (or lost a datagram) still learns it.
  * `HELLO` - sent automatically on the same beat, carrying nothing; it only
    proves the agent is alive. A peer silent for `silent_after_s`, or one that
    announced `DOWN` (closing a bus does this), counts as down.

`wait_for_peers` is the one blocking primitive: wait until every named peer
(this agent included) has announced a topic. It is how a wing waits for the
drone ahead to be in its slot, and how every drone waits for the whole V to
be assembled.

The autopilots themselves have no drone-to-drone link, so this lives next to
each vehicle, the way a companion computer would on real hardware. In this
repo the agents are threads of one process that each hold their own socket
(`local_mesh`); the same class works across separate processes or machines by
passing each agent the others' real `(host, port)` as `peers`.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, Iterable, Optional

from engine.mavlink_mission import FlightAborted

HELLO = "HELLO"
DOWN = "DOWN"

_WAIT_SLICE_S = 0.1


class PeerDown(RuntimeError):
    """A peer this agent needed has dropped out."""


class PeerBus:
    """One drone's endpoint on the formation's peer-to-peer channel."""

    def __init__(
        self,
        name: str,
        *,
        bind: tuple[str, int] = ("127.0.0.1", 0),
        peers: Optional[dict[str, tuple[str, int]]] = None,
        resend_s: float = 1.0,
        silent_after_s: float = 10.0,
        on_heard: Optional[Callable[[str, str, str, dict], None]] = None,
    ) -> None:
        """`bind` port 0 takes any free port - read it back from `address`.
        `peers` maps every *other* agent's name to its `(host, port)`; it can
        also be given later with `set_peers`. `on_heard(me, sender, topic,
        data)`, if given, runs on the receive thread the first time `sender`
        announces `topic` - for logging the exchange."""
        self.name = name
        self._resend_s = resend_s
        self._silent_after_s = silent_after_s
        self._on_heard = on_heard
        self._peers: dict[str, tuple[str, int]] = {}
        self._cond = threading.Condition()
        self._heard: dict[str, dict[str, dict]] = {}  # topic -> sender -> data
        self._seen: dict[str, float] = {}             # sender -> monotonic time last heard from
        self._sticky: dict[str, dict] = {}            # topic -> data, re-sent every beat
        self._closed = threading.Event()
        self._threads: list[threading.Thread] = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(bind)
        self._sock.settimeout(0.2)
        self.set_peers(peers or {})

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._sock.getsockname()[:2]
        return host, port

    def set_peers(self, peers: dict[str, tuple[str, int]]) -> None:
        with self._cond:
            self._peers = {n: tuple(a) for n, a in peers.items() if n != self.name}

    def start(self) -> None:
        for target in (self._recv_loop, self._beat_loop):
            thread = threading.Thread(target=target, name=f"peerbus-{self.name}", daemon=True)
            thread.start()
            self._threads.append(thread)

    def close(self) -> None:
        """Tell the peers this agent is leaving (`DOWN`), then stop."""
        if self._closed.is_set():
            return
        self._send(DOWN, {})
        self._closed.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._sock.close()

    # ---- sending ----------------------------------------------------------

    def announce(self, topic: str, **data) -> None:
        """Publish a state to every peer and keep re-sending it. It also counts
        as heard for this agent itself, so `wait_for_peers` over the whole
        formation includes the caller."""
        with self._cond:
            self._sticky[topic] = data
            self._heard.setdefault(topic, {})[self.name] = data
            self._cond.notify_all()
        self._send(topic, data)

    def _send(self, topic: str, data: dict) -> None:
        payload = json.dumps({"v": 1, "from": self.name, "topic": topic, "data": data}).encode()
        with self._cond:
            addresses = list(self._peers.values())
        for address in addresses:
            try:
                self._sock.sendto(payload, address)
            except OSError:
                pass  # that peer's socket is gone - its silence is what marks it down

    def _beat_loop(self) -> None:
        while not self._closed.wait(self._resend_s):
            self._send(HELLO, {})
            with self._cond:
                states = list(self._sticky.items())
            for topic, data in states:
                self._send(topic, data)

    # ---- receiving --------------------------------------------------------

    def _recv_loop(self) -> None:
        while not self._closed.is_set():
            try:
                raw, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue  # Windows reports an earlier send to a closed peer here
            except OSError:
                return
            self._handle(raw)

    def _handle(self, raw: bytes) -> None:
        try:
            message = json.loads(raw)
            sender, topic, data = message["from"], message["topic"], message.get("data") or {}
        except (ValueError, KeyError, TypeError):
            return  # not one of ours
        if sender == self.name:
            return
        first_time = False
        with self._cond:
            self._seen[sender] = time.monotonic()
            if topic != HELLO:
                senders = self._heard.setdefault(topic, {})
                first_time = sender not in senders
                senders[sender] = data
                self._cond.notify_all()
        if first_time and self._on_heard is not None:
            try:
                self._on_heard(self.name, sender, topic, data)
            except Exception:  # noqa: BLE001 - a logging callback must not kill the receiver
                pass

    # ---- reading ----------------------------------------------------------

    def heard(self, topic: str) -> dict[str, dict]:
        """sender -> data for everyone (this agent included) who announced `topic`."""
        with self._cond:
            return dict(self._heard.get(topic, {}))

    def is_down(self, peer: str) -> bool:
        with self._cond:
            return self._is_down_locked(peer)

    def _is_down_locked(self, peer: str) -> bool:
        if peer in self._heard.get(DOWN, {}):
            return True
        seen = self._seen.get(peer)
        return seen is not None and time.monotonic() - seen > self._silent_after_s

    def wait_for_peers(
        self,
        topic: str,
        peers: Iterable[str],
        *,
        timeout_s: float = 60.0,
        should_abort: Optional[Callable[[], bool]] = None,
        pump: Optional[Callable[[], None]] = None,
        down_ok: bool = True,
    ) -> set[str]:
        """Block until every peer in `peers` has announced `topic`. This agent
        may be in `peers` (it must have announced the topic itself first).

        A peer that is down is dropped from the wait: with `down_ok` the wait
        carries on without it and the dropped names are returned; without it
        `PeerDown` is raised at once, so a formation that cannot assemble
        fails now instead of after the timeout. `pump`, if given, runs every
        ~0.1 s - the caller's chance to keep servicing its vehicle link.
        Raises `FlightAborted` if `should_abort` turns true and `TimeoutError`
        after `timeout_s`."""
        wanted = list(peers)
        deadline = time.monotonic() + timeout_s
        while True:
            with self._cond:
                announced = self._heard.get(topic, {})
                missing = [p for p in wanted if p not in announced]
                gone = {p for p in missing if self._is_down_locked(p)}
                if gone and not down_ok:
                    raise PeerDown(f"{', '.join(sorted(gone))} dropped out while waiting for '{topic}'")
                pending = [p for p in missing if p not in gone]
                if not pending:
                    return gone
                self._cond.wait(_WAIT_SLICE_S)
            if pump is not None:
                pump()
            if should_abort is not None and should_abort():
                raise FlightAborted(f"{self.name}: aborted while waiting for '{topic}' from {', '.join(pending)}")
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{self.name}: timed out after {timeout_s:.0f}s waiting for '{topic}' from {', '.join(pending)}"
                )


def local_mesh(names: Iterable[str], **bus_kwargs) -> dict[str, PeerBus]:
    """One started `PeerBus` per name, each on its own loopback UDP port and
    wired to all the others. `bus_kwargs` go to every `PeerBus`."""
    buses = {name: PeerBus(name, **bus_kwargs) for name in names}
    addresses = {name: bus.address for name, bus in buses.items()}
    for bus in buses.values():
        bus.set_peers(addresses)
        bus.start()
    return buses
