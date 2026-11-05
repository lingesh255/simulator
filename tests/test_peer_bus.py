"""engine.peer_bus - drone-to-drone messaging over real loopback UDP sockets
(plain unittest - also collected by pytest)."""
import threading
import time
import unittest

from engine.mavlink_mission import FlightAborted
from engine.peer_bus import DOWN, PeerBus, PeerDown, local_mesh

NAMES = ("drone-lead", "drone-left", "drone-right")
FAST = dict(resend_s=0.05, silent_after_s=0.4)


class PeerBusTests(unittest.TestCase):
    def setUp(self):
        self.buses = {}

    def tearDown(self):
        for bus in self.buses.values():
            bus.close()

    def mesh(self, names=NAMES, **kwargs):
        self.buses = local_mesh(names, **{**FAST, **kwargs})
        return self.buses

    def test_announcement_reaches_every_peer(self):
        buses = self.mesh()
        buses["drone-lead"].announce("IN_SLOT", alt=60.0)
        for name in ("drone-left", "drone-right"):
            buses[name].wait_for_peers("IN_SLOT", ["drone-lead"], timeout_s=2.0)
            self.assertEqual(buses[name].heard("IN_SLOT")["drone-lead"], {"alt": 60.0})

    def test_wait_over_whole_formation_includes_the_caller(self):
        buses = self.mesh()
        for bus in buses.values():
            bus.announce("IN_SLOT")
        for bus in buses.values():
            self.assertEqual(bus.wait_for_peers("IN_SLOT", NAMES, timeout_s=2.0), set())

    def test_wing_launches_only_after_the_drone_ahead_is_in_slot(self):
        """The real sequence: lead -> left -> right, each only after hearing
        IN_SLOT from the one ahead - nothing but the buses connects the threads."""
        buses = self.mesh()
        order = []
        order_lock = threading.Lock()

        def drone(index):
            name = NAMES[index]
            bus = buses[name]
            if index > 0:
                bus.wait_for_peers("IN_SLOT", [NAMES[index - 1]], timeout_s=5.0)
            with order_lock:
                order.append(f"{name} takeoff")
            time.sleep(0.1)  # climbing out
            with order_lock:
                order.append(f"{name} in slot")
            bus.announce("IN_SLOT")

        # Start in the reverse of launch order so only the messages can sequence them.
        threads = [threading.Thread(target=drone, args=(i,)) for i in (2, 1, 0)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10.0)

        self.assertEqual(order, [
            "drone-lead takeoff", "drone-lead in slot",
            "drone-left takeoff", "drone-left in slot",
            "drone-right takeoff", "drone-right in slot",
        ])

    def test_dropped_out_peer_is_given_up_on_when_down_is_ok(self):
        buses = self.mesh()
        buses["drone-lead"].announce(DOWN)
        gone = buses["drone-left"].wait_for_peers("IN_SLOT", ["drone-lead"], timeout_s=2.0)
        self.assertEqual(gone, {"drone-lead"})

    def test_dropped_out_peer_fails_an_assembly_at_once(self):
        buses = self.mesh()
        buses["drone-left"].announce("IN_SLOT")
        buses["drone-right"].announce(DOWN)
        started = time.monotonic()
        with self.assertRaises(PeerDown):
            buses["drone-left"].wait_for_peers("IN_SLOT", NAMES, timeout_s=30.0, down_ok=False)
        self.assertLess(time.monotonic() - started, 5.0)  # not the 30 s timeout

    def test_closing_a_bus_tells_the_others_it_is_down(self):
        buses = self.mesh()
        buses["drone-right"].close()
        gone = buses["drone-left"].wait_for_peers("IN_SLOT", ["drone-right"], timeout_s=2.0)
        self.assertEqual(gone, {"drone-right"})

    def test_silent_peer_counts_as_down(self):
        a = PeerBus("a", **FAST)
        b = PeerBus("b", **FAST)  # never started: no beat, no receiver
        self.buses = {"a": a, "b": b}
        a.set_peers({"b": b.address})
        b.set_peers({"a": a.address})
        a.start()
        b.announce("IN_SLOT")  # one datagram, then silence
        a.wait_for_peers("IN_SLOT", ["b"], timeout_s=2.0)
        self.assertFalse(a.is_down("b"))
        deadline = time.monotonic() + 3.0
        while not a.is_down("b") and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(a.is_down("b"))

    def test_times_out_naming_who_it_is_waiting_for(self):
        buses = self.mesh()
        with self.assertRaisesRegex(TimeoutError, "drone-right"):
            buses["drone-lead"].wait_for_peers("IN_SLOT", ["drone-right"], timeout_s=0.3)

    def test_abort_stops_the_wait(self):
        buses = self.mesh()
        with self.assertRaises(FlightAborted):
            buses["drone-lead"].wait_for_peers(
                "IN_SLOT", ["drone-right"], timeout_s=30.0, should_abort=lambda: True,
            )

    def test_pump_runs_while_waiting(self):
        buses = self.mesh()
        calls = []
        with self.assertRaises(TimeoutError):
            buses["drone-lead"].wait_for_peers(
                "IN_SLOT", ["drone-right"], timeout_s=0.5, pump=lambda: calls.append(1),
            )
        self.assertGreaterEqual(len(calls), 2)

    def test_on_heard_fires_once_per_sender_and_topic(self):
        heard = []
        buses = self.mesh(on_heard=lambda me, sender, topic, _d: heard.append((me, sender, topic)))
        buses["drone-lead"].announce("IN_SLOT")
        buses["drone-left"].wait_for_peers("IN_SLOT", ["drone-lead"], timeout_s=2.0)
        time.sleep(0.3)  # several re-sends of the same state
        self.assertEqual(heard.count(("drone-left", "drone-lead", "IN_SLOT")), 1)

    def test_garbage_datagram_is_ignored(self):
        import socket

        buses = self.mesh()
        junk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            junk.sendto(b"not json", buses["drone-lead"].address)
            junk.sendto(b'{"unrelated": true}', buses["drone-lead"].address)
        finally:
            junk.close()
        buses["drone-left"].announce("IN_SLOT")
        buses["drone-lead"].wait_for_peers("IN_SLOT", ["drone-left"], timeout_s=2.0)


if __name__ == "__main__":
    unittest.main()
