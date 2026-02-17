import time
import unittest

import numpy as np

from acoustic_tracking import SPEED_OF_SOUND_MPS, AcousticTrackingSystem
from drone import Drone, FlightMode, Position
from flying_ledger import Ed25519SignatureProvider, FlyingLedger
from swarm_manager import SwarmManager


def _build_impulse_signals(
    sensor_positions: dict,
    source_xy: tuple,
    sample_rate_hz: float,
    length: int = 4096,
    noise_std: float = 0.0,
):
    distances = {
        drone_id: np.hypot(source_xy[0] - pos[0], source_xy[1] - pos[1])
        for drone_id, pos in sensor_positions.items()
    }
    t0 = min(distances.values()) / SPEED_OF_SOUND_MPS
    base = 300
    signals = {}
    rng = np.random.default_rng(7)
    for drone_id, dist in distances.items():
        delay_s = (dist / SPEED_OF_SOUND_MPS) - t0
        shift = int(round(delay_s * sample_rate_hz))
        sig = np.zeros(length, dtype=np.float64)
        idx = min(length - 1, max(0, base + shift))
        sig[idx] = 1.0
        if noise_std > 0:
            sig += rng.normal(0.0, noise_std, size=length)
        signals[drone_id] = sig
    return signals


class LedgerAndAcousticTests(unittest.TestCase):
    def test_blockchain_consensus(self):
        providers = {i: Ed25519SignatureProvider() for i in [1, 2, 3]}
        pub_keys = {str(i): providers[i].public_key_bytes() for i in [1, 2, 3]}

        ledgers = {}
        for i in [1, 2, 3]:
            ledgers[i] = FlyingLedger(str(i), providers[i], peer_public_keys=pub_keys)

        def _broadcast(sender, block):
            for idx, ledger in ledgers.items():
                if idx == sender:
                    continue
                ledger.append_replicated_block(block)

        for i in [1, 2, 3]:
            ledgers[i].broadcaster = lambda block, sender=i: _broadcast(sender, block)

        ledgers[1].append_local_event({"pos": [1, 2, 3]}, {"event": "STATE_TRANSITION"})
        time.sleep(0.05)
        heights = {i: ledgers[i].block_height() for i in [1, 2, 3]}
        self.assertEqual(heights[1], heights[2])
        self.assertEqual(heights[2], heights[3])

    def test_block_validation_rejection(self):
        provider_a = Ed25519SignatureProvider()
        provider_b = Ed25519SignatureProvider()
        pub_keys = {"1": provider_a.public_key_bytes(), "2": provider_b.public_key_bytes()}
        ledger_a = FlyingLedger("1", provider_a, peer_public_keys=pub_keys)
        ledger_b = FlyingLedger("2", provider_b, peer_public_keys=pub_keys)

        block = ledger_a.append_local_event({"battery": 90}, {"event": "LATENCY_SPIKE"}).to_dict()
        tampered = dict(block)
        tampered["previous_hash"] = "bad_previous_hash"
        self.assertFalse(ledger_b.append_replicated_block(tampered))
        self.assertEqual(ledger_b.block_height(), 0)

    def test_acoustic_tdoa_accuracy(self):
        tracker = AcousticTrackingSystem()
        sensors = {1: (0.0, 0.0), 2: (40.0, 0.0), 3: (0.0, 35.0), 4: (35.0, 35.0)}
        source = (18.0, 11.0)
        sample_rate = 48000.0
        signals = _build_impulse_signals(sensors, source, sample_rate)
        result = tracker.localize(
            signals=signals,
            sensor_positions=sensors,
            sample_rate_hz=sample_rate,
            total_round_trip_ms=50.0,
            acoustic_latency_limit_ms=280.0,
        )
        self.assertTrue(result["detected"])
        est = result["source_position"]
        err = np.hypot(est["x"] - source[0], est["y"] - source[1])
        self.assertLess(err, 3.0)

    def test_noise_resilience(self):
        tracker = AcousticTrackingSystem()
        sensors = {1: (0.0, 0.0), 2: (50.0, 0.0), 3: (0.0, 45.0), 4: (45.0, 45.0)}
        source = (22.0, 16.0)
        sample_rate = 44100.0
        signals = _build_impulse_signals(sensors, source, sample_rate, noise_std=0.05)
        result = tracker.localize(
            signals=signals,
            sensor_positions=sensors,
            sample_rate_hz=sample_rate,
            total_round_trip_ms=90.0,
            acoustic_latency_limit_ms=280.0,
        )
        self.assertTrue(result["detected"])
        self.assertGreater(float(result["confidence"]), 0.35)

    def test_swarm_response_to_acoustic_event(self):
        swarm = SwarmManager()
        try:
            d1 = Drone(1, Position(0, 0, 0))
            d2 = Drone(2, Position(40, 0, 0))
            d3 = Drone(3, Position(0, 35, 0))
            swarm.add_drone(d1)
            swarm.add_drone(d2)
            swarm.add_drone(d3)
            for drone in swarm.drones.values():
                drone.is_armed = True
                drone.flight_mode = FlightMode.HOVER
            swarm.communication_manager.start()
            swarm.set_acoustic_detection_enabled(True)
            swarm.set_acoustic_confidence_threshold(0.4)

            sensors = {drone_id: (d.current_position.x, d.current_position.y) for drone_id, d in swarm.drones.items()}
            signals = _build_impulse_signals(sensors, (14.0, 10.0), 48000.0)
            result = swarm.process_acoustic_signals(signals, sample_rate_hz=48000.0, total_round_trip_ms=120.0)
            time.sleep(0.05)

            self.assertTrue(result["detected"])
            self.assertIsNotNone(swarm.get_swarm_status().get("acoustic", {}).get("latest_source"))
            states = [swarm.drone_state_manager.get_state(did).value for did in swarm.drones.keys()]
            self.assertIn("ACOUSTIC_TRACKING", states)
        finally:
            swarm.stop()
            for drone in list(swarm.drones.values()):
                drone.stop()

    def test_ledger_persistence_after_drone_failure(self):
        swarm = SwarmManager()
        try:
            for i, pos in enumerate([Position(0, 0, 0), Position(10, 0, 0), Position(0, 10, 0)], start=1):
                swarm.add_drone(Drone(i, pos))
            swarm.communication_manager.start()
            swarm._record_critical_event(1, "TEST_EVENT", {"value": 1})
            time.sleep(0.1)
            swarm.remove_drone(1)
            status = swarm.get_swarm_status()
            heights = status.get("ledger", {}).get("per_drone_height", {})
            self.assertGreaterEqual(int(heights.get(2, 0)), 1)
            self.assertGreaterEqual(int(heights.get(3, 0)), 1)
        finally:
            swarm.stop()
            for drone in list(swarm.drones.values()):
                drone.stop()


if __name__ == "__main__":
    unittest.main()
