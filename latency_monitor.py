#########################################################################
#                                                                       #
#   SECURE DRONE SWARM SYSTEM - CORE MODULE                             #
#                                                                       #
#   Developer : Md Shahanur Islam Shagor                                #
#   Role      : Project Architect & Lead Developer                      #
#   Version   : 1.0.2                                                   #
#   Status    : Production Ready                                        #
#                                                                       #
#   "Protecting the skies with decentralized intelligence."             #
#                                                                       #
#########################################################################
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional


@dataclass
class LatencySample:
    t_cpp_send: float
    t_py_recv: float
    t_py_send: float
    t_cpp_recv: float

    @property
    def cpp_to_py(self) -> float:
        return max(0.0, self.t_py_recv - self.t_cpp_send)

    @property
    def py_processing(self) -> float:
        return max(0.0, self.t_py_send - self.t_py_recv)

    @property
    def py_to_cpp(self) -> float:
        return max(0.0, self.t_cpp_recv - self.t_py_send)

    @property
    def total_round_trip(self) -> float:
        return max(0.0, self.t_cpp_recv - self.t_cpp_send)


class LatencyMonitor:
    def __init__(self, window_size: int = 120, latency_threshold_ms: float = 220.0):
        self.window_size = max(10, int(window_size))
        self.latency_threshold_ms = float(latency_threshold_ms)
        self.samples: Deque[LatencySample] = deque(maxlen=self.window_size)
        self._lock = threading.RLock()
        self.logger = logging.getLogger("LatencyMonitor")

    def set_threshold_ms(self, threshold_ms: float):
        with self._lock:
            self.latency_threshold_ms = max(1.0, float(threshold_ms))

    def record(self, sample: LatencySample) -> Dict[str, float]:
        with self._lock:
            self.samples.append(sample)
            stats = self.get_stats_unlocked()
            if stats["total_round_trip_ms"] > self.latency_threshold_ms:
                self.logger.warning(
                    "Latency spike detected total=%.2fms threshold=%.2fms",
                    stats["total_round_trip_ms"],
                    self.latency_threshold_ms,
                )
            return stats

    def get_stats(self) -> Dict[str, float]:
        with self._lock:
            return self.get_stats_unlocked()

    def get_stats_unlocked(self) -> Dict[str, float]:
        if not self.samples:
            return {
                "samples": 0,
                "cpp_to_py_ms": 0.0,
                "py_processing_ms": 0.0,
                "py_to_cpp_ms": 0.0,
                "total_round_trip_ms": 0.0,
                "cpp_to_py_jitter_std_ms": 0.0,
                "py_processing_jitter_std_ms": 0.0,
                "py_to_cpp_jitter_std_ms": 0.0,
                "total_round_trip_jitter_std_ms": 0.0,
                "threshold_ms": self.latency_threshold_ms,
                "fallback_required": False,
            }

        n = float(len(self.samples))
        cpp_to_py_ms = 1000.0 * sum(s.cpp_to_py for s in self.samples) / n
        py_processing_ms = 1000.0 * sum(s.py_processing for s in self.samples) / n
        py_to_cpp_ms = 1000.0 * sum(s.py_to_cpp for s in self.samples) / n
        total_ms = 1000.0 * sum(s.total_round_trip for s in self.samples) / n

        def _std_ms(values_seconds):
            values_ms = [1000.0 * v for v in values_seconds]
            if len(values_ms) < 2:
                return 0.0
            mean = sum(values_ms) / len(values_ms)
            var = sum((v - mean) ** 2 for v in values_ms) / len(values_ms)
            return math.sqrt(var)

        cpp_jitter = _std_ms([s.cpp_to_py for s in self.samples])
        proc_jitter = _std_ms([s.py_processing for s in self.samples])
        py_cpp_jitter = _std_ms([s.py_to_cpp for s in self.samples])
        total_jitter = _std_ms([s.total_round_trip for s in self.samples])
        return {
            "samples": int(n),
            "cpp_to_py_ms": cpp_to_py_ms,
            "py_processing_ms": py_processing_ms,
            "py_to_cpp_ms": py_to_cpp_ms,
            "total_round_trip_ms": total_ms,
            "cpp_to_py_jitter_std_ms": cpp_jitter,
            "py_processing_jitter_std_ms": proc_jitter,
            "py_to_cpp_jitter_std_ms": py_cpp_jitter,
            "total_round_trip_jitter_std_ms": total_jitter,
            "threshold_ms": self.latency_threshold_ms,
            "fallback_required": total_ms > self.latency_threshold_ms,
        }


class MLBridge:
    """
    Simulated C++<->Python bridge timing hooks.
    Real IPC layer can call these same methods around send/receive boundaries.
    """

    def __init__(self, latency_monitor: LatencyMonitor, watchdog_timeout_s: float = 1.5):
        self.latency_monitor = latency_monitor
        self.logger = logging.getLogger("MLBridge")
        self.watchdog_timeout_s = max(0.1, float(watchdog_timeout_s))
        self.last_response_time = time.time()

    def round_trip(self, py_processing_seconds: float = 0.004, net_one_way_seconds: float = 0.002) -> Dict[str, float]:
        t_cpp_send = time.time()
        t_py_recv = t_cpp_send + max(0.0, net_one_way_seconds)
        t_py_send = t_py_recv + max(0.0, py_processing_seconds)
        t_cpp_recv = t_py_send + max(0.0, net_one_way_seconds)
        sample = LatencySample(
            t_cpp_send=t_cpp_send,
            t_py_recv=t_py_recv,
            t_py_send=t_py_send,
            t_cpp_recv=t_cpp_recv,
        )
        stats = self.latency_monitor.record(sample)
        self.last_response_time = time.time()
        return stats

    def inject_spike(self, total_ms: float = 400.0) -> Dict[str, float]:
        half_net = max(0.0, (total_ms / 1000.0) * 0.2)
        py_time = max(0.0, (total_ms / 1000.0) * 0.6)
        self.logger.warning("Injecting synthetic latency spike %.1fms", total_ms)
        return self.round_trip(py_processing_seconds=py_time, net_one_way_seconds=half_net)

    def is_watchdog_timed_out(self, now: Optional[float] = None) -> bool:
        current = now if now is not None else time.time()
        return (current - self.last_response_time) > self.watchdog_timeout_s
