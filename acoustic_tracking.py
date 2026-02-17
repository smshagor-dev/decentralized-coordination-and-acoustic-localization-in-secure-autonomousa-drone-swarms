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
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import correlate


SPEED_OF_SOUND_MPS = 343.0


@dataclass
class AudioSensor:
    drone_id: int
    position_xy: Tuple[float, float]


class CrossCorrelationEngine:
    """Delay estimation with GCC-PHAT backed by scipy correlation."""

    def estimate_delay_seconds(self, sig_i: np.ndarray, sig_j: np.ndarray, sample_rate_hz: float) -> float:
        sig_i = np.asarray(sig_i, dtype=np.float64)
        sig_j = np.asarray(sig_j, dtype=np.float64)
        if sig_i.size == 0 or sig_j.size == 0 or sample_rate_hz <= 0:
            return 0.0

        # GCC-PHAT in frequency domain.
        n = sig_i.size + sig_j.size
        f_i = np.fft.rfft(sig_i, n=n)
        f_j = np.fft.rfft(sig_j, n=n)
        cross = f_i * np.conjugate(f_j)
        cross /= (np.abs(cross) + 1e-12)
        gcc = np.fft.irfft(cross, n=n)
        max_shift = n // 2
        gcc = np.concatenate((gcc[-max_shift:], gcc[: max_shift + 1]))
        gcc_shift = int(np.argmax(np.abs(gcc)) - max_shift)

        # Direct cross-correlation estimate.
        corr = correlate(sig_j, sig_i, mode="full", method="fft")
        lag = int(np.argmax(corr) - (sig_i.size - 1))
        peak_direct = float(np.max(np.abs(corr))) if corr.size else 0.0
        peak_gcc = float(np.max(np.abs(gcc))) if gcc.size else 0.0

        shift = lag if peak_direct >= peak_gcc else gcc_shift
        return float(shift) / float(sample_rate_hz)


class TDOAEstimator:
    def __init__(self, correlation_engine: Optional[CrossCorrelationEngine] = None):
        self.correlation_engine = correlation_engine or CrossCorrelationEngine()

    def estimate_delays(
        self,
        signals: Dict[int, np.ndarray],
        sample_rate_hz: float,
        reference_id: Optional[int] = None,
    ) -> Dict[int, float]:
        if not signals:
            return {}
        ids = sorted(signals.keys())
        ref_id = int(reference_id) if reference_id is not None else ids[0]
        ref_signal = signals.get(ref_id)
        if ref_signal is None:
            return {}
        delays = {ref_id: 0.0}
        for drone_id in ids:
            if drone_id == ref_id:
                continue
            delay = self.correlation_engine.estimate_delay_seconds(ref_signal, signals[drone_id], sample_rate_hz)
            delays[drone_id] = delay
        return delays


class AcousticFusionEngine:
    """Estimate source location in 2D from pairwise TDOA constraints."""

    def estimate_source_xy(
        self,
        sensor_positions: Dict[int, Tuple[float, float]],
        delays_by_sensor: Dict[int, float],
    ) -> Tuple[Optional[Tuple[float, float]], float, float]:
        if len(sensor_positions) < 3 or len(delays_by_sensor) < 3:
            return None, 0.0, float("inf")

        ids = sorted(set(sensor_positions.keys()) & set(delays_by_sensor.keys()))
        if len(ids) < 3:
            return None, 0.0, float("inf")
        ref = ids[0]
        x0 = np.mean([sensor_positions[i][0] for i in ids])
        y0 = np.mean([sensor_positions[i][1] for i in ids])

        def residuals(state: np.ndarray) -> np.ndarray:
            xs, ys = float(state[0]), float(state[1])
            xi, yi = sensor_positions[ref]
            dist_i = math.hypot(xs - xi, ys - yi)
            res = []
            for j in ids[1:]:
                xj, yj = sensor_positions[j]
                dist_j = math.hypot(xs - xj, ys - yj)
                lhs = dist_j - dist_i
                rhs = SPEED_OF_SOUND_MPS * float(delays_by_sensor[j] - delays_by_sensor[ref])
                res.append(lhs - rhs)
            return np.asarray(res, dtype=np.float64)

        try:
            starts = [np.asarray([x0, y0], dtype=np.float64)]
            for i in ids:
                sx, sy = sensor_positions[i]
                starts.append(np.asarray([sx, sy], dtype=np.float64))
                starts.append(np.asarray([sx + 10.0, sy + 10.0], dtype=np.float64))
                starts.append(np.asarray([sx - 10.0, sy - 10.0], dtype=np.float64))

            best = None
            best_rmse = float("inf")
            for start in starts:
                fit = least_squares(residuals, start, method="trf", loss="soft_l1")
                if not fit.success:
                    continue
                rmse = math.sqrt(float(np.mean(np.square(fit.fun)))) if fit.fun.size else 0.0
                if rmse < best_rmse:
                    best = fit
                    best_rmse = rmse

            if best is None:
                return None, 0.0, float("inf")

            xy = (float(best.x[0]), float(best.x[1]))
            rmse = float(best_rmse)
            confidence = 1.0 / (1.0 + rmse / 6.0)
            confidence = max(0.0, min(1.0, confidence))
            return xy, confidence, rmse
        except Exception:
            return None, 0.0, float("inf")


class AcousticTrackingSystem:
    def __init__(self):
        self.tdoa_estimator = TDOAEstimator()
        self.fusion_engine = AcousticFusionEngine()
        self.logger = logging.getLogger("AcousticTracking")

    def localize(
        self,
        signals: Dict[int, np.ndarray],
        sensor_positions: Dict[int, Tuple[float, float]],
        sample_rate_hz: float,
        total_round_trip_ms: float,
        acoustic_latency_limit_ms: float,
    ) -> dict:
        ids = sorted(set(signals.keys()) & set(sensor_positions.keys()))
        if len(ids) < 3:
            return {
                "detected": False,
                "reason": "insufficient_sensors",
                "confidence": 0.0,
                "local_only": False,
            }

        local_only = float(total_round_trip_ms) > float(acoustic_latency_limit_ms)
        if local_only:
            ids = ids[:3]
            self.logger.warning(
                "Acoustic localization latency exceeded %.1fms; using local-only estimate",
                acoustic_latency_limit_ms,
            )

        used_signals = {i: signals[i] for i in ids}
        used_positions = {i: sensor_positions[i] for i in ids}
        delays = self.tdoa_estimator.estimate_delays(used_signals, sample_rate_hz=sample_rate_hz)
        source_xy, confidence, rmse = self.fusion_engine.estimate_source_xy(used_positions, delays)

        if source_xy is None:
            return {
                "detected": False,
                "reason": "solver_failed",
                "confidence": 0.0,
                "local_only": local_only,
            }

        return {
            "detected": True,
            "source_position": {"x": source_xy[0], "y": source_xy[1]},
            "confidence": float(confidence),
            "rmse": float(rmse),
            "delays": {str(k): float(v) for k, v in delays.items()},
            "local_only": local_only,
            "used_sensor_ids": ids,
        }
