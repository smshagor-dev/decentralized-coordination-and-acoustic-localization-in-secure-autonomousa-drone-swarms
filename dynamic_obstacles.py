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
import random
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class MotionType(str, Enum):
    LINEAR = "linear"
    CIRCULAR = "circular"
    RANDOM_WALK = "random_walk"


@dataclass
class ObstacleState:
    obstacle_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 8.0
    motion_type: MotionType = MotionType.LINEAR
    ax: float = 0.0
    ay: float = 0.0
    z: float = 0.0
    theta: float = 0.0
    omega: float = 0.3
    center_x: float = 0.0
    center_y: float = 0.0
    walk_jitter: float = 1.2
    last_update: float = field(default_factory=time.time)

    def as_dict(self) -> dict:
        return {
            "id": self.obstacle_id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "vx": self.vx,
            "vy": self.vy,
            "ax": self.ax,
            "ay": self.ay,
            "radius": self.radius,
            "motion_type": self.motion_type.value,
        }


class ObstacleTracker:
    def update(self, obstacle: ObstacleState, now: Optional[float] = None):
        t_now = now if now is not None else time.time()
        dt = max(0.0, t_now - obstacle.last_update)
        if dt <= 0.0:
            return

        if obstacle.motion_type == MotionType.LINEAR:
            obstacle.x += obstacle.vx * dt
            obstacle.y += obstacle.vy * dt
        elif obstacle.motion_type == MotionType.CIRCULAR:
            radius = max(1.0, math.hypot(obstacle.x - obstacle.center_x, obstacle.y - obstacle.center_y))
            obstacle.theta += obstacle.omega * dt
            obstacle.x = obstacle.center_x + radius * math.cos(obstacle.theta)
            obstacle.y = obstacle.center_y + radius * math.sin(obstacle.theta)
            obstacle.vx = -radius * obstacle.omega * math.sin(obstacle.theta)
            obstacle.vy = radius * obstacle.omega * math.cos(obstacle.theta)
        else:
            # Random-walk keeps smooth drift by perturbing acceleration lightly.
            obstacle.ax += random.uniform(-obstacle.walk_jitter, obstacle.walk_jitter) * 0.15
            obstacle.ay += random.uniform(-obstacle.walk_jitter, obstacle.walk_jitter) * 0.15
            obstacle.ax = max(-2.5, min(2.5, obstacle.ax))
            obstacle.ay = max(-2.5, min(2.5, obstacle.ay))
            obstacle.vx += obstacle.ax * dt
            obstacle.vy += obstacle.ay * dt
            speed = math.hypot(obstacle.vx, obstacle.vy)
            if speed > 18.0:
                scale = 18.0 / speed
                obstacle.vx *= scale
                obstacle.vy *= scale
            obstacle.x += obstacle.vx * dt
            obstacle.y += obstacle.vy * dt

        obstacle.last_update = t_now


class TrajectoryEstimator:
    def predict(self, obstacle: ObstacleState, horizon_s: float = 3.0, dt: float = 0.3) -> List[Tuple[float, float]]:
        samples: List[Tuple[float, float]] = []
        steps = max(1, int(horizon_s / max(0.05, dt)))
        for i in range(1, steps + 1):
            t = i * dt
            if abs(obstacle.ax) > 1e-6 or abs(obstacle.ay) > 1e-6:
                px = obstacle.x + obstacle.vx * t + 0.5 * obstacle.ax * (t ** 2)
                py = obstacle.y + obstacle.vy * t + 0.5 * obstacle.ay * (t ** 2)
            else:
                px = obstacle.x + obstacle.vx * t
                py = obstacle.y + obstacle.vy * t
            samples.append((px, py))
        return samples


class PathReplanner:
    def replan_target(
        self,
        drone_pos: Tuple[float, float, float],
        v_new: Tuple[float, float, float],
        lookahead_s: float = 1.2,
    ) -> Tuple[float, float, float]:
        x, y, z = drone_pos
        return (
            x + v_new[0] * lookahead_s,
            y + v_new[1] * lookahead_s,
            max(1.0, z + v_new[2] * lookahead_s),
        )


class AvoidanceController:
    def blend_velocity(
        self,
        current_v: Tuple[float, float, float],
        v_goal: Tuple[float, float, float],
        v_avoidance: Tuple[float, float, float],
        max_accel: float,
        smooth_factor: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        # Requested model: v_new = v_goal + v_avoidance
        desired = (
            v_goal[0] + v_avoidance[0],
            v_goal[1] + v_avoidance[1],
            v_goal[2] + v_avoidance[2],
        )

        # Smooth steering.
        alpha = max(0.0, min(1.0, smooth_factor))
        blended = (
            current_v[0] + (desired[0] - current_v[0]) * alpha,
            current_v[1] + (desired[1] - current_v[1]) * alpha,
            current_v[2] + (desired[2] - current_v[2]) * alpha,
        )

        # Acceleration limiting.
        ax = (blended[0] - current_v[0]) / max(0.01, dt)
        ay = (blended[1] - current_v[1]) / max(0.01, dt)
        az = (blended[2] - current_v[2]) / max(0.01, dt)
        accel = math.sqrt(ax * ax + ay * ay + az * az)
        if accel <= max_accel:
            return blended

        scale = max_accel / accel
        return (
            current_v[0] + (blended[0] - current_v[0]) * scale,
            current_v[1] + (blended[1] - current_v[1]) * scale,
            current_v[2] + (blended[2] - current_v[2]) * scale,
        )


class DynamicObstaclePredictor:
    """
    Lightweight personal predictor:
    - Short-horizon kinematic prediction
    - Learned aggressiveness score from near-miss history
    - Collision probability + safe avoidance vector output
    """

    def __init__(self):
        self.logger = logging.getLogger("DynamicObstaclePredictor")
        self._risk_memory: Dict[int, float] = {}

    def _update_motion_pattern(self, obstacle: ObstacleState):
        speed = math.hypot(obstacle.vx, obstacle.vy)
        accel = math.hypot(obstacle.ax, obstacle.ay)
        prior = self._risk_memory.get(obstacle.obstacle_id, 0.3)
        learned = 0.86 * prior + 0.14 * min(1.0, (speed / 20.0) + (accel / 6.0))
        self._risk_memory[obstacle.obstacle_id] = learned

    def _collision_cone_probability(
        self,
        drone_pos: Tuple[float, float, float],
        drone_vel: Tuple[float, float, float],
        obstacle: ObstacleState,
    ) -> float:
        rx = obstacle.x - drone_pos[0]
        ry = obstacle.y - drone_pos[1]
        dist = math.hypot(rx, ry)
        if dist < 1e-3:
            return 1.0

        rvx = obstacle.vx - drone_vel[0]
        rvy = obstacle.vy - drone_vel[1]
        rel_speed = math.hypot(rvx, rvy)
        if rel_speed < 1e-3:
            return 0.0

        cone_half = math.asin(min(0.999, max(0.0, obstacle.radius / max(obstacle.radius + 1.0, dist))))
        dir_to_obs_x = rx / dist
        dir_to_obs_y = ry / dist
        rel_dir_x = rvx / rel_speed
        rel_dir_y = rvy / rel_speed
        dot = max(-1.0, min(1.0, rel_dir_x * dir_to_obs_x + rel_dir_y * dir_to_obs_y))
        angle = math.acos(dot)
        if angle >= cone_half:
            return 0.0

        # Time to closest approach to reject far-future cone intersections.
        rel_pos_dot_rel_vel = rx * rvx + ry * rvy
        t_ca = -rel_pos_dot_rel_vel / max(1e-6, rel_speed * rel_speed)
        if t_ca < 0.0 or t_ca > 4.0:
            return 0.0
        closeness = max(0.0, 1.0 - (angle / max(1e-3, cone_half)))
        time_factor = max(0.2, 1.0 - (t_ca / 4.0))
        return min(1.0, closeness * time_factor)

    def predict_for_drone(
        self,
        drone_pos: Tuple[float, float, float],
        drone_vel: Tuple[float, float, float],
        obstacles: List[ObstacleState],
        estimator: TrajectoryEstimator,
        horizon_s: float = 3.0,
    ) -> dict:
        if not obstacles:
            return {
                "collision_probability": 0.0,
                "avoidance_vector": (0.0, 0.0, 0.0),
                "predictions": {},
            }

        px, py, pz = drone_pos
        vx, vy, _ = drone_vel
        drone_speed = max(0.5, math.hypot(vx, vy))

        max_prob = 0.0
        combined_avoid_x = 0.0
        combined_avoid_y = 0.0
        cone_max_prob = 0.0
        threat_obstacle_id: Optional[int] = None
        confidence_terms: List[float] = []
        preds: Dict[int, List[Tuple[float, float]]] = {}

        for obstacle in obstacles:
            self._update_motion_pattern(obstacle)
            predicted = estimator.predict(obstacle, horizon_s=horizon_s, dt=0.3)
            preds[obstacle.obstacle_id] = predicted
            learned_risk = self._risk_memory.get(obstacle.obstacle_id, 0.3)
            cone_prob = self._collision_cone_probability(drone_pos, drone_vel, obstacle)
            cone_max_prob = max(cone_max_prob, cone_prob)
            confidence_terms.append(min(1.0, 0.45 + 0.35 * learned_risk + 0.2 * cone_prob))

            obstacle_prob = 0.0
            avoid_x = 0.0
            avoid_y = 0.0
            for i, (ox, oy) in enumerate(predicted, start=1):
                t = i * 0.3
                dx = ox - (px + vx * t)
                dy = oy - (py + vy * t)
                dist = math.hypot(dx, dy)
                safe_dist = obstacle.radius + 8.0 + drone_speed * 0.25
                if dist >= safe_dist:
                    continue

                proximity = 1.0 - (dist / safe_dist)
                time_factor = max(0.1, 1.0 - (t / max(1.0, horizon_s)))
                prob = min(1.0, proximity * 0.7 + learned_risk * 0.3) * time_factor
                obstacle_prob = max(obstacle_prob, prob)

                # Perpendicular escape vector from obstacle direction.
                inv = 1.0 / max(0.1, dist)
                away_x = -dx * inv
                away_y = -dy * inv
                perp_x, perp_y = -away_y, away_x
                avoid_x += perp_x * prob * 6.0
                avoid_y += perp_y * prob * 6.0

            if obstacle_prob > max_prob:
                max_prob = obstacle_prob
                threat_obstacle_id = obstacle.obstacle_id
            combined_avoid_x += avoid_x
            combined_avoid_y += avoid_y

        ml_confidence = sum(confidence_terms) / max(1, len(confidence_terms))
        return {
            "collision_probability": min(1.0, max_prob),
            "collision_cone_probability": min(1.0, cone_max_prob),
            "avoidance_vector": (combined_avoid_x, combined_avoid_y, 0.0),
            "ml_confidence": min(1.0, max(0.0, ml_confidence)),
            "threat_obstacle_id": threat_obstacle_id,
            "predictions": preds,
        }


class ObstacleManager:
    def __init__(self):
        self._obstacles: Dict[int, ObstacleState] = {}
        self._next_id = 1
        self._lock = threading.RLock()
        self.tracker = ObstacleTracker()
        self.logger = logging.getLogger("ObstacleManager")

    def add_obstacle(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        motion_type: MotionType = MotionType.LINEAR,
        radius: float = 8.0,
        z: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
    ) -> int:
        with self._lock:
            obstacle_id = self._next_id
            self._next_id += 1
            state = ObstacleState(
                obstacle_id=obstacle_id,
                x=float(x),
                y=float(y),
                z=float(z),
                vx=float(vx),
                vy=float(vy),
                ax=float(ax),
                ay=float(ay),
                radius=float(radius),
                motion_type=motion_type,
                center_x=float(x),
                center_y=float(y),
                theta=0.0,
            )
            self._obstacles[obstacle_id] = state
            self.logger.info(
                "Dynamic obstacle added id=%s type=%s start=(%.1f, %.1f) vel=(%.1f, %.1f)",
                obstacle_id,
                motion_type.value,
                x,
                y,
                vx,
                vy,
            )
            return obstacle_id

    def clear(self):
        with self._lock:
            self._obstacles.clear()

    def update(self):
        with self._lock:
            now = time.time()
            for obstacle in self._obstacles.values():
                self.tracker.update(obstacle, now=now)

    def get_obstacles(self) -> List[ObstacleState]:
        with self._lock:
            return list(self._obstacles.values())

    def get_obstacles_as_dict(self) -> List[dict]:
        with self._lock:
            return [o.as_dict() for o in self._obstacles.values()]
