from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

from drone import Drone, Position, FlightMode, DroneRole


class DroneOperationalState(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    WAITING_FOR_COMMAND = "WAITING_FOR_COMMAND"
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    RETURNING_HOME = "RETURNING_HOME"
    GPS_ML_ACTIVE = "GPS_ML_ACTIVE"


@dataclass
class CommandEvent:
    command: str
    payload: Dict[str, Any]
    issued_by: int
    issued_at: float


class DroneStateManager:
    """Thread-safe state tracker that is independent from low-level flight modes."""

    def __init__(self):
        self._states: Dict[int, DroneOperationalState] = {}
        self._lock = threading.RLock()
        self.logger = logging.getLogger("DroneStateManager")

    def init_drone(self, drone_id: int):
        with self._lock:
            self._states[drone_id] = DroneOperationalState.IDLE

    def remove_drone(self, drone_id: int):
        with self._lock:
            self._states.pop(drone_id, None)

    def get_state(self, drone_id: int) -> DroneOperationalState:
        with self._lock:
            return self._states.get(drone_id, DroneOperationalState.IDLE)

    def set_state(self, drone_id: int, new_state: DroneOperationalState):
        with self._lock:
            old_state = self._states.get(drone_id, DroneOperationalState.IDLE)
            if old_state == new_state:
                return
            self._states[drone_id] = new_state
            self.logger.info(
                "Drone %s state transition %s -> %s",
                drone_id,
                old_state.value,
                new_state.value,
            )

    def is_gps_ml_active(self, drone_id: int) -> bool:
        return self.get_state(drone_id) == DroneOperationalState.GPS_ML_ACTIVE


class GPSNavigationModule:
    """GPS mode gate and coordinate routing policy."""

    def is_active(self, payload: Dict[str, Any], drone_id: int) -> bool:
        gps_mode_map = payload.get("gps_mode_map") or {}
        if drone_id in gps_mode_map:
            return bool(gps_mode_map[drone_id])
        return bool(payload.get("gps_active", False))


class MLNavigationModule:
    """Personal ML navigation with safe fallback to default goto logic."""

    def __init__(self):
        self.logger = logging.getLogger("MLNavigationModule")

    def navigate(self, drone: Drone, target: Position) -> bool:
        ml = getattr(drone, "ml_system", None)
        if ml is None:
            return drone.goto(target)

        current = (
            drone.current_position.x,
            drone.current_position.y,
            drone.current_position.z,
        )
        velocity = (drone.velocity.x, drone.velocity.y, drone.velocity.z)
        risk = ml.predict_collision_risk(current, velocity)
        path_collision = ml.check_path_collision(current, (target.x, target.y, target.z))
        if risk > 0.8 or path_collision:
            suggested = ml.suggest_avoidance_maneuver(
                current,
                velocity,
                (target.x, target.y, target.z),
            )
            alt_target = Position(
                drone.current_position.x + suggested[0],
                drone.current_position.y + suggested[1],
                max(1.0, drone.current_position.z + suggested[2]),
            )
            self.logger.info(
                "Drone %s ML adjusted route (risk=%.2f, collision=%s)",
                drone.drone_id,
                risk,
                path_collision,
            )
            return drone.goto(alt_target)
        return drone.goto(target)


class CommunicationManager:
    """Thread-safe event bus for command and status propagation."""

    def __init__(self):
        self._subs: Dict[str, List[Callable[[Dict[str, Any]], None]]] = {}
        self._queue: "queue.Queue[Optional[tuple[str, Dict[str, Any]]]]" = queue.Queue()
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.logger = logging.getLogger("SwarmEventBus")

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._queue.put(None)
        if self._thread:
            self._thread.join(timeout=2.0)

    def subscribe(self, event_name: str, handler: Callable[[Dict[str, Any]], None]):
        with self._lock:
            self._subs.setdefault(event_name, []).append(handler)

    def publish(self, event_name: str, payload: Dict[str, Any]):
        self._queue.put((event_name, payload))

    def _loop(self):
        while self._running:
            item = self._queue.get()
            if item is None:
                continue
            event_name, payload = item
            with self._lock:
                handlers = list(self._subs.get(event_name, []))
            for handler in handlers:
                try:
                    handler(payload)
                except Exception as exc:
                    self.logger.error("Event handler failed for %s: %s", event_name, exc)


class LeaderCommandHandler:
    """Leader-only command entrypoint. Followers react only through this handler."""

    def __init__(self, swarm: "SwarmManagerProtocol"):
        self.swarm = swarm
        self.logger = logging.getLogger("LeaderCommandHandler")

    def issue_takeoff(self):
        leader = self.swarm.get_leader()
        if leader is None:
            self.logger.warning("Takeoff command rejected: no leader")
            return
        event = CommandEvent(
            command="TAKEOFF",
            payload={},
            issued_by=leader.drone_id,
            issued_at=time.time(),
        )
        self.swarm.communication_manager.publish("LEADER_COMMAND", event.__dict__)

    def issue_move_to_target(
        self,
        targets: Dict[int, Position],
        gps_mode_map: Optional[Dict[int, bool]] = None,
    ):
        leader = self.swarm.get_leader()
        if leader is None:
            self.logger.warning("Move command rejected: no leader")
            return
        payload = {
            "targets": {str(k): {"x": v.x, "y": v.y, "z": v.z} for k, v in targets.items()},
            "gps_mode_map": gps_mode_map or {},
        }
        event = CommandEvent(
            command="MOVE_TO_TARGET",
            payload=payload,
            issued_by=leader.drone_id,
            issued_at=time.time(),
        )
        self.swarm.communication_manager.publish("LEADER_COMMAND", event.__dict__)

    def issue_return_to_home(self):
        leader = self.swarm.get_leader()
        if leader is None:
            self.logger.warning("RETURN_TO_HOME rejected: no leader")
            return
        event = CommandEvent(
            command="RETURN_TO_HOME",
            payload={},
            issued_by=leader.drone_id,
            issued_at=time.time(),
        )
        self.swarm.communication_manager.publish("LEADER_COMMAND", event.__dict__)


class SwarmManagerProtocol:
    """Minimal protocol type for LeaderCommandHandler."""

    communication_manager: CommunicationManager

    def get_leader(self) -> Optional[Drone]:
        raise NotImplementedError
