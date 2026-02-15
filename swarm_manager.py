"""
Swarm Manager - Manages drone fleet with leader election and fault tolerance
"""

import time
import threading
import logging
import math
import queue
import random
from typing import List, Optional, Dict
from drone import Drone, DroneRole, FlightMode, Position
from dynamic_obstacles import (
    AvoidanceController,
    DynamicObstaclePredictor,
    MotionType,
    ObstacleManager,
    PathReplanner,
    TrajectoryEstimator,
)
from latency_monitor import LatencyMonitor, MLBridge
from leader_follower_logic import (
    CommunicationManager as EventCommunicationManager,
    DroneOperationalState,
    DroneStateManager,
    GPSNavigationModule,
    LeaderCommandHandler,
    MLNavigationModule,
)

class SwarmManager:
    """
    Manages the drone swarm with automatic leader election
    and fault tolerance
    """
    
    HEARTBEAT_TIMEOUT = 5.0  # seconds
    ELECTION_TIMEOUT = 3.0   # seconds
    
    def __init__(self):
        self.drones: Dict[int, Drone] = {}
        self.leader_id: Optional[int] = None
        self.election_in_progress = False
        self.running = False
        self.leader_follow_pattern = "v"   # "v" or "line"
        self.follow_spacing_m = 45.0
        self.leader_follow_enabled = False
        self._lock = threading.RLock()
        
        # Communication
        self.heartbeats: Dict[int, float] = {}
        self.reported_failures = set()
        
        # Thread management
        self.monitor_thread = None
        self._mission_targets: Dict[int, Position] = {}
        self._mission_active = False
        self._mission_arrival_threshold_m = 6.0
        self._event_notifications: "queue.Queue[dict]" = queue.Queue()
        self._avoidance_active_ids = set()

        # Dynamic obstacle prediction and avoidance stack
        self.obstacle_manager = ObstacleManager()
        self.dynamic_predictor = DynamicObstaclePredictor()
        self.trajectory_estimator = TrajectoryEstimator()
        self.path_replanner = PathReplanner()
        self.avoidance_controller = AvoidanceController()
        self.dynamic_collision_threshold = 0.42
        self.use_personal_ml_avoidance = True

        # C++ <-> Python latency monitor and bridge hooks
        self.latency_monitor = LatencyMonitor(window_size=120, latency_threshold_ms=220.0)
        self.ml_bridge = MLBridge(self.latency_monitor, watchdog_timeout_s=1.8)
        self.fallback_local_avoidance_mode = False
        self.per_drone_latency_threshold_ms: Dict[int, float] = {}

        # Event-driven leader/follower architecture
        self.communication_manager = EventCommunicationManager()
        self.drone_state_manager = DroneStateManager()
        self.gps_navigation_module = GPSNavigationModule()
        self.ml_navigation_module = MLNavigationModule()
        self.leader_command_handler = LeaderCommandHandler(self)
        
        # Logging
        self.logger = logging.getLogger("SwarmManager")
        self.setup_logging()
        self._register_event_handlers()
        
        self.logger.info("Swarm Manager initialized")
    
    def setup_logging(self):
        """Configure logging"""
        if self.logger.handlers:
            return
        handler = logging.FileHandler('logs/swarm_manager.log')
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # Console handler
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        console.setFormatter(logging.Formatter('%(name)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(console)
        self.logger.propagate = False

    def _register_event_handlers(self):
        """Register event handlers for leader commands and mission updates."""
        self.communication_manager.subscribe("LEADER_COMMAND", self._on_leader_command)
        self.communication_manager.subscribe("MISSION_COMPLETE", self._on_mission_complete)

    def _push_system_event(self, event: dict):
        """Push internal swarm event for GUI/system audit stream."""
        try:
            self._event_notifications.put_nowait(event)
        except Exception:
            pass

    def drain_system_events(self, max_items: int = 100) -> List[dict]:
        """Drain queued swarm events for external log/audit consumers."""
        events: List[dict] = []
        for _ in range(max_items):
            try:
                events.append(self._event_notifications.get_nowait())
            except queue.Empty:
                break
        return events

    def _on_leader_command(self, event: dict):
        """Event handler: execute leader-issued commands."""
        command = str(event.get("command", "")).strip().upper()
        payload = event.get("payload", {}) or {}
        self._push_system_event(
            {
                "kind": "command",
                "command": command,
                "payload": payload,
                "issued_by": event.get("issued_by"),
            }
        )
        self.logger.info(
            "Leader command received: %s by drone=%s",
            command,
            event.get("issued_by"),
        )
        if command == "TAKEOFF":
            self._execute_leader_takeoff()
        elif command == "MOVE_TO_TARGET":
            self._execute_leader_move(payload)
        elif command == "RETURN_TO_HOME":
            self._execute_leader_return_home()

    def _on_mission_complete(self, event: dict):
        """Event handler: first arrival triggers leader-broadcast return-to-home."""
        reached_drone_id = int(event.get("drone_id", -1))
        target = event.get("target", {})
        self._push_system_event(
            {
                "kind": "message",
                "message_type": "MISSION_COMPLETE",
                "data": event,
            }
        )
        self._push_system_event(
            {
                "kind": "message",
                "message_type": "LEADER_BROADCAST",
                "data": {
                    "action": "RETURN_TO_HOME",
                    "reason": f"MISSION_COMPLETE by Drone {reached_drone_id}",
                    "target": target,
                },
            }
        )
        self.logger.info(
            "MISSION_COMPLETE received from Drone %s target=%s -> leader broadcasting RETURN_TO_HOME",
            reached_drone_id,
            target,
        )
        self.leader_command_handler.issue_return_to_home()

    def _execute_leader_takeoff(self):
        """Leader-issued coordinated takeoff with follower hover-at-own-position policy."""
        with self._lock:
            for drone in self.drones.values():
                if not drone.is_armed:
                    drone.arm()
                if drone.takeoff():
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.TAKEOFF)

    def _execute_leader_move(self, payload: dict):
        """Leader-issued movement command: every move is explicit and target-driven."""
        with self._lock:
            raw_targets = payload.get("targets", {}) or {}
            if not raw_targets:
                return

            self._mission_targets.clear()
            self._mission_active = True
            for drone_id_text, raw in raw_targets.items():
                try:
                    drone_id = int(drone_id_text)
                except (TypeError, ValueError):
                    continue
                drone = self.drones.get(drone_id)
                if drone is None:
                    continue
                target = Position(
                    float(raw.get("x", drone.current_position.x)),
                    float(raw.get("y", drone.current_position.y)),
                    float(raw.get("z", max(1.0, drone.current_position.z))),
                )
                if self._command_move_for_drone(drone, target, payload):
                    self._mission_targets[drone_id] = target

    def _execute_leader_return_home(self):
        """Leader broadcasted return home; GPS+ML active drones ignore this by requirement."""
        with self._lock:
            for drone in self.drones.values():
                if self.drone_state_manager.is_gps_ml_active(drone.drone_id) or drone.area_mission.active:
                    self.logger.info(
                        "Drone %s ignored RETURN_TO_HOME due to GPS_ML_ACTIVE state",
                        drone.drone_id,
                    )
                    continue
                drone.return_to_home("Leader broadcast RETURN_TO_HOME")
                self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.RETURNING_HOME)
                self._push_system_event(
                    {
                        "kind": "command",
                        "command": "RETURN_TO_HOME",
                        "payload": {
                            "target_drone_id": drone.drone_id,
                            "reason": "Leader broadcast RETURN_TO_HOME",
                        },
                        "issued_by": self.leader_id,
                    }
                )

            self._mission_active = False
            self._mission_targets.clear()

    def _command_move_for_drone(self, drone: Drone, target: Position, payload: dict) -> bool:
        """Route movement through GPS+ML module if active, else default swarm goto."""
        if not self._prepare_drone_for_goto(drone, preferred_alt=target.z):
            return False

        if drone.area_mission.active or self.gps_navigation_module.is_active(payload, drone.drone_id):
            ok = self.ml_navigation_module.navigate(drone, target)
            if ok:
                self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.GPS_ML_ACTIVE)
            return ok

        ok = drone.goto(target)
        if ok:
            self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.MOVING_TO_TARGET)
        return ok
    
    def add_drone(self, drone: Drone) -> bool:
        """Add drone to the swarm"""
        with self._lock:
            if drone.drone_id in self.drones:
                self.logger.warning(f"Drone {drone.drone_id} already in swarm")
                return False
            
            self.drones[drone.drone_id] = drone
            self.heartbeats[drone.drone_id] = time.time()
            self.reported_failures = {
                item for item in self.reported_failures if item[0] != drone.drone_id
            }
            self.drone_state_manager.init_drone(drone.drone_id)
            
            # Start drone systems
            drone.start()
            
            self.logger.info(f"Drone {drone.drone_id} added to swarm")
            
            # If no leader, elect one
            if self.leader_id is None and len(self.drones) > 0:
                self.elect_leader()
            
            return True
    
    def remove_drone(self, drone_id: int) -> bool:
        """Remove drone from swarm"""
        with self._lock:
            if drone_id not in self.drones:
                return False
            
            drone = self.drones[drone_id]
            drone.stop()
            
            del self.drones[drone_id]
            self.drone_state_manager.remove_drone(drone_id)
            if drone_id in self.heartbeats:
                del self.heartbeats[drone_id]
            self.reported_failures = {
                item for item in self.reported_failures if item[0] != drone_id
            }
            
            self.logger.info(f"Drone {drone_id} removed from swarm")
            
            # If removed drone was leader, elect new leader
            if drone_id == self.leader_id:
                self.logger.warning(f"Leader drone {drone_id} removed - electing new leader")
                self.leader_id = None
                if len(self.drones) > 0:
                    self.elect_leader()
            
            return True
    
    def start(self):
        """Start swarm monitoring"""
        if not self.running:
            self.running = True
            self.communication_manager.start()
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            self.logger.info("Swarm monitoring started")
    
    def stop(self):
        """Stop swarm operations"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self.communication_manager.stop()
        
        # Stop all drones
        for drone in self.drones.values():
            drone.stop()
        
        self.logger.info("Swarm monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            self.obstacle_manager.update()
            self._update_adaptive_latency_thresholds()

            # Check heartbeats
            self._check_heartbeats()
            
            # Monitor leader
            if self.leader_id is not None:
                self._monitor_leader()
            
            # Check for failed drones
            self._check_drone_status()

            # Explicit-command architecture: no autonomous leader-follow movement.
            # Personal ML based close-range separation
            self._apply_personal_ml_separation()
            self._apply_dynamic_obstacle_avoidance()

            # Bridge latency heartbeat.
            try:
                latency_stats = self.ml_bridge.round_trip()
            except Exception as exc:
                self.logger.warning("MLBridge round-trip failed: %s", exc)
                latency_stats = self.latency_monitor.get_stats()

            watchdog_timed_out = self.ml_bridge.is_watchdog_timed_out()
            if latency_stats.get("fallback_required", False) or watchdog_timed_out:
                if not self.fallback_local_avoidance_mode:
                    reason = "watchdog timeout" if watchdog_timed_out else "latency threshold"
                    self.logger.warning("Switching to fallback local avoidance due to %s", reason)
                self.fallback_local_avoidance_mode = True
                self._push_system_event(
                    {
                        "kind": "warning",
                        "message_type": "ML_BRIDGE_TIMEOUT" if watchdog_timed_out else "LATENCY_SPIKE",
                        "data": {
                            **latency_stats,
                            "watchdog_timed_out": watchdog_timed_out,
                        },
                    }
                )
            else:
                self.fallback_local_avoidance_mode = False

            # Keep high-level state model and event-driven mission completion in sync.
            self._synchronize_operational_states()
            self._check_mission_arrivals()
            
            time.sleep(1.0)

    def _synchronize_operational_states(self):
        """Map low-level flight mode to high-level operational state."""
        with self._lock:
            for drone in self.drones.values():
                current_state = self.drone_state_manager.get_state(drone.drone_id)
                if drone.flight_mode == FlightMode.IDLE:
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.IDLE)
                    continue
                if drone.flight_mode == FlightMode.TAKEOFF:
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.TAKEOFF)
                    continue
                if drone.flight_mode == FlightMode.RETURNING_HOME:
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.RETURNING_HOME)
                    continue
                if drone.drone_id in self._avoidance_active_ids:
                    self.drone_state_manager.set_state(
                        drone.drone_id, DroneOperationalState.AVOIDING_DYNAMIC_OBSTACLE
                    )
                    continue
                if current_state == DroneOperationalState.GPS_ML_ACTIVE:
                    continue
                if drone.role == DroneRole.FOLLOWER and drone.flight_mode == FlightMode.HOVER:
                    self.drone_state_manager.set_state(
                        drone.drone_id, DroneOperationalState.WAITING_FOR_COMMAND
                    )
                    continue
                if drone.flight_mode == FlightMode.FLYING:
                    self.drone_state_manager.set_state(
                        drone.drone_id, DroneOperationalState.MOVING_TO_TARGET
                    )

    def _update_adaptive_latency_thresholds(self):
        """
        Adaptive per-drone threshold:
        - faster drones and drones with lower processing capability get tighter thresholds
        - keep thresholds bounded for stability
        """
        thresholds: Dict[int, float] = {}
        for drone in self.drones.values():
            speed = math.sqrt(drone.velocity.x ** 2 + drone.velocity.y ** 2 + drone.velocity.z ** 2)
            proc = float(max(1.0, getattr(drone, "processing_capability", 100.0)))
            speed_penalty = min(70.0, speed * 3.0)
            proc_penalty = min(55.0, max(0.0, (100.0 - proc) * 0.8))
            threshold_ms = max(110.0, min(280.0, 220.0 - speed_penalty - proc_penalty))
            thresholds[drone.drone_id] = threshold_ms

        self.per_drone_latency_threshold_ms = thresholds
        if thresholds:
            self.latency_monitor.set_threshold_ms(min(thresholds.values()))

    def _check_mission_arrivals(self):
        """If first drone reaches mission target Y, publish mission completion event."""
        if not self._mission_active or not self._mission_targets:
            return
        with self._lock:
            for drone_id, target in list(self._mission_targets.items()):
                drone = self.drones.get(drone_id)
                if drone is None:
                    continue
                distance = drone.current_position.distance_to(target)
                if distance > self._mission_arrival_threshold_m:
                    continue
                self.drone_state_manager.set_state(drone_id, DroneOperationalState.MISSION_COMPLETE)
                self.logger.info(
                    "MISSION_COMPLETE: Drone %s reached destination (x=%.1f, y=%.1f, z=%.1f)",
                    drone_id,
                    target.x,
                    target.y,
                    target.z,
                )
                self.communication_manager.publish(
                    "MISSION_COMPLETE",
                    {
                        "drone_id": drone_id,
                        "target": {"x": target.x, "y": target.y, "z": target.z},
                        "message": f"MISSION_COMPLETE: Drone {drone_id} reached destination",
                    },
                )
                self._mission_active = False
                break

    def _prepare_drone_for_goto(self, drone: Drone, preferred_alt: float = 10.0) -> bool:
        """Ensure drone can accept goto command with coordinated takeoff support."""
        if drone.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
            return False

        # If follower is on ground, auto-arm and start takeoff so it can join leader.
        if drone.flight_mode == FlightMode.IDLE:
            if not drone.is_armed:
                drone.arm()
            drone.takeoff()
            return False

        # While taking off, wait for a minimal safe altitude, then switch to hover.
        if drone.flight_mode == FlightMode.TAKEOFF:
            join_alt = max(2.0, min(12.0, preferred_alt * 0.25))
            if drone.current_position.z >= join_alt:
                drone.flight_mode = FlightMode.HOVER
                if drone.current_position.z < 1.0:
                    drone.current_position.z = 1.0
                return True
            return False

        return True

    def _ml_follow_leader(self):
        """
        Followers track leader command intent with formation offsets.
        Personal ML is used for safety, not to dive directly into leader position.
        """
        if not self.leader_follow_enabled:
            return
        leader = self.get_leader()
        if not leader:
            return
        if leader.flight_mode not in [FlightMode.FLYING, FlightMode.HOVER, FlightMode.TAKEOFF]:
            return

        leader_has_target = leader.target_position is not None
        speed = math.sqrt(
            leader.velocity.x ** 2 + leader.velocity.y ** 2 + leader.velocity.z ** 2
        )
        if speed < 0.05 and not leader_has_target and leader.flight_mode != FlightMode.FLYING:
            return

        followers = self.get_followers()
        if not followers:
            return

        if leader_has_target:
            base_target = leader.target_position
            heading_x = base_target.x - leader.current_position.x
            heading_y = base_target.y - leader.current_position.y
            heading_norm = math.sqrt(heading_x ** 2 + heading_y ** 2)
            if heading_norm < 0.001:
                dir_x, dir_y = (1.0, 0.0) if speed < 0.05 else (
                    leader.velocity.x / max(speed, 1e-6),
                    leader.velocity.y / max(speed, 1e-6)
                )
            else:
                dir_x, dir_y = heading_x / heading_norm, heading_y / heading_norm
            base_x = base_target.x
            base_y = base_target.y
            base_z = base_target.z
        else:
            if speed < 0.05:
                dir_x, dir_y = 1.0, 0.0
            else:
                dir_x = leader.velocity.x / speed
                dir_y = leader.velocity.y / speed
            base_x = leader.current_position.x
            base_y = leader.current_position.y
            base_z = leader.current_position.z
        perp_x, perp_y = -dir_y, dir_x
        pattern = (self.leader_follow_pattern or "v").strip().lower()
        spacing = max(15.0, float(self.follow_spacing_m))

        for i, follower in enumerate(followers):
            if follower.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
                continue

            if not self._prepare_drone_for_goto(follower, preferred_alt=leader.current_position.z):
                continue

            if pattern == "line":
                # Single-file line behind leader.
                rank = i + 1
                follow_distance = spacing * rank
                lateral_distance = 0.0
                side = 1.0
            else:
                # Clear V formation behind leader.
                side = -1.0 if i % 2 == 0 else 1.0
                rank = (i // 2) + 1
                follow_distance = spacing * rank
                lateral_distance = spacing * 0.75 * rank

            target_x = (
                base_x
                - dir_x * follow_distance
                + perp_x * side * lateral_distance
            )
            target_y = (
                base_y
                - dir_y * follow_distance
                + perp_y * side * lateral_distance
            )
            target_z = max(1.0, min(base_z, follower.MAX_ALTITUDE))

            # Personal ML feasibility check.
            can_follow = True
            ml = follower.ml_system
            if ml is not None:
                current = (
                    follower.current_position.x,
                    follower.current_position.y,
                    follower.current_position.z
                )
                target = (target_x, target_y, target_z)
                velocity = (
                    follower.velocity.x,
                    follower.velocity.y,
                    follower.velocity.z
                )
                risk = ml.predict_collision_risk(current, velocity)
                collision_on_path = ml.check_path_collision(current, target)
                if risk > 0.65 or collision_on_path:
                    can_follow = False
                    self.logger.info(
                        f"Drone {follower.drone_id} personal ML rejected follow (risk={risk:.2f}, collision={collision_on_path})"
                    )
            
            if can_follow:
                follower.goto(Position(target_x, target_y, target_z))

    def set_leader_follow_pattern(self, pattern: str, spacing_m: Optional[float] = None) -> bool:
        """Set continuous leader-follow shape used by followers."""
        selected = (pattern or "").strip().lower()
        if selected not in {"v", "line"}:
            self.logger.warning(f"Invalid leader follow pattern '{pattern}'")
            return False
        self.leader_follow_pattern = selected
        if spacing_m is not None:
            self.follow_spacing_m = max(10.0, float(spacing_m))
        self.logger.info(
            f"Leader follow pattern set to '{self.leader_follow_pattern}' spacing={self.follow_spacing_m:.1f}m"
        )
        return True

    def set_leader_follow_enabled(self, enabled: bool):
        """Enable/disable continuous leader-follow behavior."""
        self.leader_follow_enabled = bool(enabled)
        self.logger.info(f"Leader-follow enabled={self.leader_follow_enabled}")

    def _apply_personal_ml_separation(self):
        """If drones get too close, use each drone's own ML to move away."""
        active = [
            d for d in self.drones.values()
            if d.is_active and d.flight_mode not in [FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING]
        ]
        if len(active) < 2:
            return

        min_safe_distance = 8.0
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                a = active[i]
                b = active[j]
                distance = a.current_position.distance_to(b.current_position)
                if distance >= min_safe_distance:
                    continue

                dx = a.current_position.x - b.current_position.x
                dy = a.current_position.y - b.current_position.y
                norm = math.sqrt(dx * dx + dy * dy) or 1.0
                ux, uy = dx / norm, dy / norm
                push = (min_safe_distance - distance) + 4.0

                for drone, direction in [(a, 1.0), (b, -1.0)]:
                    if not self._prepare_drone_for_goto(drone):
                        continue
                    tx = drone.current_position.x + ux * push * direction
                    ty = drone.current_position.y + uy * push * direction
                    tz = max(1.0, drone.current_position.z)

                    ml = drone.ml_system
                    if ml is not None:
                        current = (
                            drone.current_position.x,
                            drone.current_position.y,
                            drone.current_position.z
                        )
                        velocity = (
                            drone.velocity.x,
                            drone.velocity.y,
                            drone.velocity.z
                        )
                        # Personal ML picks safer velocity toward tentative separation target.
                        suggested = ml.suggest_avoidance_maneuver(current, velocity, (tx, ty, tz))
                        tx = drone.current_position.x + suggested[0]
                        ty = drone.current_position.y + suggested[1]
                        tz = max(1.0, drone.current_position.z + suggested[2] * 0.3)

                    drone.goto(Position(tx, ty, min(tz, drone.MAX_ALTITUDE)))

    def _apply_dynamic_obstacle_avoidance(self):
        """Predict dynamic obstacle motion and proactively replan drone movement."""
        obstacles = self.obstacle_manager.get_obstacles()
        if not obstacles:
            self._avoidance_active_ids.clear()
            return

        obstacles_by_id = {o.obstacle_id: o for o in obstacles}
        active_ids = set()
        for drone in self.drones.values():
            if not drone.is_active:
                continue
            if drone.flight_mode in [FlightMode.IDLE, FlightMode.LANDING, FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING]:
                continue
            if not self._prepare_drone_for_goto(drone):
                continue

            target = self._mission_targets.get(drone.drone_id) or drone.target_position
            if target is None:
                continue

            current = (drone.current_position.x, drone.current_position.y, drone.current_position.z)
            velocity = (drone.velocity.x, drone.velocity.y, drone.velocity.z)
            to_goal = (
                target.x - drone.current_position.x,
                target.y - drone.current_position.y,
                target.z - drone.current_position.z,
            )
            norm_goal = math.sqrt(to_goal[0] ** 2 + to_goal[1] ** 2 + to_goal[2] ** 2) or 1.0
            measured_speed = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)
            intent_speed = min(drone.MAX_SPEED, max(2.5, norm_goal * 0.25))
            speed = min(drone.MAX_SPEED, max(measured_speed, intent_speed))
            v_goal = (
                (to_goal[0] / norm_goal) * speed,
                (to_goal[1] / norm_goal) * speed,
                (to_goal[2] / norm_goal) * min(speed, 3.0),
            )

            current_speed = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)
            prediction_velocity = velocity
            # If immediate measured speed is near zero but the drone has a target,
            # use intent velocity so static-front obstacles are still predicted early.
            if current_speed < 0.5 and target is not None:
                prediction_velocity = v_goal

            if self.fallback_local_avoidance_mode or not self.use_personal_ml_avoidance or not drone.personal_ml_enabled:
                # Geometric fallback: short side-step from nearest obstacle.
                nearest = min(
                    obstacles,
                    key=lambda o: math.hypot(drone.current_position.x - o.x, drone.current_position.y - o.y),
                )
                dx = drone.current_position.x - nearest.x
                dy = drone.current_position.y - nearest.y
                d = math.hypot(dx, dy) or 1.0
                v_avoid = ((-dy / d) * 5.0, (dx / d) * 5.0, 0.0)
                risk_result = {
                    "collision_probability": 0.65,
                    "collision_cone_probability": 0.55,
                    "ml_confidence": 0.35,
                    "predictions": {},
                }
            else:
                risk_result = self.dynamic_predictor.predict_for_drone(
                    current,
                    prediction_velocity,
                    obstacles,
                    self.trajectory_estimator,
                    horizon_s=6.0,
                )
                v_avoid = risk_result.get("avoidance_vector", (0.0, 0.0, 0.0))

            collision_prob = float(risk_result.get("collision_probability", 0.0))
            cone_prob = float(risk_result.get("collision_cone_probability", 0.0))
            ml_confidence = float(risk_result.get("ml_confidence", 0.0))
            should_avoid = (
                collision_prob > self.dynamic_collision_threshold
                or cone_prob > (self.dynamic_collision_threshold * 0.8)
            )
            if not should_avoid:
                goal = self._mission_targets.get(drone.drone_id)
                if (
                    goal is not None
                    and drone.flight_mode == FlightMode.HOVER
                    and drone.current_position.distance_to(goal) > 6.0
                ):
                    drone.goto(Position(goal.x, goal.y, goal.z))
                continue

            v_new = self.avoidance_controller.blend_velocity(
                current_v=velocity,
                v_goal=v_goal,
                v_avoidance=v_avoid,
                max_accel=float(getattr(drone, "max_lateral_accel", 4.5)),
                smooth_factor=float(getattr(drone, "steering_smooth_factor", 0.28)),
                dt=0.25,
            )
            replanned = self.path_replanner.replan_target(current, v_new, lookahead_s=2.8)

            threat_id = risk_result.get("threat_obstacle_id")
            bypass_target = None
            threat = obstacles_by_id.get(threat_id) if threat_id is not None else None
            if threat is not None:
                bypass_target = self._build_bypass_target(drone.current_position, target, threat)

            safe_target = Position(
                (bypass_target.x if bypass_target else replanned[0]),
                (bypass_target.y if bypass_target else replanned[1]),
                max(1.0, min(drone.MAX_ALTITUDE, replanned[2])),
            )
            if drone.goto(safe_target):
                active_ids.add(drone.drone_id)
                self.logger.info(
                    "Dynamic obstacle avoidance drone=%s prob=%.2f cone=%.2f conf=%.2f target=(%.1f, %.1f, %.1f)",
                    drone.drone_id,
                    collision_prob,
                    cone_prob,
                    ml_confidence,
                    safe_target.x,
                    safe_target.y,
                    safe_target.z,
                )
                self._push_system_event(
                    {
                        "kind": "path_replan",
                        "message_type": "DYNAMIC_OBSTACLE_AVOIDANCE",
                        "data": {
                            "drone_id": drone.drone_id,
                            "collision_probability": collision_prob,
                            "collision_cone_probability": cone_prob,
                            "ml_confidence": ml_confidence,
                            "fallback_mode": self.fallback_local_avoidance_mode,
                            "target": {"x": safe_target.x, "y": safe_target.y, "z": safe_target.z},
                        },
                    }
                )
        self._avoidance_active_ids = active_ids

    def _build_bypass_target(self, current: Position, goal: Position, obstacle) -> Position:
        """Create a meaningful side-step waypoint around a blocking obstacle."""
        gx = goal.x - current.x
        gy = goal.y - current.y
        gnorm = math.hypot(gx, gy) or 1.0
        dir_x, dir_y = gx / gnorm, gy / gnorm
        perp_x, perp_y = -dir_y, dir_x

        clearance = float(getattr(obstacle, "radius", 8.0)) + 18.0
        forward = min(90.0, max(35.0, gnorm * 0.25))
        c1 = Position(
            current.x + dir_x * forward + perp_x * clearance,
            current.y + dir_y * forward + perp_y * clearance,
            max(1.0, current.z),
        )
        c2 = Position(
            current.x + dir_x * forward - perp_x * clearance,
            current.y + dir_y * forward - perp_y * clearance,
            max(1.0, current.z),
        )
        d1 = math.hypot(c1.x - obstacle.x, c1.y - obstacle.y)
        d2 = math.hypot(c2.x - obstacle.x, c2.y - obstacle.y)
        return c1 if d1 >= d2 else c2

    def add_dynamic_obstacle(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        motion_type: str = "linear",
        radius: float = 8.0,
        z: float = 0.0,
    ) -> int:
        try:
            mtype = MotionType(str(motion_type).strip().lower())
        except Exception:
            mtype = MotionType.LINEAR
        return self.obstacle_manager.add_obstacle(x, y, vx, vy, motion_type=mtype, radius=radius, z=z)

    def populate_dynamic_obstacle_field(
        self,
        count: int = 24,
        center_x: Optional[float] = None,
        center_y: Optional[float] = None,
        area_radius: float = 2600.0,
    ) -> int:
        """
        Auto-create a field of moving obstacles in the visible operation frame.
        Returns number of obstacles added.
        """
        if count <= 0:
            return 0

        if center_x is None or center_y is None:
            if self.drones:
                homes = [d.home_position for d in self.drones.values()]
                center_x = sum(p.x for p in homes) / len(homes)
                center_y = sum(p.y for p in homes) / len(homes)
            else:
                center_x = 0.0
                center_y = 0.0

        motion_pool = ["linear", "circular", "random_walk"]
        added = 0
        for _ in range(int(count)):
            angle = random.uniform(0.0, 2.0 * math.pi)
            r = random.uniform(180.0, max(200.0, float(area_radius)))
            x = float(center_x) + r * math.cos(angle)
            y = float(center_y) + r * math.sin(angle)

            speed = random.uniform(2.0, 11.0)
            heading = random.uniform(0.0, 2.0 * math.pi)
            vx = speed * math.cos(heading)
            vy = speed * math.sin(heading)
            motion_type = random.choice(motion_pool)
            radius = random.uniform(7.0, 18.0)

            self.add_dynamic_obstacle(
                x=x,
                y=y,
                vx=vx,
                vy=vy,
                motion_type=motion_type,
                radius=radius,
            )
            added += 1

        self.logger.info(
            "Auto obstacle field initialized with %s dynamic obstacles (center=%.1f, %.1f, radius=%.1f)",
            added,
            float(center_x),
            float(center_y),
            float(area_radius),
        )
        return added

    def add_static_obstacle(self, x: float, y: float, radius: float = 8.0, z: float = 0.0) -> int:
        return self.obstacle_manager.add_obstacle(x, y, 0.0, 0.0, motion_type=MotionType.LINEAR, radius=radius, z=z)

    def clear_obstacles(self):
        self.obstacle_manager.clear()

    def set_use_personal_ml_avoidance(self, enabled: bool):
        self.use_personal_ml_avoidance = bool(enabled)

    def set_personal_ml_enabled(self, drone_id: int, enabled: bool) -> bool:
        drone = self.drones.get(drone_id)
        if drone is None:
            return False
        drone.personal_ml_enabled = bool(enabled)
        return True

    def set_personal_ml_enabled_all(self, enabled: bool):
        for drone in self.drones.values():
            drone.personal_ml_enabled = bool(enabled)

    def simulate_latency_spike(self, total_ms: float = 450.0) -> dict:
        return self.ml_bridge.inject_spike(total_ms=total_ms)
    
    def _check_heartbeats(self):
        """Check heartbeat status of all drones"""
        current_time = time.time()
        
        for drone_id, drone in list(self.drones.items()):
            last_heartbeat = drone.last_heartbeat
            
            if current_time - last_heartbeat > self.HEARTBEAT_TIMEOUT:
                self.logger.error(f"Drone {drone_id} heartbeat timeout")
                self._handle_drone_failure(drone_id, "Heartbeat timeout")
    
    def _monitor_leader(self):
        """Monitor leader drone status"""
        if self.leader_id not in self.drones:
            self.logger.error(f"Leader drone {self.leader_id} not in swarm")
            self.leader_id = None
            self.elect_leader()
            return
        
        leader = self.drones[self.leader_id]
        
        # Check if leader is still operational
        if not leader.is_active or leader.flight_mode in [
            FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING
        ] or leader.role in [DroneRole.EMERGENCY, DroneRole.GROUNDED] or leader.emergency_status.active:
            self.logger.warning(f"Leader drone {self.leader_id} no longer operational")
            self._handle_drone_failure(self.leader_id, "Leader failure")
    
    def _check_drone_status(self):
        """Check status of all drones"""
        for drone_id, drone in list(self.drones.items()):
            # Check if drone crashed
            if drone.flight_mode == FlightMode.CRASHED:
                failure_key = (drone_id, "crashed")
                if failure_key not in self.reported_failures:
                    self.reported_failures.add(failure_key)
                    self.logger.error(f"Drone {drone_id} crashed")
                    self._handle_drone_failure(drone_id, "Crashed")
                continue
            
            # Check motor failures
            failed_motors = [m for m in drone.motors if not m.operational]
            if len(failed_motors) >= 2:  # Multiple motor failure
                self.logger.error(f"Drone {drone_id} has {len(failed_motors)} failed motors")
            
            # Emergency drones are not valid leaders
            if drone_id == self.leader_id and (
                drone.flight_mode == FlightMode.EMERGENCY_LANDING
                or drone.role in [DroneRole.EMERGENCY, DroneRole.GROUNDED]
                or drone.emergency_status.active
            ):
                self.logger.warning(f"Leader drone {drone_id} entered emergency state")
                self._handle_drone_failure(drone_id, "Leader entered emergency state")
    
    def _handle_drone_failure(self, drone_id: int, reason: str):
        """Handle drone failure"""
        self.logger.error(f"Handling failure of drone {drone_id}: {reason}")
        
        # If failed drone was leader, elect new one
        if drone_id == self.leader_id:
            self.logger.warning(f"Leader drone {drone_id} failed - initiating leader election")
            self.leader_id = None
            
            # Give remaining drones time to stabilize
            time.sleep(0.2)
            
            if len(self.drones) > 1:  # Still have drones
                self.elect_leader()
    
    def elect_leader(self):
        """
        Elect new leader based on suitability score
        Score = 0.4*battery + 0.3*signal + 0.2*processing + 0.1*motors
        """
        if self.election_in_progress:
            return
        
        if len(self.drones) == 0:
            self.logger.warning("Cannot elect leader - no drones in swarm")
            return
        
        self.election_in_progress = True
        self.logger.info("Starting leader election...")
        
        # Calculate suitability scores for all operational drones
        candidates = {}
        for drone_id, drone in self.drones.items():
            if drone.is_active and drone.flight_mode not in [
                FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING
            ] and drone.role not in [DroneRole.EMERGENCY, DroneRole.GROUNDED] and not drone.emergency_status.active:
                score = drone.get_suitability_score()
                candidates[drone_id] = score
                self.logger.info(f"  Drone {drone_id}: suitability score = {score:.2f}")
        
        if not candidates:
            self.logger.error("No suitable candidates for leader")
            self.election_in_progress = False
            return
        
        # Select drone with highest score
        new_leader_id = max(candidates.items(), key=lambda x: x[1])[0]
        
        # Update roles
        for drone_id, drone in self.drones.items():
            if drone_id == new_leader_id:
                drone.set_role(DroneRole.LEADER)
            else:
                drone.set_role(DroneRole.FOLLOWER)
        
        self.leader_id = new_leader_id
        self.logger.info(f"Drone {new_leader_id} elected as new LEADER (score: {candidates[new_leader_id]:.2f})")
        
        self.election_in_progress = False
    
    def get_leader(self) -> Optional[Drone]:
        """Get current leader drone"""
        if self.leader_id in self.drones:
            return self.drones[self.leader_id]
        return None
    
    def get_followers(self) -> List[Drone]:
        """Get all follower drones"""
        return [
            drone for drone in self.drones.values()
            if drone.role == DroneRole.FOLLOWER and drone.is_active
        ]
    
    def get_active_drones(self) -> List[Drone]:
        """Get all active drones"""
        return [drone for drone in self.drones.values() if drone.is_active]
    
    def get_swarm_status(self) -> dict:
        """Get complete swarm status"""
        drones_status = {}
        for drone_id, drone in self.drones.items():
            status = drone.get_status()
            status["swarm_state"] = self.drone_state_manager.get_state(drone_id).value
            drones_status[drone_id] = status
        
        return {
            "total_drones": len(self.drones),
            "active_drones": len(self.get_active_drones()),
            "leader_id": self.leader_id,
            "election_in_progress": self.election_in_progress,
            "drones": drones_status,
            "dynamic_obstacles": self.obstacle_manager.get_obstacles_as_dict(),
            "latency": self.latency_monitor.get_stats(),
            "per_drone_latency_threshold_ms": dict(self.per_drone_latency_threshold_ms),
            "fallback_local_avoidance_mode": self.fallback_local_avoidance_mode,
            "use_personal_ml_avoidance": self.use_personal_ml_avoidance,
        }
    
    def formation_flight(self, formation_type: str = "line"):
        """
        Command swarm to fly in formation
        
        Args:
            formation_type: "line", "v", "circle", "grid"
        """
        leader = self.get_leader()
        if not leader:
            self.logger.error("Cannot form formation - no leader")
            return
        
        followers = self.get_followers()
        if not followers:
            self.logger.info("No followers to form formation")
            return
        
        leader_pos = leader.current_position
        formation = (formation_type or "line").strip().lower()
        spacing = 12.0
        commanded = 0
        skipped = 0

        if formation not in {"line", "v", "circle", "grid"}:
            self.logger.warning(f"Unknown formation '{formation_type}', falling back to 'line'")
            formation = "line"

        # Order followers around center: +1, -1, +2, -2, ...
        line_slots = []
        if formation == "line":
            for rank in range(1, len(followers) + 1):
                line_slots.append(rank)
                line_slots.append(-rank)
            line_slots = line_slots[:len(followers)]

        # Build centered square offsets and skip leader center (0, 0).
        grid_slots = []
        if formation == "grid":
            side = int(math.ceil(math.sqrt(len(followers) + 1)))
            if side % 2 == 0:
                side += 1
            half = side // 2
            for row in range(-half, half + 1):
                for col in range(-half, half + 1):
                    if row == 0 and col == 0:
                        continue
                    grid_slots.append((row, col))
            grid_slots.sort(key=lambda rc: (max(abs(rc[0]), abs(rc[1])), abs(rc[0]) + abs(rc[1]), rc[0], rc[1]))
            grid_slots = grid_slots[:len(followers)]

        for i, follower in enumerate(followers):
            # Ensure follower can accept goto from idle/transitional modes.
            if not self._prepare_drone_for_goto(follower, preferred_alt=leader_pos.z):
                skipped += 1
                continue

            if formation == "line":
                # Leader at center, followers split left-right on one line.
                lateral_index = line_slots[i]
                target = Position(
                    leader_pos.x,
                    leader_pos.y + spacing * lateral_index,
                    leader_pos.z
                )
            elif formation == "v":
                # V shape with leader at the front.
                side = 1 if i % 2 == 0 else -1
                rank = (i // 2) + 1
                back = spacing * rank
                lateral = spacing * rank * side
                target = Position(
                    leader_pos.x - back,
                    leader_pos.y + lateral,
                    leader_pos.z
                )
            elif formation == "circle":
                # Evenly distribute followers around leader.
                radius = max(15.0, spacing + len(followers) * 1.5)
                angle = (2.0 * math.pi * i) / len(followers)
                target = Position(
                    leader_pos.x + radius * math.cos(angle),
                    leader_pos.y + radius * math.sin(angle),
                    leader_pos.z
                )
            else:  # grid
                # Centered square around leader.
                row, col = grid_slots[i]
                target = Position(
                    leader_pos.x + spacing * row,
                    leader_pos.y + spacing * col,
                    leader_pos.z
                )

            if follower.goto(target):
                commanded += 1
            else:
                skipped += 1

        self.logger.info(
            f"Formation '{formation}' commanded: success={commanded}, skipped={skipped}, followers={len(followers)}"
        )
    
    def emergency_land_all(self, reason: str = "Emergency commanded"):
        """Emergency land all drones"""
        self.logger.error(f"EMERGENCY LAND ALL DRONES: {reason}")
        for drone in self.drones.values():
            drone.trigger_personal_emergency(reason)

    def emergency_land_drone(self, drone_id: int, reason: str = "Personal emergency commanded") -> bool:
        """Emergency land a specific drone only"""
        drone = self.drones.get(drone_id)
        if not drone:
            self.logger.warning(f"Drone {drone_id} not found for personal emergency landing")
            return False
        drone.trigger_personal_emergency(reason)
        self.logger.error(f"Personal emergency landing triggered for Drone {drone_id}: {reason}")
        return True
    
    def return_all_to_home(self):
        """Command all drones to return home"""
        self.logger.info("Leader-commanded RETURN_TO_HOME requested")
        self.leader_command_handler.issue_return_to_home()

    def leader_takeoff(self):
        """Public API: explicit leader command for coordinated takeoff."""
        self.leader_command_handler.issue_takeoff()

    def leader_move_to_target(
        self,
        targets: Dict[int, Position],
        gps_mode_map: Optional[Dict[int, bool]] = None,
    ):
        """Public API: explicit leader command for movement."""
        self.leader_command_handler.issue_move_to_target(targets, gps_mode_map=gps_mode_map)

    def leader_move_all_to_single_target(
        self,
        target: Position,
        gps_mode_map: Optional[Dict[int, bool]] = None,
    ):
        """Helper: move all active drones to the same target Y."""
        targets = {drone_id: Position(target.x, target.y, target.z) for drone_id in self.drones.keys()}
        self.leader_move_to_target(targets, gps_mode_map=gps_mode_map)

    def assign_area_mission_to_drone(
        self,
        drone_id: int,
        ref_lat: float,
        ref_lon: float,
        target_lat: float,
        target_lon: float,
        radius_m: float
    ) -> bool:
        """Assign targeted GPS area mission to one drone."""
        drone = self.drones.get(drone_id)
        if not drone:
            self.logger.warning(f"Drone {drone_id} not found for area mission")
            return False
        drone.set_gps_reference(ref_lat, ref_lon)
        drone.assign_area_mission(target_lat, target_lon, radius_m)
        self.drone_state_manager.set_state(drone_id, DroneOperationalState.GPS_ML_ACTIVE)
        self.logger.info(
            f"Area mission assigned to drone={drone_id} target=({target_lat},{target_lon}) radius={radius_m}m"
        )
        return True

    def clear_area_mission_for_drone(self, drone_id: int) -> bool:
        """Clear targeted mission for one drone."""
        drone = self.drones.get(drone_id)
        if not drone:
            return False
        drone.clear_area_mission()
        if drone.flight_mode in [FlightMode.HOVER, FlightMode.FLYING]:
            self.drone_state_manager.set_state(drone_id, DroneOperationalState.WAITING_FOR_COMMAND)
        else:
            self.drone_state_manager.set_state(drone_id, DroneOperationalState.IDLE)
        return True
