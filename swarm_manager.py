"""
Swarm Manager - Manages drone fleet with leader election and fault tolerance
"""

import time
import threading
import logging
import math
from typing import List, Optional, Dict
from drone import Drone, DroneRole, FlightMode, Position
from collections import defaultdict

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
        self.leader_follow_enabled = True
        
        # Communication
        self.heartbeats: Dict[int, float] = {}
        self.reported_failures = set()
        
        # Thread management
        self.monitor_thread = None
        
        # Logging
        self.logger = logging.getLogger("SwarmManager")
        self.setup_logging()
        
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
    
    def add_drone(self, drone: Drone) -> bool:
        """Add drone to the swarm"""
        if drone.drone_id in self.drones:
            self.logger.warning(f"Drone {drone.drone_id} already in swarm")
            return False
        
        self.drones[drone.drone_id] = drone
        self.heartbeats[drone.drone_id] = time.time()
        self.reported_failures = {
            item for item in self.reported_failures if item[0] != drone.drone_id
        }
        
        # Start drone systems
        drone.start()
        
        self.logger.info(f"Drone {drone.drone_id} added to swarm")
        
        # If no leader, elect one
        if self.leader_id is None and len(self.drones) > 0:
            self.elect_leader()
        
        return True
    
    def remove_drone(self, drone_id: int) -> bool:
        """Remove drone from swarm"""
        if drone_id not in self.drones:
            return False
        
        drone = self.drones[drone_id]
        drone.stop()
        
        del self.drones[drone_id]
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
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            self.logger.info("Swarm monitoring started")
    
    def stop(self):
        """Stop swarm operations"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        # Stop all drones
        for drone in self.drones.values():
            drone.stop()
        
        self.logger.info("Swarm monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            # Check heartbeats
            self._check_heartbeats()
            
            # Monitor leader
            if self.leader_id is not None:
                self._monitor_leader()
            
            # Check for failed drones
            self._check_drone_status()
            
            # Personal ML based leader-follow behavior
            self._ml_follow_leader()
            
            # Personal ML based close-range separation
            self._apply_personal_ml_separation()
            
            time.sleep(1.0)

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
            time.sleep(1.0)
            
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
        self.logger.info(f"✓ Drone {new_leader_id} elected as new LEADER (score: {candidates[new_leader_id]:.2f})")
        
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
        drones_status = {
            drone_id: drone.get_status()
            for drone_id, drone in self.drones.items()
        }
        
        return {
            "total_drones": len(self.drones),
            "active_drones": len(self.get_active_drones()),
            "leader_id": self.leader_id,
            "election_in_progress": self.election_in_progress,
            "drones": drones_status
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
        self.logger.info("Commanding all drones to return home")
        for drone in self.drones.values():
            drone.return_to_home("Swarm RTH command")

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
        return True
