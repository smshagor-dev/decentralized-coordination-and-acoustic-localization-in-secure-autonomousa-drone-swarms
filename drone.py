"""
Drone Class - Core drone functionality with dynamic battery management
Supports real drone integration via MAVLink/MAVSDK
"""

import time
import math
import threading
import logging
import os
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
import json

class DroneRole(Enum):
    """Drone role in the swarm"""
    FOLLOWER = "follower"
    LEADER = "leader"
    EMERGENCY = "emergency"
    GROUNDED = "grounded"

class FlightMode(Enum):
    """Flight operation modes"""
    IDLE = "idle"
    TAKEOFF = "takeoff"
    HOVER = "hover"
    FLYING = "flying"
    RETURNING_HOME = "returning_home"
    LANDING = "landing"
    EMERGENCY_LANDING = "emergency_landing"
    CRASHED = "crashed"

@dataclass
class Position:
    """3D position with latitude, longitude, altitude"""
    x: float = 0.0  # or latitude
    y: float = 0.0  # or longitude
    z: float = 0.0  # altitude in meters
    
    def distance_to(self, other: 'Position') -> float:
        """Calculate 3D distance to another position"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z}

@dataclass
class MotorStatus:
    """Status of each motor"""
    motor_id: int
    operational: bool
    rpm: float
    temperature: float

@dataclass
class EmergencyLandingStatus:
    """Per-drone emergency landing state and audit info"""
    active: bool = False
    reason: str = ""
    trigger_source: str = ""
    triggered_at: float = 0.0
    landing_position: Optional[Position] = None
    completed_at: float = 0.0
    completion_note: str = ""

@dataclass
class AreaMission:
    """Per-drone targeted area mission (GPS + local frame)."""
    active: bool = False
    center_local: Optional[Position] = None
    center_lat: float = 0.0
    center_lon: float = 0.0
    radius_m: float = 0.0
    assigned_at: float = 0.0
    status: str = "idle"
    last_update: float = 0.0

class Drone:
    """
    Main Drone class with full autonomous capabilities
    Designed for real drone integration
    """
    
    # Battery consumption rates (% per second)
    BATTERY_IDLE = 0.001        # 0.1% per 100 seconds
    BATTERY_HOVER = 0.01        # 1% per 100 seconds
    BATTERY_FLYING = 0.02       # 2% per 100 seconds
    BATTERY_EMERGENCY = 0.005   # 0.5% per 100 seconds
    
    # Battery thresholds
    CRITICAL_BATTERY = 20.0     # Emergency landing
    LOW_BATTERY = 30.0          # Return to home
    
    # Flight parameters
    MAX_SPEED = 15.0            # m/s
    MAX_ALTITUDE = 10000.0      # 10 km
    TAKEOFF_ALTITUDE = 10000.0  # 10 km
    LANDING_SPEED = 1.0         # m/s
    MAX_OPERATION_RADIUS = 10000.0  # 10 km
    
    def __init__(self, drone_id: int, home_position: Position, 
                 real_drone_connection: Optional[str] = None):
        """
        Initialize drone with ID and home position
        
        Args:
            drone_id: Unique identifier for this drone
            home_position: Starting position (return point)
            real_drone_connection: MAVLink connection string (e.g., "udp://:14540")
        """
        self.drone_id = drone_id
        self.home_position = Position(home_position.x, home_position.y, home_position.z)
        self.current_position = Position(home_position.x, home_position.y, 0.0)
        
        # Drone state
        self.battery_level = 100.0
        self.role = DroneRole.FOLLOWER
        self.flight_mode = FlightMode.IDLE
        self.is_active = True
        self.is_armed = False
        
        # Motor status (quadcopter: 4 motors)
        self.motors = [
            MotorStatus(i, True, 0.0, 25.0) for i in range(4)
        ]
        
        # Communication
        self.signal_strength = 100.0
        self.last_heartbeat = time.time()
        self.leader_id = None
        
        # Flight data
        self.velocity = Position(0, 0, 0)
        self.target_position = None
        self.waypoints: List[Position] = []
        
        # Real drone connection
        self.real_drone_connection = real_drone_connection
        self.mavlink_connected = False
        
        # Performance metrics
        self.processing_capability = 100.0
        self.total_flight_time = 0.0
        self.mission_count = 0

        # Logging
        self.logger = logging.getLogger(f"Drone_{drone_id}")
        self.setup_logging()

        # Per-drone ML system
        self.ml_system = None
        self.ml_trainer = None
        self._last_auto_train_samples = 0
        self._initialize_ml_system()

        # Demo/visual realism: keep moving in hover after takeoff.
        self.auto_motion_enabled = True
        self.auto_motion_radius = 220.0   # meters
        self.auto_motion_speed = 8.0      # m/s
        self._auto_motion_phase = float(self.drone_id) * 0.7
        self._auto_motion_time = 0.0

        # Personal emergency landing system (per drone)
        self.emergency_status = EmergencyLandingStatus()
        
        # GPS reference + per-drone mission
        self.gps_ref_lat = 23.8103
        self.gps_ref_lon = 90.4125
        self.area_mission = AreaMission()
        
        # Thread management
        self.update_thread = None
        self.running = False
        
        self.logger.info(f"Drone {drone_id} initialized at {home_position.to_dict()}")
    
    def setup_logging(self):
        """Configure logging for this drone"""
        handler = logging.FileHandler(f'logs/drone_{self.drone_id}.log')
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

    def _initialize_ml_system(self):
        """Attach dedicated ML module to this drone."""
        try:
            from ml_system import MLDecisionSupport, PhysicalMLTrainer
            self.ml_system = MLDecisionSupport(owner_id=self.drone_id)
            self.ml_trainer = PhysicalMLTrainer(owner_id=self.drone_id)
            self.ml_trainer.load_model()
            self._bootstrap_personal_training_dataset()
            self.logger.info(f"Drone {self.drone_id}: Personal ML system initialized")
        except Exception as e:
            self.ml_system = None
            self.ml_trainer = None
            self.logger.warning(f"Drone {self.drone_id}: ML system unavailable - {e}")

    def _bootstrap_personal_training_dataset(self):
        """
        Optional startup bootstrap:
        auto-load and train from datasets/personal_drone_<id>.csv|json if present.
        """
        if not self.ml_trainer:
            return
        candidates = [
            f"datasets/personal_drone_{self.drone_id}.csv",
            f"datasets/personal_drone_{self.drone_id}.json",
            "datasets/personal_training.csv",
            "datasets/personal_training.json",
        ]
        for path in candidates:
            if not os.path.exists(path):
                continue
            loaded = self.ml_trainer.import_dataset(path, append=True)
            if loaded > 0:
                self.ml_trainer.train(min_samples=50, poly_degree=2)
                self.logger.info(
                    f"Drone {self.drone_id}: bootstrap personal trainer loaded {loaded} rows from {path}"
                )
                break

    def _horizontal_distance(self, a: Position, b: Position) -> float:
        """2D horizontal distance between two positions."""
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def set_gps_reference(self, ref_lat: float, ref_lon: float):
        """Set local-world origin in GPS coordinates for this drone."""
        self.gps_ref_lat = ref_lat
        self.gps_ref_lon = ref_lon

    def gps_to_local(self, lat: float, lon: float) -> Position:
        """Convert GPS lat/lon to local meters (x east, y north)."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.gps_ref_lat))
        x = (lon - self.gps_ref_lon) * meters_per_deg_lon
        y = (lat - self.gps_ref_lat) * meters_per_deg_lat
        return Position(x, y, 0.0)

    def local_to_gps(self, position: Position) -> Tuple[float, float]:
        """Convert local meters to GPS lat/lon."""
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.gps_ref_lat))
        lat = self.gps_ref_lat + (position.y / meters_per_deg_lat)
        lon = self.gps_ref_lon + (position.x / meters_per_deg_lon)
        return lat, lon

    def assign_area_mission(self, center_lat: float, center_lon: float, radius_m: float):
        """Assign targeted GPS area mission to this drone."""
        center_local = self.gps_to_local(center_lat, center_lon)
        self.area_mission = AreaMission(
            active=True,
            center_local=center_local,
            center_lat=center_lat,
            center_lon=center_lon,
            radius_m=max(10.0, radius_m),
            assigned_at=time.time(),
            status="assigned",
            last_update=0.0
        )
        self.logger.info(
            f"Drone {self.drone_id}: area mission assigned lat={center_lat} lon={center_lon} radius={radius_m}m"
        )

    def clear_area_mission(self):
        """Clear targeted area mission."""
        self.area_mission = AreaMission()
        self.logger.info(f"Drone {self.drone_id}: area mission cleared")

    def _send_real_drone_command(self, command: str, payload: Optional[dict] = None):
        """
        Placeholder bridge for real hardware commands.

        The simulator runs without a real drone. If `real_drone_connection` is configured,
        this method is where MAVLink/MAVSDK calls should be wired.
        """
        if not self.real_drone_connection:
            return
        self.logger.info(
            f"Drone {self.drone_id}: real-drone command={command} payload={payload}"
        )
        # Real drone integration example (commented intentionally):
        # from mavsdk import System
        # real_drone = System(mavsdk_server_address="127.0.0.1", port=50051)
        # await real_drone.connect(system_address=self.real_drone_connection)
        # if command == "arm":
        #     await real_drone.action.arm()
        # elif command == "takeoff":
        #     await real_drone.action.takeoff()
        # elif command == "land":
        #     await real_drone.action.land()
        # elif command == "goto":
        #     await real_drone.action.goto_location(
        #         payload["lat"], payload["lon"], payload["alt"], payload["yaw"]
        #     )
    
    def start(self):
        """Start drone autonomous systems"""
        if not self.running:
            self.running = True
            self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
            self.update_thread.start()
            self.logger.info(f"Drone {self.drone_id} started")
    
    def stop(self):
        """Stop drone systems"""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
        self.logger.info(f"Drone {self.drone_id} stopped")
    
    def _update_loop(self):
        """Main update loop running in separate thread"""
        last_update = time.time()
        
        while self.running and self.is_active:
            current_time = time.time()
            delta_time = current_time - last_update
            last_update = current_time
            
            # Update battery
            self._update_battery(delta_time)
            
            # Check battery status
            self._check_battery_status()
            
            # Check motor status
            self._check_motor_status()
            
            # Update position based on flight mode
            prev_position = Position(
                self.current_position.x, self.current_position.y, self.current_position.z
            )
            self._update_position(delta_time)
            if delta_time > 0:
                self.velocity = Position(
                    (self.current_position.x - prev_position.x) / delta_time,
                    (self.current_position.y - prev_position.y) / delta_time,
                    (self.current_position.z - prev_position.z) / delta_time
                )

            # Capture physical/sim telemetry samples for training.
            self._collect_physical_training_sample()
            
            # Execute assigned targeted-area mission.
            self._execute_area_mission(current_time)
            
            # Update heartbeat
            self.last_heartbeat = current_time
            
            # Sleep to maintain update rate
            time.sleep(0.1)  # 10 Hz update rate

    def _execute_area_mission(self, now: float):
        """Execute targeted area mission independently for this drone."""
        mission = self.area_mission
        if not mission.active or mission.center_local is None:
            return
        if self.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
            mission.status = "blocked_emergency"
            return

        # Ensure controllable flight state.
        if not self.is_armed:
            self.arm()
        if self.flight_mode in [FlightMode.IDLE, FlightMode.TAKEOFF, FlightMode.LANDING]:
            self.flight_mode = FlightMode.HOVER
            if self.current_position.z < 20.0:
                self.current_position.z = 20.0

        distance = self._horizontal_distance(self.current_position, mission.center_local)
        if distance > mission.radius_m * 0.8:
            # Move toward target area center.
            mission.status = "transit_to_area"
            if self.target_position is None or self.flight_mode != FlightMode.FLYING:
                self.goto(Position(
                    mission.center_local.x,
                    mission.center_local.y,
                    max(20.0, self.current_position.z)
                ))
            return

        # In-area loiter pattern (circle) for active mission execution.
        mission.status = "on_target_loiter"
        if now - mission.last_update < 1.5:
            return
        mission.last_update = now
        loiter_radius = max(10.0, mission.radius_m * 0.6)
        angle = now * 0.25
        tx = mission.center_local.x + loiter_radius * math.cos(angle)
        ty = mission.center_local.y + loiter_radius * math.sin(angle)
        tz = max(20.0, self.current_position.z)
        self.goto(Position(tx, ty, tz))

    def _collect_physical_training_sample(self):
        """Collect telemetry for physical ML training pipeline."""
        if not self.ml_trainer:
            return
        # Feature vector from live drone state.
        features = [
            self.battery_level,
            self.signal_strength,
            self.processing_capability,
            self.current_position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z
        ]
        # Target control vector (behavioral proxy in this simulator).
        targets = [self.velocity.x, self.velocity.y, self.velocity.z]
        self.ml_trainer.ingest_sample(features, targets)

        sample_count = len(self.ml_trainer.samples_x)
        # Auto-train in background cadence once enough physical samples exist.
        if sample_count >= 200 and sample_count - self._last_auto_train_samples >= 200:
            if self.ml_trainer.train(min_samples=200):
                self._last_auto_train_samples = sample_count

    def ingest_physical_training_sample(self, sensor_features: List[float], target_controls: List[float]):
        """Public API: ingest real drone sample from external sensor stream."""
        if not self.ml_trainer:
            return
        self.ml_trainer.ingest_sample(sensor_features, target_controls)

    def train_physical_ml_model(self, min_samples: int = 200) -> bool:
        """Public API: train ML model from accumulated physical samples."""
        if not self.ml_trainer:
            return False
        return self.ml_trainer.train(min_samples=min_samples)

    def train_physical_ml_from_dataset(
        self,
        dataset_path: str,
        min_samples: int = 50,
        poly_degree: int = 2,
        append: bool = False,
    ) -> bool:
        """Public API: load CSV/JSON dataset and train personal model."""
        if not self.ml_trainer:
            return False
        return self.ml_trainer.train_from_dataset(
            dataset_path,
            min_samples=min_samples,
            poly_degree=poly_degree,
            append=append,
        )

    def export_physical_training_dataset(self, output_path: str, file_format: str = "csv") -> bool:
        """Public API: export current personal training samples to CSV/JSON."""
        if not self.ml_trainer:
            return False
        return self.ml_trainer.export_dataset(output_path, file_format=file_format)
    
    def _update_battery(self, delta_time: float):
        """Update battery level based on current flight mode"""
        if not self.is_active or self.flight_mode == FlightMode.CRASHED:
            return
        
        # Calculate battery consumption
        consumption_rate = {
            FlightMode.IDLE: self.BATTERY_IDLE,
            FlightMode.TAKEOFF: self.BATTERY_FLYING * 1.5,
            FlightMode.HOVER: self.BATTERY_HOVER,
            FlightMode.FLYING: self.BATTERY_FLYING,
            FlightMode.RETURNING_HOME: self.BATTERY_FLYING,
            FlightMode.LANDING: self.BATTERY_HOVER,
            FlightMode.EMERGENCY_LANDING: self.BATTERY_EMERGENCY,
        }.get(self.flight_mode, self.BATTERY_IDLE)
        
        # Apply consumption
        self.battery_level = max(0.0, self.battery_level - (consumption_rate * delta_time))
        
        # Battery depleted
        if self.battery_level <= 0:
            self.emergency_land("Battery depleted", trigger_source="battery_monitor")
    
    def _check_battery_status(self):
        """Monitor battery and trigger appropriate actions"""
        if self.flight_mode in [FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING]:
            return
        
        if self.battery_level <= self.CRITICAL_BATTERY:
            self.logger.warning(f"Drone {self.drone_id}: Critical battery {self.battery_level:.1f}%")
            self.emergency_land(
                f"Critical battery: {self.battery_level:.1f}%",
                trigger_source="battery_monitor"
            )
        
        elif self.battery_level <= self.LOW_BATTERY and self.flight_mode not in [
            FlightMode.RETURNING_HOME, FlightMode.LANDING, FlightMode.IDLE
        ]:
            self.logger.warning(f"Drone {self.drone_id}: Low battery {self.battery_level:.1f}%")
            self.return_to_home("Low battery")
    
    def _check_motor_status(self):
        """Check motor status and handle failures"""
        failed_motors = [m for m in self.motors if not m.operational]
        
        if failed_motors and self.flight_mode not in [
            FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED, FlightMode.IDLE
        ]:
            self.logger.error(f"Drone {self.drone_id}: Motor failure detected - Motors {[m.motor_id for m in failed_motors]}")
            self.emergency_land(
                f"Motor failure: {len(failed_motors)} motors failed",
                trigger_source="motor_monitor"
            )
    
    def _update_position(self, delta_time: float):
        """Update drone position based on current flight mode"""
        if self.flight_mode == FlightMode.TAKEOFF:
            self._handle_takeoff(delta_time)
        
        elif self.flight_mode == FlightMode.FLYING:
            self._handle_flying(delta_time)

        elif self.flight_mode == FlightMode.HOVER:
            self._handle_hover_motion(delta_time)
        
        elif self.flight_mode == FlightMode.RETURNING_HOME:
            self._handle_return_to_home(delta_time)
        
        elif self.flight_mode == FlightMode.LANDING or self.flight_mode == FlightMode.EMERGENCY_LANDING:
            self._handle_landing(delta_time)

    def _handle_hover_motion(self, delta_time: float):
        """
        Keep the drone moving smoothly in hover mode (realistic loiter pattern).
        This improves visibility of forward/back/left/right movement in the GUI.
        """
        if not self.auto_motion_enabled:
            return
        if not self.is_armed:
            return
        if self.target_position is not None:
            return
        if self.area_mission.active:
            return
        if self.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
            return

        self._auto_motion_time += max(0.0, delta_time)
        omega = max(0.05, self.auto_motion_speed / max(1.0, self.auto_motion_radius))
        phase = self._auto_motion_phase + self._auto_motion_time * omega

        # Elliptical loiter around home so X/Y motion is easy to understand.
        desired_x = self.home_position.x + self.auto_motion_radius * math.cos(phase)
        desired_y = self.home_position.y + (self.auto_motion_radius * 0.65) * math.sin(phase)
        desired_z = max(1.0, self.current_position.z)

        dx = desired_x - self.current_position.x
        dy = desired_y - self.current_position.y
        dz = desired_z - self.current_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < 0.001:
            return

        step = min(distance, self.auto_motion_speed * delta_time)
        self.current_position.x += (dx / distance) * step
        self.current_position.y += (dy / distance) * step
        self.current_position.z += (dz / distance) * step
    
    def _handle_takeoff(self, delta_time: float):
        """Handle takeoff procedure"""
        if self.current_position.z < self.TAKEOFF_ALTITUDE:
            self.current_position.z += self.LANDING_SPEED * 2 * delta_time
        else:
            self.flight_mode = FlightMode.HOVER
            self.logger.info(f"Drone {self.drone_id}: Takeoff complete at {self.current_position.z:.1f}m")
    
    def _handle_flying(self, delta_time: float):
        """Handle flying to target position"""
        if not self.target_position:
            self.flight_mode = FlightMode.HOVER
            return
        
        # Calculate direction
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        dz = self.target_position.z - self.current_position.z
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < 1.0:  # Reached target
            self.current_position = Position(
                self.target_position.x,
                self.target_position.y,
                self.target_position.z
            )
            self.target_position = None
            self.flight_mode = FlightMode.HOVER
            self.logger.info(f"Drone {self.drone_id}: Reached target position")
        else:
            # Move towards target
            speed = min(self.MAX_SPEED, distance / delta_time)
            self.current_position.x += (dx / distance) * speed * delta_time
            self.current_position.y += (dy / distance) * speed * delta_time
            self.current_position.z += (dz / distance) * speed * delta_time
    
    def _handle_return_to_home(self, delta_time: float):
        """Handle return to home procedure"""
        distance = self.current_position.distance_to(self.home_position)
        
        if distance < 1.0 and abs(self.current_position.z - self.home_position.z) < 1.0:
            self.logger.info(f"Drone {self.drone_id}: Reached home position")
            self.land()
        else:
            # Move towards home
            dx = self.home_position.x - self.current_position.x
            dy = self.home_position.y - self.current_position.y
            
            horizontal_distance = math.sqrt(dx**2 + dy**2)
            
            if horizontal_distance > 1.0:
                speed = min(self.MAX_SPEED, horizontal_distance / delta_time)
                self.current_position.x += (dx / horizontal_distance) * speed * delta_time
                self.current_position.y += (dy / horizontal_distance) * speed * delta_time
            
            # Descend gradually
            if abs(self.current_position.z - self.home_position.z) > 0.5:
                self.current_position.z -= self.LANDING_SPEED * 0.5 * delta_time
    
    def _handle_landing(self, delta_time: float):
        """Handle landing procedure"""
        if self.current_position.z > 0.1:
            descent_speed = self.LANDING_SPEED * 1.5 if self.flight_mode == FlightMode.EMERGENCY_LANDING else self.LANDING_SPEED
            self.current_position.z = max(0, self.current_position.z - descent_speed * delta_time)
        else:
            was_emergency = self.flight_mode == FlightMode.EMERGENCY_LANDING
            self.current_position.z = 0
            self.is_armed = False
            self.flight_mode = FlightMode.IDLE
            self.logger.info(f"Drone {self.drone_id}: Landed safely")
            
            # If emergency landing not at home, mark as needs recovery
            if was_emergency:
                distance_from_home = math.sqrt(
                    (self.current_position.x - self.home_position.x)**2 +
                    (self.current_position.y - self.home_position.y)**2
                )
                self.role = DroneRole.GROUNDED
                self.emergency_status.completed_at = time.time()
                self.emergency_status.active = False
                self.emergency_status.completion_note = "Emergency landing completed safely"
                if distance_from_home > 5.0:
                    self.emergency_status.completion_note = (
                        f"Emergency landing completed {distance_from_home:.1f}m from home"
                    )
                    self.logger.warning(f"Drone {self.drone_id}: Emergency landed {distance_from_home:.1f}m from home")
    
    # Public control methods
    
    def arm(self) -> bool:
        """Arm the drone motors"""
        if self.is_armed:
            return True
        
        if self.battery_level < 10:
            self.logger.error(f"Drone {self.drone_id}: Cannot arm - battery too low")
            return False
        
        if self.role in [DroneRole.EMERGENCY, DroneRole.GROUNDED]:
            self.role = DroneRole.FOLLOWER

        self.is_armed = True
        self._send_real_drone_command("arm")
        self.logger.info(f"Drone {self.drone_id}: Armed")
        return True
    
    def disarm(self):
        """Disarm the drone motors"""
        if self.flight_mode != FlightMode.IDLE:
            self.logger.warning(f"Drone {self.drone_id}: Cannot disarm while flying")
            return False
        
        self.is_armed = False
        self.logger.info(f"Drone {self.drone_id}: Disarmed")
        return True
    
    def takeoff(self) -> bool:
        """Initiate takeoff sequence"""
        if not self.is_armed:
            self.logger.warning(f"Drone {self.drone_id}: Cannot takeoff - not armed")
            return False
        
        if self.flight_mode != FlightMode.IDLE:
            self.logger.warning(f"Drone {self.drone_id}: Cannot takeoff - already flying")
            return False
        
        if self.battery_level < 15:
            self.logger.error(f"Drone {self.drone_id}: Cannot takeoff - battery too low")
            return False
        
        self.flight_mode = FlightMode.TAKEOFF
        self._send_real_drone_command("takeoff")
        self.logger.info(f"Drone {self.drone_id}: Taking off")
        return True
    
    def land(self):
        """Initiate landing sequence"""
        if self.flight_mode == FlightMode.IDLE:
            return
        
        self.target_position = None
        self.flight_mode = FlightMode.LANDING
        self._send_real_drone_command("land")
        self.logger.info(f"Drone {self.drone_id}: Landing initiated")
    
    def emergency_land(self, reason: str, trigger_source: str = "manual"):
        """Initiate emergency landing"""
        if self.flight_mode in [FlightMode.CRASHED, FlightMode.IDLE]:
            return
        
        if self.emergency_status.active:
            return
        
        landing_position = Position(self.current_position.x, self.current_position.y, 0.0)
        self.emergency_status = EmergencyLandingStatus(
            active=True,
            reason=reason,
            trigger_source=trigger_source,
            triggered_at=time.time(),
            landing_position=landing_position,
            completion_note=""
        )
        self.role = DroneRole.EMERGENCY
        self.target_position = None
        self.flight_mode = FlightMode.EMERGENCY_LANDING
        self._send_real_drone_command("emergency_land", {"reason": reason})
        self.logger.error(
            f"Drone {self.drone_id}: EMERGENCY LANDING - {reason} "
            f"(source={trigger_source}, site={landing_position.to_dict()})"
        )

    def trigger_personal_emergency(self, reason: str = "Personal emergency command"):
        """Public per-drone emergency trigger"""
        self.emergency_land(reason, trigger_source="personal_command")
    
    def return_to_home(self, reason: str = "Manual RTH"):
        """Return to home position"""
        if self.flight_mode in [FlightMode.RETURNING_HOME, FlightMode.LANDING, 
                                FlightMode.EMERGENCY_LANDING, FlightMode.IDLE]:
            return
        
        self.target_position = None
        self.flight_mode = FlightMode.RETURNING_HOME
        self._send_real_drone_command("return_to_home", {"reason": reason})
        self.logger.info(f"Drone {self.drone_id}: Returning to home - {reason}")
    
    def goto(self, position: Position) -> bool:
        """Fly to specified position"""
        if self.flight_mode not in [FlightMode.HOVER, FlightMode.FLYING]:
            # Frequent internal callers may attempt goto during transitions.
            # Keep this silent to avoid warning spam in logs.
            return False
        
        if position.z > self.MAX_ALTITUDE:
            self.logger.warning(f"Drone {self.drone_id}: Target altitude too high")
            return False
        
        target_distance_from_home = self._horizontal_distance(position, self.home_position)
        if target_distance_from_home > self.MAX_OPERATION_RADIUS:
            self.logger.warning(
                f"Drone {self.drone_id}: Target beyond 10km operational range "
                f"({target_distance_from_home:.1f}m)"
            )
            return False
        
        self.target_position = position
        self.flight_mode = FlightMode.FLYING
        # For real drones, map local x/y to GPS here before sending `goto`.
        self._send_real_drone_command(
            "goto",
            {
                "x": position.x,
                "y": position.y,
                "z": position.z,
                # "lat": gps_lat,
                # "lon": gps_lon,
                # "alt": position.z,
                # "yaw": 0.0,
            },
        )
        self.logger.info(f"Drone {self.drone_id}: Flying to {position.to_dict()}")
        return True
    
    def set_role(self, role: DroneRole):
        """Set drone role in the swarm"""
        old_role = self.role
        self.role = role
        self.logger.info(f"Drone {self.drone_id}: Role changed from {old_role.value} to {role.value}")
    
    def simulate_motor_failure(self, motor_id: int):
        """Simulate motor failure for testing"""
        if 0 <= motor_id < len(self.motors):
            self.motors[motor_id].operational = False
            self.logger.error(f"Drone {self.drone_id}: Motor {motor_id} failed!")
    
    def get_suitability_score(self) -> float:
        """
        Calculate suitability score for leader election
        Based on battery, signal strength, and processing capability
        """
        score = (
            self.battery_level * 0.4 +
            self.signal_strength * 0.3 +
            self.processing_capability * 0.3
        )
        
        return score
    
    def get_status(self) -> dict:
        """Get complete drone status"""
        gps_lat, gps_lon = self.local_to_gps(self.current_position)
        return {
            "drone_id": self.drone_id,
            "role": self.role.value,
            "flight_mode": self.flight_mode.value,
            "position": self.current_position.to_dict(),
            "position_gps": {"lat": gps_lat, "lon": gps_lon},
            "home_position": self.home_position.to_dict(),
            "battery": round(self.battery_level, 2),
            "signal_strength": round(self.signal_strength, 2),
            "ml_enabled": self.ml_system is not None,
            "physical_ml_enabled": self.ml_trainer is not None,
            "physical_ml_samples": (
                len(self.ml_trainer.samples_x) if self.ml_trainer else 0
            ),
            "physical_ml_trained": (
                self.ml_trainer.weights is not None if self.ml_trainer else False
            ),
            "physical_ml_metrics": (
                self.ml_trainer.training_metrics if self.ml_trainer else {}
            ),
            "auto_motion_enabled": self.auto_motion_enabled,
            "max_operation_radius_m": self.MAX_OPERATION_RADIUS,
            "is_active": self.is_active,
            "is_armed": self.is_armed,
            "velocity": self.velocity.to_dict(),
            "motors": [
                {
                    "id": m.motor_id,
                    "operational": m.operational,
                    "rpm": m.rpm
                } for m in self.motors
            ],
            "emergency": {
                "active": self.emergency_status.active,
                "reason": self.emergency_status.reason,
                "trigger_source": self.emergency_status.trigger_source,
                "triggered_at": self.emergency_status.triggered_at,
                "landing_position": (
                    self.emergency_status.landing_position.to_dict()
                    if self.emergency_status.landing_position else None
                ),
                "completed_at": self.emergency_status.completed_at,
                "completion_note": self.emergency_status.completion_note
            },
            "suitability_score": round(self.get_suitability_score(), 2),
            "total_flight_time": round(self.total_flight_time, 2),
            "gps_reference": {"lat": self.gps_ref_lat, "lon": self.gps_ref_lon},
            "mission": {
                "active": self.area_mission.active,
                "status": self.area_mission.status,
                "center_lat": self.area_mission.center_lat,
                "center_lon": self.area_mission.center_lon,
                "radius_m": self.area_mission.radius_m,
                "assigned_at": self.area_mission.assigned_at
            }
        }
    
    def to_json(self) -> str:
        """Convert drone status to JSON"""
        return json.dumps(self.get_status(), indent=2)
