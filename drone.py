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
"""
Drone Class - Core drone functionality with dynamic battery management
Supports real drone integration via MAVLink/MAVSDK
"""

import time
import math
import asyncio
import queue
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
    TAKEOFF_ALTITUDE = 120.0    # practical visual takeoff altitude for simulation
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
        self.real_drone_connection = self._normalize_connection_string(real_drone_connection)
        self.use_real_drone = bool(real_drone_connection)
        self.mavlink_connected = False
        self._real_system = None
        self._real_command_queue: "queue.Queue[Optional[Tuple[str, dict]]]" = queue.Queue()
        self._real_thread = None
        self._real_backend_started = False
        
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
        self.personal_ml_enabled = True
        self.max_lateral_accel = 4.5
        self.steering_smooth_factor = 0.28

        # Followers must hold takeoff position until explicit leader command.
        self.auto_motion_enabled = False
        self.auto_motion_radius = 220.0   # meters
        self.auto_motion_speed = 8.0      # m/s
        self._auto_motion_phase = float(self.drone_id) * 0.7
        self._auto_motion_time = 0.0

        # Personal emergency landing system (per drone)
        self.emergency_status = EmergencyLandingStatus()
        self.motor_failure_warning = False
        self.failed_motor_count = 0
        self.degraded_return_active = False
        self.emergency_return_active = False
        self._wind_phase = float(self.drone_id) * 1.37
        self.wind_enabled = True
        self.wind_speed_mps = 1.2
        self.wind_direction_deg = (self._wind_phase * 57.2957795) % 360.0
        self.wind_gust_factor = 0.35
        self.wind_compensation_hover = 0.84
        self.wind_compensation_flying = 0.62
        self.wind_compensation_rth = 0.90
        self.wind_vector = Position(0.0, 0.0, 0.0)
        self.current_wind_vector = Position(0.0, 0.0, 0.0)
        self._rebuild_wind_vector()
        
        # GPS reference + per-drone mission
        self.gps_ref_lat = 23.8103
        self.gps_ref_lon = 90.4125
        self.area_mission = AreaMission()
        
        # Thread management
        self.update_thread = None
        self.running = False
        
        self.logger.info(f"Drone {drone_id} initialized at {home_position.to_dict()}")

    def _normalize_connection_string(self, connection: Optional[str]) -> Optional[str]:
        """Normalize legacy MAVSDK connection URL forms."""
        if not connection:
            return connection
        text = str(connection).strip()
        if text.startswith("udp://"):
            # MAVSDK now prefers udpin:// for listen sockets.
            return "udpin://" + text[len("udp://"):]
        return text
    
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

    def _rebuild_wind_vector(self):
        """Recompute base wind vector from configured speed + direction."""
        speed = max(0.0, float(self.wind_speed_mps))
        direction_deg = float(self.wind_direction_deg) % 360.0
        direction_rad = math.radians(direction_deg)
        self.wind_vector = Position(
            speed * math.cos(direction_rad),
            speed * math.sin(direction_rad),
            0.0
        )
        self.current_wind_vector = Position(self.wind_vector.x, self.wind_vector.y, 0.0)

    def set_wind_conditions(
        self,
        speed_mps: float,
        direction_deg: float,
        gust_factor: float = 0.35,
        enabled: bool = True,
    ):
        """Configure local wind effect for simulator physics."""
        self.wind_enabled = bool(enabled)
        self.wind_speed_mps = max(0.0, min(30.0, float(speed_mps)))
        self.wind_direction_deg = float(direction_deg) % 360.0
        self.wind_gust_factor = max(0.0, min(0.95, float(gust_factor)))
        self._rebuild_wind_vector()
        self.logger.info(
            "Drone %s wind updated: enabled=%s speed=%.2f dir=%.1f gust=%.2f",
            self.drone_id,
            self.wind_enabled,
            self.wind_speed_mps,
            self.wind_direction_deg,
            self.wind_gust_factor,
        )

    def _sample_wind(self, delta_time: float) -> Position:
        """Generate smooth wind with gusting and slight directional wobble."""
        if not self.wind_enabled or self.wind_speed_mps <= 0.01:
            self.current_wind_vector = Position(0.0, 0.0, 0.0)
            return self.current_wind_vector

        dt = max(0.0, float(delta_time))
        self._wind_phase += dt * 0.65
        gust_scale = 1.0 + self.wind_gust_factor * math.sin(self._wind_phase * 1.3 + self.drone_id * 0.11)
        cross_scale = 0.28 * self.wind_gust_factor * math.cos(self._wind_phase * 0.7 + self.drone_id * 0.19)

        base_x = self.wind_vector.x
        base_y = self.wind_vector.y
        wind_x = base_x * gust_scale - base_y * cross_scale
        wind_y = base_y * gust_scale + base_x * cross_scale
        self.current_wind_vector = Position(wind_x, wind_y, 0.0)
        return self.current_wind_vector

    def _apply_wind_drift(self, delta_time: float, compensation: float):
        """Apply wind drift to X/Y with controller compensation [0..1]."""
        if self.use_real_drone and self.mavlink_connected:
            return
        wind = self._sample_wind(delta_time)
        compensation = max(0.0, min(1.0, float(compensation)))
        drift_scale = 1.0 - compensation
        if drift_scale <= 0.0:
            return
        dt = max(0.0, float(delta_time))
        self.current_position.x += wind.x * drift_scale * dt
        self.current_position.y += wind.y * drift_scale * dt

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
        """Queue command for MAVSDK backend when real-drone mode is enabled."""
        if not self.use_real_drone:
            return
        payload = payload or {}
        self.logger.info(f"Drone {self.drone_id}: real-drone command={command} payload={payload}")
        if not self._real_backend_started:
            self._start_real_backend()
        self._real_command_queue.put((command, payload))

    def _start_real_backend(self):
        """Start async MAVSDK backend in a dedicated thread."""
        if not self.use_real_drone or self._real_backend_started:
            return
        self._real_backend_started = True
        self._real_thread = threading.Thread(
            target=self._real_backend_thread,
            name=f"Drone{self.drone_id}-MAVSDK",
            daemon=True
        )
        self._real_thread.start()

    def _stop_real_backend(self):
        """Stop MAVSDK backend thread gracefully."""
        if not self._real_backend_started:
            return
        self._real_command_queue.put(None)
        if self._real_thread:
            self._real_thread.join(timeout=3.0)
        self._real_backend_started = False
        self.mavlink_connected = False

    def _real_backend_thread(self):
        """Thread entrypoint for MAVSDK event loop."""
        try:
            asyncio.run(self._real_backend_main())
        except Exception as e:
            self.logger.error(f"Drone {self.drone_id}: MAVSDK backend error - {e}", exc_info=True)
            self.mavlink_connected = False

    async def _real_backend_main(self):
        """Connect to MAVSDK and process command queue + telemetry."""
        try:
            from mavsdk import System
        except Exception as e:
            self.logger.error(
                f"Drone {self.drone_id}: MAVSDK import failed. Install 'mavsdk'. Error: {e}"
            )
            return

        self._real_system = System()
        self.logger.info(
            f"Drone {self.drone_id}: Connecting to real drone ({self.real_drone_connection})"
        )
        await self._real_system.connect(system_address=self.real_drone_connection)

        connected = await self._wait_for_real_connection(timeout_s=20.0)
        if not connected:
            self.logger.error(f"Drone {self.drone_id}: MAVSDK connection timeout")
            return

        self.mavlink_connected = True
        self.logger.info(f"Drone {self.drone_id}: MAVSDK connected")

        telemetry_tasks = [
            asyncio.create_task(self._real_telemetry_position_loop()),
            asyncio.create_task(self._real_telemetry_battery_loop()),
            asyncio.create_task(self._real_telemetry_armed_loop()),
            asyncio.create_task(self._real_telemetry_flight_mode_loop()),
        ]

        try:
            while self.running and self.use_real_drone:
                command_item = await asyncio.to_thread(self._real_command_queue.get)
                if command_item is None:
                    break
                command, payload = command_item
                await self._execute_real_command(command, payload)
        finally:
            for task in telemetry_tasks:
                task.cancel()
            await asyncio.gather(*telemetry_tasks, return_exceptions=True)
            self.mavlink_connected = False

    async def _wait_for_real_connection(self, timeout_s: float) -> bool:
        """Wait for MAVSDK connection state."""
        deadline = time.time() + timeout_s
        async for state in self._real_system.core.connection_state():
            if state.is_connected:
                return True
            if time.time() > deadline:
                return False
        return False

    async def _execute_real_command(self, command: str, payload: dict):
        """Translate simulator command names to MAVSDK actions."""
        if not self._real_system:
            return
        try:
            if command == "arm":
                await self._real_system.action.arm()
            elif command == "takeoff":
                altitude = float(payload.get("altitude_m", self.TAKEOFF_ALTITUDE))
                await self._real_system.action.set_takeoff_altitude(altitude)
                await self._real_system.action.takeoff()
            elif command == "land":
                await self._real_system.action.land()
            elif command == "return_to_home" or command == "emergency_return_home":
                await self._real_system.action.return_to_launch()
            elif command == "hover":
                await self._real_system.action.hold()
            elif command == "goto":
                x = float(payload.get("x", 0.0))
                y = float(payload.get("y", 0.0))
                z = float(payload.get("z", 0.0))
                gps_lat, gps_lon = self.local_to_gps(Position(x, y, z))
                rel_alt = max(1.0, z)
                yaw_deg = float(payload.get("yaw_deg", 0.0))
                await self._real_system.action.goto_location(gps_lat, gps_lon, rel_alt, yaw_deg)
            else:
                self.logger.warning(f"Drone {self.drone_id}: Unknown real command '{command}'")
        except Exception as e:
            self.logger.error(
                f"Drone {self.drone_id}: Real command failed ({command}) - {e}",
                exc_info=True
            )

    async def _real_telemetry_position_loop(self):
        """Continuously sync real GPS/alt telemetry into local frame."""
        try:
            async for pos in self._real_system.telemetry.position():
                local = self.gps_to_local(pos.latitude_deg, pos.longitude_deg)
                self.current_position.x = local.x
                self.current_position.y = local.y
                self.current_position.z = max(0.0, pos.relative_altitude_m)
                self.last_heartbeat = time.time()
        except Exception:
            return

    async def _real_telemetry_battery_loop(self):
        """Continuously sync real battery telemetry."""
        try:
            async for bat in self._real_system.telemetry.battery():
                self.battery_level = max(0.0, min(100.0, float(bat.remaining_percent) * 100.0))
        except Exception:
            return

    async def _real_telemetry_armed_loop(self):
        """Continuously sync real armed/disarmed state."""
        try:
            async for armed in self._real_system.telemetry.armed():
                self.is_armed = bool(armed)
        except Exception:
            return

    async def _real_telemetry_flight_mode_loop(self):
        """Continuously sync real flight mode to simulator enum."""
        mapping = {
            "takeoff": FlightMode.TAKEOFF,
            "hold": FlightMode.HOVER,
            "mission": FlightMode.FLYING,
            "return_to_launch": FlightMode.RETURNING_HOME,
            "rtl": FlightMode.RETURNING_HOME,
            "land": FlightMode.LANDING,
            "offboard": FlightMode.FLYING,
            "position": FlightMode.FLYING,
            "manual": FlightMode.HOVER,
            "altctl": FlightMode.HOVER,
            "posctl": FlightMode.HOVER,
        }
        try:
            async for mode in self._real_system.telemetry.flight_mode():
                mode_key = str(mode).split(".")[-1].strip().lower()
                self.flight_mode = mapping.get(mode_key, self.flight_mode)
        except Exception:
            return
    
    def start(self):
        """Start drone autonomous systems"""
        if not self.running:
            self.running = True
            if self.use_real_drone:
                self._start_real_backend()
            self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
            self.update_thread.start()
            self.logger.info(f"Drone {self.drone_id} started")
    
    def stop(self):
        """Stop drone systems"""
        self.running = False
        self._stop_real_backend()
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
            
            real_connected = self.use_real_drone and self.mavlink_connected
            if not real_connected:
                # In simulator mode, battery and position are physics-driven here.
                self._update_battery(delta_time)
                self._check_battery_status()
            
            # Check motor status
            self._check_motor_status()
            
            # Update position based on active mode.
            # Real mode receives position from telemetry loops.
            prev_position = Position(self.current_position.x, self.current_position.y, self.current_position.z)
            if not real_connected:
                self._update_position(delta_time)
            if delta_time > 0:
                self.velocity = Position(
                    (self.current_position.x - prev_position.x) / max(delta_time, 0.001),
                    (self.current_position.y - prev_position.y) / max(delta_time, 0.001),
                    (self.current_position.z - prev_position.z) / max(delta_time, 0.001)
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
        failed_count = len(failed_motors)
        self.failed_motor_count = failed_count
        self.motor_failure_warning = failed_count > 0

        if failed_count == 0:
            self.degraded_return_active = False
            return

        if self.flight_mode in [FlightMode.CRASHED, FlightMode.IDLE]:
            return

        # One failed motor: keep 3-motor degraded control and return home.
        if failed_count == 1:
            if self.degraded_return_active:
                return
            self.logger.error(
                f"Drone {self.drone_id}: Motor failure detected - Motors {[m.motor_id for m in failed_motors]}"
            )
            self._activate_degraded_return(
                "Motor failure (3 motors active): returning to home",
                trigger_source="motor_monitor"
            )
            return

        # Two or more failed motors: emergency return to home and immediate landing at home.
        if self.emergency_return_active:
            return
        self.logger.error(
            f"Drone {self.drone_id}: Motor failure detected - Motors {[m.motor_id for m in failed_motors]}"
        )
        self.emergency_land(
            f"Severe motor failure: {failed_count} motors failed",
            trigger_source="motor_monitor"
        )

    def _activate_degraded_return(self, reason: str, trigger_source: str = "motor_monitor"):
        """Enter degraded return mode: wind affected, slower, but still tries to reach home."""
        if self.flight_mode in [FlightMode.CRASHED, FlightMode.IDLE]:
            return
        self.degraded_return_active = True
        self.role = DroneRole.EMERGENCY
        self.target_position = None
        if not self.emergency_status.active:
            self.emergency_status = EmergencyLandingStatus(
                active=True,
                reason=reason,
                trigger_source=trigger_source,
                triggered_at=time.time(),
                landing_position=Position(self.current_position.x, self.current_position.y, 0.0),
                completion_note=""
            )
        if self.flight_mode != FlightMode.RETURNING_HOME:
            self.flight_mode = FlightMode.RETURNING_HOME
            self._send_real_drone_command("return_to_home", {"reason": reason, "degraded": True})
        self.logger.warning(f"Drone {self.drone_id}: Degraded return active - {reason}")
    
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
        if self.flight_mode in [FlightMode.EMERGENCY_LANDING, FlightMode.CRASHED]:
            return

        if (
            self.auto_motion_enabled
            and self.is_armed
            and self.target_position is None
            and not self.area_mission.active
        ):
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
            if distance >= 0.001:
                step = min(distance, self.auto_motion_speed * delta_time)
                self.current_position.x += (dx / distance) * step
                self.current_position.y += (dy / distance) * step
                self.current_position.z += (dz / distance) * step

        self._apply_wind_drift(delta_time, self.wind_compensation_hover)
    
    def _handle_takeoff(self, delta_time: float):
        """Handle takeoff procedure"""
        if self.current_position.z < self.TAKEOFF_ALTITUDE:
            self.current_position.z += self.LANDING_SPEED * 2 * delta_time
            self._apply_wind_drift(delta_time, self.wind_compensation_hover)
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
            self._apply_wind_drift(delta_time, self.wind_compensation_flying)
    
    def _handle_return_to_home(self, delta_time: float):
        """Handle return to home procedure"""
        distance = self.current_position.distance_to(self.home_position)
        
        if distance < 1.0 and abs(self.current_position.z - self.home_position.z) < 1.0:
            self.logger.info(f"Drone {self.drone_id}: Reached home position")
            if self.emergency_status.active or self.emergency_return_active or self.degraded_return_active:
                self.flight_mode = FlightMode.EMERGENCY_LANDING
                self.target_position = None
            else:
                self.land()
        else:
            # Move towards home
            dx = self.home_position.x - self.current_position.x
            dy = self.home_position.y - self.current_position.y
            
            horizontal_distance = math.sqrt(dx**2 + dy**2)
            
            if horizontal_distance > 1.0:
                speed_cap = self.MAX_SPEED
                if self.degraded_return_active:
                    speed_cap *= 0.55
                if self.emergency_return_active:
                    speed_cap *= 0.75
                speed = min(speed_cap, horizontal_distance / max(delta_time, 0.001))
                self.current_position.x += (dx / horizontal_distance) * speed * delta_time
                self.current_position.y += (dy / horizontal_distance) * speed * delta_time

            wind_compensation = self.wind_compensation_rth
            if self.degraded_return_active:
                wind_compensation = 0.45
            elif self.emergency_return_active:
                wind_compensation = max(0.55, self.wind_compensation_rth - 0.22)
            self._apply_wind_drift(delta_time, wind_compensation)
            
            # Descend gradually
            if abs(self.current_position.z - self.home_position.z) > 0.5:
                descend_rate = self.LANDING_SPEED * 0.5
                if self.degraded_return_active:
                    descend_rate *= 0.6
                self.current_position.z = max(
                    self.home_position.z,
                    self.current_position.z - descend_rate * delta_time
                )

            if self._horizontal_distance(self.current_position, self.home_position) < 0.35:
                self.current_position.x = self.home_position.x
                self.current_position.y = self.home_position.y
    
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
            self.degraded_return_active = False
            self.emergency_return_active = False
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
        """Emergency behavior: return to home immediately, then perform emergency landing at home."""
        if self.flight_mode in [FlightMode.CRASHED, FlightMode.IDLE]:
            return
        
        if self.emergency_status.active and self.emergency_return_active:
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
        self.emergency_return_active = True
        self.target_position = None
        self.flight_mode = FlightMode.RETURNING_HOME
        self._send_real_drone_command("emergency_return_home", {"reason": reason})
        self.logger.error(
            f"Drone {self.drone_id}: EMERGENCY RETURN HOME - {reason} "
            f"(source={trigger_source}, origin={landing_position.to_dict()})"
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
        if self.degraded_return_active or self.emergency_return_active:
            return False
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
            self.failed_motor_count = len([m for m in self.motors if not m.operational])
            self.motor_failure_warning = self.failed_motor_count > 0
            self.logger.error(f"Drone {self.drone_id}: Motor {motor_id} failed!")
            if self.flight_mode not in [FlightMode.IDLE, FlightMode.CRASHED, FlightMode.EMERGENCY_LANDING]:
                self._activate_degraded_return(
                    "Motor failure simulation: return to home with degraded control",
                    trigger_source="manual_test"
                )
    
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
            "personal_ml_enabled": self.personal_ml_enabled,
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
            "motor_failure_warning": self.motor_failure_warning,
            "failed_motor_count": self.failed_motor_count,
            "degraded_return_active": self.degraded_return_active,
            "emergency_return_active": self.emergency_return_active,
            "wind_vector": self.current_wind_vector.to_dict(),
            "wind_profile": {
                "enabled": self.wind_enabled,
                "speed_mps": round(self.wind_speed_mps, 3),
                "direction_deg": round(self.wind_direction_deg, 2),
                "gust_factor": round(self.wind_gust_factor, 3),
            },
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
