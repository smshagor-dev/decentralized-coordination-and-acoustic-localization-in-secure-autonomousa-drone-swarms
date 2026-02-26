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
Swarm Manager - Manages drone fleet with leader election and fault tolerance
"""

import time
import threading
import logging
import math
import queue
import random
import json
import csv
import re
import copy
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Dict, Tuple
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
from flying_ledger import Ed25519SignatureProvider, FlyingLedger
from acoustic_tracking import AcousticTrackingSystem
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
        # Final goals for non-mission/manual commands, so avoidance can detour
        # without losing the original destination.
        self._manual_goal_targets: Dict[int, Position] = {}
        self._event_notifications: "queue.Queue[dict]" = queue.Queue()
        self._avoidance_active_ids = set()
        self._drone_state_cache: Dict[int, DroneOperationalState] = {}

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
        self._last_latency_ledger_log_at = 0.0

        # Runtime latency graphing (latency vs number of drones)
        self.runtime_latency_graph_enabled = True
        # Disable CSV-derived graph generation by default.
        self.runtime_csv_graph_generation_enabled = False
        # Disable log-derived graph generation by default.
        self.runtime_log_graph_generation_enabled = False
        self._latency_graph_samples = []
        self._last_graph_sample_at = 0.0
        self._last_graph_save_at = 0.0
        self._last_log_merge_at = 0.0
        self._log_offsets: Dict[Path, int] = {}
        self._run_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self._runtime_dir = Path("performance_graphs")
        self._runtime_csv_dir = self._runtime_dir / "csv"
        self._runtime_logs_dir = self._runtime_dir / "logs"
        self._runtime_img_dir = self._runtime_dir / "img"
        self._runtime_image_root_dir = self._runtime_dir / "image"
        self._runtime_image_logs_dir = self._runtime_image_root_dir / "logs" / self._run_id
        self._runtime_image_csv_dir = self._runtime_image_root_dir / "csv" / self._run_id
        self._runtime_csv_path = self._runtime_csv_dir / f"runtime_latency_vs_drones_{self._run_id}.csv"
        self._runtime_png_path = self._runtime_img_dir / f"latency_timeseries_{self._run_id}.png"
        self._merged_log_path = self._runtime_logs_dir / f"merged_logs_{self._run_id}.log"
        self._dynamic_metrics_log_path = self._runtime_logs_dir / f"dynamic_metrics_{self._run_id}.log"
        self._split_logs_root_dir = self._runtime_logs_dir / "by_log" / self._run_id
        self._split_csv_root_dir = self._runtime_csv_dir / "by_log" / self._run_id
        self._metric_logs_dir = self._runtime_logs_dir / "metrics" / self._run_id
        self._metric_csv_dir = self._runtime_csv_dir / "metrics" / self._run_id
        self._flying_logs_dir = self._runtime_logs_dir / "flying" / self._run_id
        self._flying_csv_dir = self._runtime_csv_dir / "flying" / self._run_id
        self._spike_png_path = self._runtime_img_dir / f"latency_spike_timeline_{self._run_id}.png"
        self._spike_csv_path = self._runtime_csv_dir / f"latency_spike_timeline_{self._run_id}.csv"
        self._split_log_paths: Dict[str, Path] = {}
        self._split_csv_paths: Dict[str, Path] = {}
        self._metric_log_paths: Dict[str, Path] = {}
        self._metric_csv_paths: Dict[str, Path] = {}
        self._latest_latency_stats: Dict[str, float] = {}
        self._last_dynamic_metrics_log_at = 0.0
        self._runtime_graphs_finalized = False
        self._latest_acoustic_rmse = 0.0
        self._latest_collision_avoidance_path_m = 0.0
        self._last_leader_election_at = ""
        self._last_leader_election_outcome = "NONE"
        self._last_leader_election_score = 0.0
        self._last_leader_election_candidates = 0

        # Event-driven leader/follower architecture
        self.communication_manager = EventCommunicationManager()
        self.drone_state_manager = DroneStateManager()
        self.gps_navigation_module = GPSNavigationModule()
        self.ml_navigation_module = MLNavigationModule()
        self.leader_command_handler = LeaderCommandHandler(self)

        # Decentralized flying ledger (per-drone mini blockchain replicas)
        self.flying_ledgers: Dict[int, FlyingLedger] = {}
        self._ledger_signature_providers: Dict[int, Ed25519SignatureProvider] = {}
        self._ledger_public_keys: Dict[str, bytes] = {}
        self._ledger_last_sync_status = "IDLE"

        # Acoustic source localization subsystem
        self.acoustic_tracker = AcousticTrackingSystem()
        self.acoustic_detection_enabled = False
        self.acoustic_confidence_threshold = 0.65
        self.acoustic_latency_limit_ms = 280.0
        self._latest_acoustic_source: Optional[Dict[str, float]] = None
        self._latest_acoustic_confidence = 0.0
        self._latest_acoustic_local_only = False
        self.wind_enabled = True
        self.wind_speed_mps = 2.5
        self.wind_direction_deg = 35.0
        self.wind_gust_factor = 0.35
        
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
        self.communication_manager.subscribe("LEDGER_BLOCK", self._on_ledger_block)
        self.communication_manager.subscribe("ACOUSTIC_EVENT", self._on_acoustic_event)
        self.drone_state_manager.register_transition_listener(self._on_state_transition)

    def _on_state_transition(
        self,
        drone_id: int,
        old_state: DroneOperationalState,
        new_state: DroneOperationalState,
    ):
        self._drone_state_cache[drone_id] = new_state
        if new_state == DroneOperationalState.LEDGER_SYNCING:
            return
        self._record_critical_event(
            drone_id=drone_id,
            event_type="STATE_TRANSITION",
            payload={"from": old_state.value, "to": new_state.value},
        )

    def _init_drone_ledger(self, drone_id: int):
        provider = Ed25519SignatureProvider()
        self._ledger_signature_providers[drone_id] = provider
        self._ledger_public_keys[str(drone_id)] = provider.public_key_bytes()
        ledger = FlyingLedger(
            drone_id=str(drone_id),
            signature_provider=provider,
            broadcaster=lambda block, sender=drone_id: self._broadcast_ledger_block(sender, block),
            peer_public_keys=self._ledger_public_keys,
        )
        self.flying_ledgers[drone_id] = ledger
        self._refresh_ledger_peer_keys()

    def _refresh_ledger_peer_keys(self):
        for ledger in self.flying_ledgers.values():
            ledger.set_peer_public_keys(self._ledger_public_keys)

    def _broadcast_ledger_block(self, sender_id: int, block_data: dict):
        self._ledger_last_sync_status = "BROADCASTING"
        accepted = 0
        rejected = 0
        for drone_id, ledger in self.flying_ledgers.items():
            if drone_id == sender_id:
                continue
            if ledger.append_replicated_block(copy.deepcopy(block_data)):
                accepted += 1
            else:
                rejected += 1
        self._ledger_last_sync_status = "SYNCED" if rejected == 0 else "PARTIAL_REJECT"
        self._push_system_event(
            {
                "kind": "message",
                "message_type": "LEDGER_SYNC",
                "data": {
                    "sender_id": int(sender_id),
                    "accepted": accepted,
                    "rejected": rejected,
                    "status": self._ledger_last_sync_status,
                },
            }
        )

    def _on_ledger_block(self, event: dict):
        sender_id = int(event.get("sender_id", -1))
        block_data = event.get("block") or {}
        if sender_id < 0 or not block_data:
            return

        accepted = 0
        rejected = 0
        for drone_id, ledger in self.flying_ledgers.items():
            if drone_id == sender_id:
                continue
            self.drone_state_manager.set_state(drone_id, DroneOperationalState.LEDGER_SYNCING)
            ok = ledger.append_replicated_block(block_data)
            if ok:
                accepted += 1
            else:
                rejected += 1

        self._ledger_last_sync_status = "SYNCED" if rejected == 0 else "PARTIAL_REJECT"
        if rejected:
            self.logger.warning(
                "Ledger block rejected sender=%s accepted=%s rejected=%s",
                sender_id,
                accepted,
                rejected,
            )
        self._push_system_event(
            {
                "kind": "message",
                "message_type": "LEDGER_SYNC",
                "data": {
                    "sender_id": sender_id,
                    "accepted": accepted,
                    "rejected": rejected,
                    "status": self._ledger_last_sync_status,
                },
            }
        )

    def _record_critical_event(
        self,
        drone_id: int,
        event_type: str,
        payload: Optional[dict] = None,
        telemetry_snapshot: Optional[dict] = None,
    ):
        ledger = self.flying_ledgers.get(drone_id)
        drone = self.drones.get(drone_id)
        if ledger is None or drone is None:
            return
        telemetry = telemetry_snapshot or {
            "position": {
                "x": float(drone.current_position.x),
                "y": float(drone.current_position.y),
                "z": float(drone.current_position.z),
            },
            "velocity": {
                "x": float(drone.velocity.x),
                "y": float(drone.velocity.y),
                "z": float(drone.velocity.z),
            },
            "battery": float(drone.battery_level),
            "flight_mode": str(drone.flight_mode.value),
            "is_active": bool(drone.is_active),
        }
        event_payload = {
            "event_type": str(event_type),
            "drone_id": int(drone_id),
            "payload": payload or {},
            "ts": time.time(),
        }
        try:
            ledger.append_local_event(telemetry, event_payload)
        except Exception as exc:
            self.logger.warning("Failed to append ledger event for drone=%s: %s", drone_id, exc)

    def _on_acoustic_event(self, event: dict):
        data = event.get("data", {}) or {}
        self._latest_acoustic_source = data.get("source_position")
        self._latest_acoustic_confidence = float(data.get("confidence", 0.0))
        self._latest_acoustic_local_only = bool(data.get("local_only", False))
        if self._latest_acoustic_source:
            self._push_system_event(
                {
                    "kind": "message",
                    "message_type": "ACOUSTIC_EVENT",
                    "data": data,
                }
            )

    def set_acoustic_detection_enabled(self, enabled: bool):
        self.acoustic_detection_enabled = bool(enabled)

    def set_acoustic_confidence_threshold(self, threshold: float):
        self.acoustic_confidence_threshold = max(0.0, min(1.0, float(threshold)))

    def set_wind_conditions(
        self,
        speed_mps: float,
        direction_deg: float,
        gust_factor: float = 0.35,
        enabled: bool = True,
    ):
        """Apply a global wind profile to all drones in the simulator."""
        self.wind_enabled = bool(enabled)
        self.wind_speed_mps = max(0.0, min(30.0, float(speed_mps)))
        self.wind_direction_deg = float(direction_deg) % 360.0
        self.wind_gust_factor = max(0.0, min(0.95, float(gust_factor)))
        for drone in self.drones.values():
            drone.set_wind_conditions(
                speed_mps=self.wind_speed_mps,
                direction_deg=self.wind_direction_deg,
                gust_factor=self.wind_gust_factor,
                enabled=self.wind_enabled,
            )
        self.logger.info(
            "Global wind updated: enabled=%s speed=%.2f dir=%.1f gust=%.2f",
            self.wind_enabled,
            self.wind_speed_mps,
            self.wind_direction_deg,
            self.wind_gust_factor,
        )

    def move_formation_to(self, source_position: dict):
        targets: Dict[int, Position] = {}
        for drone_id, drone in self.drones.items():
            target = Position(
                float(source_position.get("x", drone.current_position.x)),
                float(source_position.get("y", drone.current_position.y)),
                max(5.0, float(drone.current_position.z)),
            )
            targets[drone_id] = target
        if targets:
            self.leader_move_to_target(targets)

    def process_acoustic_signals(
        self,
        signals: Dict[int, object],
        sample_rate_hz: float,
        total_round_trip_ms: Optional[float] = None,
    ) -> dict:
        if not self.acoustic_detection_enabled:
            return {"detected": False, "reason": "disabled", "confidence": 0.0}

        sensor_positions: Dict[int, Tuple[float, float]] = {}
        for drone_id, drone in self.drones.items():
            sensor_positions[drone_id] = (float(drone.current_position.x), float(drone.current_position.y))

        latency_ms = (
            float(total_round_trip_ms)
            if total_round_trip_ms is not None
            else float(self._latest_latency_stats.get("total_round_trip_ms", 0.0))
        )
        result = self.acoustic_tracker.localize(
            signals=signals,
            sensor_positions=sensor_positions,
            sample_rate_hz=float(sample_rate_hz),
            total_round_trip_ms=latency_ms,
            acoustic_latency_limit_ms=self.acoustic_latency_limit_ms,
        )

        if result.get("local_only"):
            self._record_critical_event(
                drone_id=self.leader_id or next(iter(self.drones.keys()), -1),
                event_type="ACOUSTIC_LATENCY_LOCAL_ONLY",
                payload={"latency_ms": latency_ms, "limit_ms": self.acoustic_latency_limit_ms},
            )

        if not result.get("detected"):
            return result

        confidence = float(result.get("confidence", 0.0))
        if confidence < self.acoustic_confidence_threshold:
            result["ignored"] = True
            return result

        source_position = result.get("source_position")
        self._latest_acoustic_source = source_position
        self._latest_acoustic_confidence = confidence
        self._latest_acoustic_local_only = bool(result.get("local_only", False))
        self._latest_acoustic_rmse = float(result.get("rmse", 0.0))

        event_payload = {
            "source_position": source_position,
            "confidence": confidence,
            "rmse": float(result.get("rmse", 0.0)),
            "local_only": bool(result.get("local_only", False)),
            "latency_ms": latency_ms,
        }
        self.communication_manager.publish("ACOUSTIC_EVENT", {"data": event_payload})
        self.move_formation_to(source_position or {})
        for drone in self.drones.values():
            self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.ACOUSTIC_TRACKING)
        self._record_critical_event(
            drone_id=self.leader_id or next(iter(self.drones.keys()), -1),
            event_type="ACOUSTIC_DETECTION",
            payload=event_payload,
        )
        return result

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
            track_mission = bool(payload.get("track_mission", True))

            if track_mission:
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
                    if track_mission:
                        self._manual_goal_targets.pop(drone_id, None)
                        self._mission_targets[drone_id] = target
                    else:
                        self._manual_goal_targets[drone_id] = target

    def _execute_leader_return_home(self):
        """Leader broadcasted return home; mission-active drones must ignore until mission complete."""
        with self._lock:
            returned_ids = set()
            skipped_ids = set()
            for drone in self.drones.values():
                pending_single_mission = self._drone_has_pending_single_mission(drone)
                if (
                    self.drone_state_manager.is_gps_ml_active(drone.drone_id)
                    or drone.area_mission.active
                    or pending_single_mission
                ):
                    self.logger.info(
                        "Drone %s ignored RETURN_TO_HOME due to active mission state",
                        drone.drone_id,
                    )
                    skipped_ids.add(drone.drone_id)
                    continue
                drone.return_to_home("Leader broadcast RETURN_TO_HOME")
                self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.RETURNING_HOME)
                returned_ids.add(drone.drone_id)
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

            # Keep mission maps for skipped drones so they can continue/complete mission.
            for drone_id in list(self._mission_targets.keys()):
                if drone_id in returned_ids:
                    self._mission_targets.pop(drone_id, None)
            for drone_id in list(self._manual_goal_targets.keys()):
                if drone_id in returned_ids:
                    self._manual_goal_targets.pop(drone_id, None)
            if not self._mission_targets:
                self._mission_active = False
            if skipped_ids:
                self.logger.info(
                    "RETURN_TO_HOME skipped for mission-active drones: %s",
                    sorted(skipped_ids),
                )

    def _drone_has_pending_single_mission(self, drone: Drone) -> bool:
        """True when this drone still has an unfinished single-target mission."""
        target = self._mission_targets.get(drone.drone_id)
        if target is not None:
            dx = float(target.x) - float(drone.current_position.x)
            dy = float(target.y) - float(drone.current_position.y)
            if math.sqrt(dx * dx + dy * dy) > float(self._mission_arrival_threshold_m):
                return True
        manual_target = self._manual_goal_targets.get(drone.drone_id)
        if manual_target is not None:
            dx = float(manual_target.x) - float(drone.current_position.x)
            dy = float(manual_target.y) - float(drone.current_position.y)
            if math.sqrt(dx * dx + dy * dy) > float(self._mission_arrival_threshold_m):
                return True
        return False

    def _command_move_for_drone(self, drone: Drone, target: Position, payload: dict) -> bool:
        """Route movement through GPS+ML module if active, else default swarm goto."""
        if not self._prepare_drone_for_goto(drone, preferred_alt=target.z):
            return False

        if drone.area_mission.active or self.gps_navigation_module.is_active(payload, drone.drone_id):
            ok = self.ml_navigation_module.navigate(drone, target)
            if ok:
                current = self.drone_state_manager.get_state(drone.drone_id)
                if current != DroneOperationalState.ACOUSTIC_TRACKING:
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.GPS_ML_ACTIVE)
            return ok

        ok = drone.goto(target)
        if ok:
            current = self.drone_state_manager.get_state(drone.drone_id)
            if current != DroneOperationalState.ACOUSTIC_TRACKING:
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
            self._init_drone_ledger(drone.drone_id)
            drone.set_wind_conditions(
                speed_mps=self.wind_speed_mps,
                direction_deg=self.wind_direction_deg,
                gust_factor=self.wind_gust_factor,
                enabled=self.wind_enabled,
            )
            
            # Start drone systems
            drone.start()
            
            self.logger.info(f"Drone {drone.drone_id} added to swarm")
            self._record_critical_event(
                drone_id=drone.drone_id,
                event_type="DRONE_JOINED",
                payload={"leader_id": self.leader_id},
            )
            
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
            self.flying_ledgers.pop(drone_id, None)
            self._ledger_signature_providers.pop(drone_id, None)
            self._ledger_public_keys.pop(str(drone_id), None)
            self._refresh_ledger_peer_keys()
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
            self._init_runtime_artifacts()
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
        self.finalize_runtime_graphs()
        self.logger.info("Swarm monitoring stopped")

    def finalize_runtime_graphs(self, force: bool = False):
        """
        Final graph generation on shutdown:
        - Merge any remaining log lines
        - Generate summary runtime graphs from CSV (optional)
        - Generate per-file graphs from CSV tree (optional)
        - LOG-derived graphs are optional and disabled by default
        """
        if not self.runtime_latency_graph_enabled:
            return
        if self._runtime_graphs_finalized and not force:
            return
        self._init_runtime_artifacts()
        self._merge_runtime_logs()
        if self.runtime_csv_graph_generation_enabled:
            self._save_latency_timeseries_from_csv()
            self._generate_per_file_graphs_from_csv_tree()
        if self.runtime_log_graph_generation_enabled:
            self._save_latency_spike_timeline_from_logs()
            self._generate_per_file_graphs_from_log_tree()
        self._runtime_graphs_finalized = True
    
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
            self._latest_latency_stats = dict(latency_stats)

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
                now = time.time()
                if now - self._last_latency_ledger_log_at >= 1.0:
                    self._last_latency_ledger_log_at = now
                    self._record_critical_event(
                        drone_id=self.leader_id or next(iter(self.drones.keys()), -1),
                        event_type="ML_BRIDGE_TIMEOUT" if watchdog_timed_out else "LATENCY_THRESHOLD_BREACH",
                        payload={
                            "watchdog_timed_out": watchdog_timed_out,
                            "latency_stats": latency_stats,
                        },
                    )
            else:
                self.fallback_local_avoidance_mode = False

            # Auto-collect latency vs drone-count samples and save graph periodically.
            self._record_latency_graph_sample(latency_stats)
            self._append_dynamic_metrics_log(latency_stats)

            # Keep high-level state model and event-driven mission completion in sync.
            self._synchronize_operational_states()
            self._check_mission_arrivals()
            
            time.sleep(1.0)

    def _record_latency_graph_sample(self, latency_stats: dict):
        if not self.runtime_latency_graph_enabled:
            return
        now = time.time()
        if now - self._last_graph_sample_at < 2.0:
            return
        self._last_graph_sample_at = now
        drone_count = len(self.drones)
        total_ms = float(latency_stats.get("total_round_trip_ms", 0.0))
        self._latency_graph_samples.append((now, drone_count, total_ms))
        self._append_runtime_csv_sample(now, drone_count, total_ms)
        if now - self._last_log_merge_at >= 2.0:
            self._last_log_merge_at = now
            self._merge_runtime_logs()

    def _init_runtime_artifacts(self):
        self._runtime_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_csv_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_logs_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_img_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_image_root_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_image_logs_dir.mkdir(parents=True, exist_ok=True)
        self._runtime_image_csv_dir.mkdir(parents=True, exist_ok=True)
        self._split_logs_root_dir.mkdir(parents=True, exist_ok=True)
        self._split_csv_root_dir.mkdir(parents=True, exist_ok=True)
        self._metric_logs_dir.mkdir(parents=True, exist_ok=True)
        self._metric_csv_dir.mkdir(parents=True, exist_ok=True)
        self._flying_logs_dir.mkdir(parents=True, exist_ok=True)
        self._flying_csv_dir.mkdir(parents=True, exist_ok=True)
        if not self._runtime_csv_path.exists():
            try:
                with self._runtime_csv_path.open("w", encoding="utf-8") as f:
                    f.write("timestamp,drone_count,latency_ms,drone_status_json\n")
            except Exception as exc:
                self.logger.warning("Failed to initialize runtime CSV: %s", exc)
        if not self._merged_log_path.exists():
            try:
                with self._merged_log_path.open("w", encoding="utf-8") as f:
                    f.write(f"merged_log_run_id={self._run_id}\n")
            except Exception as exc:
                self.logger.warning("Failed to initialize merged log: %s", exc)
        if not self._dynamic_metrics_log_path.exists():
            try:
                with self._dynamic_metrics_log_path.open("w", encoding="utf-8") as f:
                    f.write(f"dynamic_metric_run_id={self._run_id}\n")
            except Exception as exc:
                self.logger.warning("Failed to initialize dynamic metrics log: %s", exc)
        self._init_metric_artifacts()
        # Initialize log offsets so merged log only contains new data from this run.
        log_dir = Path("logs")
        candidates = [
            log_dir / "swarm_manager.log",
            log_dir / "ml_system.log",
        ]
        candidates.extend(sorted(log_dir.glob("system_*.log")))
        for path in candidates:
            if not path.exists():
                continue
            try:
                self._log_offsets[path] = path.stat().st_size
            except Exception:
                self._log_offsets[path] = 0

    def _init_metric_artifacts(self):
        metric_names = [
            "acoustic_localization_error",
            "security_computational_overhead",
            "collision_avoidance_path",
            "winds",
            "flying",
            "leader_election",
        ]
        for metric in metric_names:
            metric_log_name = f"{metric}_log.log"
            metric_csv_name = f"{metric}_log.csv"
            log_dir = self._flying_logs_dir if metric == "flying" else self._metric_logs_dir
            csv_dir = self._flying_csv_dir if metric == "flying" else self._metric_csv_dir
            log_path = log_dir / metric_log_name
            csv_path = csv_dir / metric_csv_name
            self._metric_log_paths[metric] = log_path
            self._metric_csv_paths[metric] = csv_path
            if not log_path.exists():
                try:
                    with log_path.open("w", encoding="utf-8") as f:
                        f.write(f"{metric}_log_run_id={self._run_id}\n")
                except Exception as exc:
                    self.logger.warning("Failed to initialize metric log %s: %s", metric, exc)
            if not csv_path.exists():
                try:
                    import csv
                    with csv_path.open("w", encoding="utf-8", newline="") as f:
                        writer = csv.writer(f)
                        if metric == "winds":
                            writer.writerow(["timestamp", "enabled", "speed_mps", "direction_deg", "gust_factor"])
                        elif metric == "flying":
                            writer.writerow(["timestamp", "flying_count", "total_drones", "avg_speed_mps"])
                        elif metric == "leader_election":
                            writer.writerow(
                                [
                                    "timestamp",
                                    "state",
                                    "leader_id",
                                    "last_outcome",
                                    "candidates",
                                    "score",
                                    "elected_at",
                                ]
                            )
                        else:
                            writer.writerow(["timestamp", "value"])
                except Exception as exc:
                    self.logger.warning("Failed to initialize metric CSV %s: %s", metric, exc)

    def _to_log_key(self, source_name: str) -> str:
        return re.sub(r"[^a-z0-9]+", "_", source_name.lower()).strip("_")

    def _parse_standard_log_line(self, line: str) -> Tuple[str, str, str, str]:
        parts = line.split(" - ", 3)
        if len(parts) == 4:
            return parts[0], parts[1], parts[2], parts[3]
        return "", "", "", line

    def _append_split_log_line(self, source_name: str, line: str):
        key = self._to_log_key(Path(source_name).stem)
        if not key:
            key = "unknown"
        log_path = self._split_log_paths.get(key)
        csv_path = self._split_csv_paths.get(key)
        if log_path is None:
            log_path = self._split_logs_root_dir / f"{key}_log.log"
            self._split_log_paths[key] = log_path
            if not log_path.exists():
                with log_path.open("w", encoding="utf-8") as f:
                    f.write(f"{key}_log_run_id={self._run_id}\n")
        if csv_path is None:
            csv_path = self._split_csv_root_dir / f"{key}_log.csv"
            self._split_csv_paths[key] = csv_path
            if not csv_path.exists():
                import csv
                with csv_path.open("w", encoding="utf-8", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["timestamp", "logger", "level", "message", "raw_line"])
        with log_path.open("a", encoding="utf-8") as f:
            f.write(f"{line}\n")
        ts, logger_name, level_name, message = self._parse_standard_log_line(line)
        import csv
        with csv_path.open("a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
            writer.writerow([ts, logger_name, level_name, message, line])

    def _append_runtime_csv_sample(self, ts: float, drone_count: int, total_ms: float):
        try:
            status = self.get_swarm_status()
            drones_status = status.get("drones", {})
            payload = json.dumps(drones_status, separators=(",", ":"), ensure_ascii=False)
            import csv
            with self._runtime_csv_path.open("a", encoding="utf-8", newline="") as f:
                writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
                writer.writerow([f"{ts:.3f}", drone_count, f"{total_ms:.3f}", payload])
        except Exception as exc:
            self.logger.warning("Failed to append runtime CSV: %s", exc)

    def _merge_runtime_logs(self):
        log_dir = Path("logs")
        candidates = [
            log_dir / "swarm_manager.log",
            log_dir / "ml_system.log",
        ]
        candidates.extend(sorted(log_dir.glob("system_*.log")))
        for path in candidates:
            if not path.exists():
                continue
            offset = self._log_offsets.get(path, 0)
            try:
                with path.open("r", encoding="utf-8", errors="ignore") as f:
                    f.seek(offset)
                    new_data = f.read()
                    self._log_offsets[path] = f.tell()
                if not new_data:
                    continue
                with self._merged_log_path.open("a", encoding="utf-8") as out:
                    for line in new_data.splitlines():
                        out.write(f"[{path.name}] {line}\n")
                        self._append_split_log_line(path.name, line)
            except Exception as exc:
                self.logger.warning("Failed to merge log %s: %s", path, exc)

    def _compute_security_computational_overhead_pct(self, latency_stats: dict) -> float:
        total_ms = float(latency_stats.get("total_round_trip_ms", 0.0))
        jitter_ms = float(latency_stats.get("total_round_trip_jitter_std_ms", 0.0))
        drone_factor = float(len(self.drones)) * 1.8
        ledger_factor = float(len(self.flying_ledgers)) * 1.2
        fallback_factor = 8.0 if self.fallback_local_avoidance_mode else 0.0
        overhead = (total_ms / 10.0) + (jitter_ms * 0.6) + drone_factor + ledger_factor + fallback_factor
        return max(0.0, min(100.0, overhead))

    def _append_metric_log_line(self, metric: str, ts: str, message: str):
        path = self._metric_log_paths.get(metric)
        if path is None:
            return
        with path.open("a", encoding="utf-8") as f:
            f.write(f"[metrics.log] {ts} - PerformanceMetrics - INFO - {message}\n")

    def _append_metric_csv_row(self, metric: str, row: list):
        path = self._metric_csv_paths.get(metric)
        if path is None:
            return
        import csv
        with path.open("a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    def _append_dynamic_metrics_log(self, latency_stats: dict):
        now = time.time()
        if now - self._last_dynamic_metrics_log_at < 2.0:
            return
        self._last_dynamic_metrics_log_at = now
        try:
            acoustic_error_m = max(0.0, float(self._latest_acoustic_rmse))
            security_overhead_pct = self._compute_security_computational_overhead_pct(latency_stats)
            collision_path_m = max(0.0, float(self._latest_collision_avoidance_path_m))
            flying_drones = self.get_active_drones()
            flying_count = len(flying_drones)
            avg_flying_speed = 0.0
            if flying_count:
                speed_sum = 0.0
                for drone in flying_drones:
                    speed_sum += math.sqrt(
                        float(drone.velocity.x) ** 2
                        + float(drone.velocity.y) ** 2
                        + float(drone.velocity.z) ** 2
                    )
                avg_flying_speed = speed_sum / flying_count
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]
            with self._dynamic_metrics_log_path.open("a", encoding="utf-8") as f:
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Acoustic Localization Error={acoustic_error_m:.2f} m\n"
                )
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Security & Computational Overhead={security_overhead_pct:.2f} %\n"
                )
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Collision Avoidance Path={collision_path_m:.2f} m\n"
                )
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Wind Profile enabled={self.wind_enabled} speed={float(self.wind_speed_mps):.2f} mps dir={float(self.wind_direction_deg):.1f} deg gust={float(self.wind_gust_factor):.2f}\n"
                )
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Flying Drones={flying_count}/{len(self.drones)} avg_speed={avg_flying_speed:.2f} mps\n"
                )
                f.write(
                    f"[metrics.log] {ts} - PerformanceMetrics - INFO - Leader Election state={'IN_PROGRESS' if self.election_in_progress else 'IDLE'} leader_id={self.leader_id if self.leader_id is not None else -1} last_outcome={self._last_leader_election_outcome} candidates={self._last_leader_election_candidates} score={self._last_leader_election_score:.2f} at={self._last_leader_election_at or 'N/A'}\n"
                )

            leader_state = "IN_PROGRESS" if self.election_in_progress else "IDLE"
            leader_id = self.leader_id if self.leader_id is not None else -1
            self._append_metric_log_line(
                "acoustic_localization_error", ts, f"Acoustic Localization Error={acoustic_error_m:.2f} m"
            )
            self._append_metric_csv_row("acoustic_localization_error", [ts, f"{acoustic_error_m:.2f}"])
            self._append_metric_log_line(
                "security_computational_overhead",
                ts,
                f"Security & Computational Overhead={security_overhead_pct:.2f} %",
            )
            self._append_metric_csv_row("security_computational_overhead", [ts, f"{security_overhead_pct:.2f}"])
            self._append_metric_log_line(
                "collision_avoidance_path", ts, f"Collision Avoidance Path={collision_path_m:.2f} m"
            )
            self._append_metric_csv_row("collision_avoidance_path", [ts, f"{collision_path_m:.2f}"])
            self._append_metric_log_line(
                "winds",
                ts,
                f"Wind Profile enabled={self.wind_enabled} speed={float(self.wind_speed_mps):.2f} mps dir={float(self.wind_direction_deg):.1f} deg gust={float(self.wind_gust_factor):.2f}",
            )
            self._append_metric_csv_row(
                "winds",
                [
                    ts,
                    str(bool(self.wind_enabled)),
                    f"{float(self.wind_speed_mps):.2f}",
                    f"{float(self.wind_direction_deg):.1f}",
                    f"{float(self.wind_gust_factor):.2f}",
                ],
            )
            self._append_metric_log_line(
                "flying",
                ts,
                f"Flying Drones={flying_count}/{len(self.drones)} avg_speed={avg_flying_speed:.2f} mps",
            )
            self._append_metric_csv_row(
                "flying",
                [ts, int(flying_count), int(len(self.drones)), f"{avg_flying_speed:.2f}"],
            )
            self._append_metric_log_line(
                "leader_election",
                ts,
                f"Leader Election state={leader_state} leader_id={leader_id} last_outcome={self._last_leader_election_outcome} candidates={self._last_leader_election_candidates} score={self._last_leader_election_score:.2f} at={self._last_leader_election_at or 'N/A'}",
            )
            self._append_metric_csv_row(
                "leader_election",
                [
                    ts,
                    leader_state,
                    int(leader_id),
                    self._last_leader_election_outcome,
                    int(self._last_leader_election_candidates),
                    f"{self._last_leader_election_score:.2f}",
                    self._last_leader_election_at or "N/A",
                ],
            )
        except Exception as exc:
            self.logger.warning("Failed to append dynamic metrics log: %s", exc)

    def _save_latency_timeseries_from_csv(self):
        if not self._runtime_csv_path.exists():
            return
        times = []
        latencies = []
        try:
            with self._runtime_csv_path.open("r", encoding="utf-8", errors="ignore") as f:
                header = f.readline()
                if not header:
                    return
                for line in f:
                    parts = line.strip().split(",", 3)
                    if len(parts) < 3:
                        continue
                    try:
                        ts = float(parts[0])
                        ms = float(parts[2])
                    except ValueError:
                        continue
                    times.append(datetime.fromtimestamp(ts))
                    latencies.append(ms)
        except Exception as exc:
            self.logger.warning("Failed to read runtime CSV for graph: %s", exc)
            return

        if not times:
            return

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.logger.warning("matplotlib unavailable for latency graph: %s", exc)
            return

        try:
            plt.figure(figsize=(9, 4.5))
            plt.plot(times, latencies, linewidth=1.8, color="#1b5e20")
            plt.title("Latency Over Time (Runtime)")
            plt.xlabel("Time")
            plt.ylabel("Round-Trip Latency (ms)")
            plt.grid(True, linestyle="--", alpha=0.4)
            plt.tight_layout()
            plt.savefig(self._runtime_png_path, dpi=160)
        except Exception as exc:
            self.logger.warning("Failed to save latency graph PNG: %s", exc)
        finally:
            plt.close("all")

    def _save_latency_spike_timeline_from_logs(self):
        """
        Parse merged logs and plot latency spike counts over time.
        """
        if not self._merged_log_path.exists():
            return
        spike_pattern = re.compile(r"latency spike", re.IGNORECASE)
        event_times = []
        try:
            with self._merged_log_path.open("r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if not spike_pattern.search(line):
                        continue
                    # Expect timestamp at start of log line: YYYY-MM-DD HH:MM:SS,mmm
                    ts_match = re.search(r"(20\\d{2}-\\d{2}-\\d{2} \\d{2}:\\d{2}:\\d{2}),\\d{3}", line)
                    if not ts_match:
                        continue
                    try:
                        ts = datetime.strptime(ts_match.group(1), "%Y-%m-%d %H:%M:%S")
                        event_times.append(ts)
                    except ValueError:
                        continue
        except Exception as exc:
            self.logger.warning("Failed to parse merged logs for spike timeline: %s", exc)
            return

        if not event_times:
            return

        # Bucket counts per minute
        buckets: Dict[datetime, int] = {}
        for ts in event_times:
            minute = ts.replace(second=0, microsecond=0)
            buckets[minute] = buckets.get(minute, 0) + 1
        times = sorted(buckets.keys())
        counts = [buckets[t] for t in times]

        # Write CSV
        try:
            with self._spike_csv_path.open("w", encoding="utf-8") as f:
                f.write("minute,count\n")
                for t, c in zip(times, counts):
                    f.write(f"{t.strftime('%Y-%m-%d %H:%M')},{c}\n")
        except Exception as exc:
            self.logger.warning("Failed to write spike timeline CSV: %s", exc)

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.logger.warning("matplotlib unavailable for spike timeline: %s", exc)
            return

        try:
            plt.figure(figsize=(9, 4.5))
            plt.plot(times, counts, marker="o", linewidth=2, color="#b71c1c")
            plt.title("Latency Spike Count vs Time (Per Minute)")
            plt.xlabel("Time")
            plt.ylabel("Spike Count")
            plt.grid(True, linestyle="--", alpha=0.4)
            plt.tight_layout()
            plt.savefig(self._spike_png_path, dpi=160)
        except Exception as exc:
            self.logger.warning("Failed to save spike timeline PNG: %s", exc)
        finally:
            plt.close("all")

    def _image_path_for_source(self, source_path: Path, source_root: Path, image_root: Path) -> Path:
        rel = source_path.relative_to(source_root)
        return image_root / rel.with_suffix(".png")

    def _try_parse_time_text(self, value: str):
        text = (value or "").strip()
        if not text:
            return None
        for fmt in ("%Y-%m-%d %H:%M:%S,%f", "%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M"):
            try:
                return datetime.strptime(text, fmt)
            except ValueError:
                pass
        try:
            ts = float(text)
            if ts > 100000000:
                return datetime.fromtimestamp(ts)
        except ValueError:
            return None
        return None

    def _to_float(self, value):
        if value is None:
            return None
        text = str(value).strip()
        if not text:
            return None
        if text.lower() in {"true", "false"}:
            return 1.0 if text.lower() == "true" else 0.0
        try:
            return float(text)
        except ValueError:
            return None

    def _plot_csv_file(self, csv_path: Path, out_path: Path):
        def _save_placeholder(reason: str):
            try:
                import matplotlib.pyplot as plt
            except Exception:
                return
            try:
                fig, ax = plt.subplots(figsize=(9, 4.5))
                ax.axis("off")
                ax.text(
                    0.5,
                    0.5,
                    f"{csv_path.stem}\n{reason}",
                    ha="center",
                    va="center",
                    fontsize=12,
                )
                out_path.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(out_path, dpi=160)
            except Exception as exc:
                self.logger.warning("Failed to save CSV placeholder %s: %s", out_path, exc)
            finally:
                plt.close("all")

        rows = []
        try:
            with csv_path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row:
                        rows.append(row)
        except Exception as exc:
            self.logger.warning("Failed to read CSV %s for graph: %s", csv_path, exc)
            _save_placeholder("Read error")
            return
        if not rows:
            _save_placeholder("No data rows")
            return

        time_col = None
        for key in rows[0].keys():
            k = (key or "").strip().lower()
            if k in {"timestamp", "time", "datetime", "date", "minute"}:
                time_col = key
                break

        x_values = []
        series: Dict[str, List[Tuple[int, float]]] = {}
        for idx, row in enumerate(rows):
            x = idx
            if time_col:
                parsed = self._try_parse_time_text(str(row.get(time_col, "")))
                if parsed is not None:
                    x = parsed
            x_values.append(x)
            for col, raw in row.items():
                if col == time_col:
                    continue
                num = self._to_float(raw)
                if num is None:
                    continue
                series.setdefault(col, []).append((idx, num))

        if not series:
            _save_placeholder("No numeric columns found")
            return

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.logger.warning("matplotlib unavailable for CSV graph %s: %s", csv_path, exc)
            return

        try:
            plt.figure(figsize=(10, 5))
            for col, points in series.items():
                xs = [x_values[i] for i, _ in points]
                ys = [v for _, v in points]
                plt.plot(xs, ys, linewidth=1.6, label=col)
            plt.title(csv_path.stem)
            plt.xlabel(time_col or "row_index")
            plt.ylabel("value")
            plt.grid(True, linestyle="--", alpha=0.35)
            if len(series) <= 10:
                plt.legend(loc="best", fontsize=8)
            plt.tight_layout()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(out_path, dpi=160)
        except Exception as exc:
            self.logger.warning("Failed to save CSV graph %s: %s", out_path, exc)
        finally:
            plt.close("all")

    def _plot_log_file(self, log_path: Path, out_path: Path):
        def _save_placeholder(reason: str):
            try:
                import matplotlib.pyplot as plt
            except Exception:
                return
            try:
                fig, ax = plt.subplots(figsize=(9, 4.5))
                ax.axis("off")
                ax.text(
                    0.5,
                    0.5,
                    f"{log_path.stem}\n{reason}",
                    ha="center",
                    va="center",
                    fontsize=12,
                )
                out_path.parent.mkdir(parents=True, exist_ok=True)
                fig.savefig(out_path, dpi=160)
            except Exception as exc:
                self.logger.warning("Failed to save LOG placeholder %s: %s", out_path, exc)
            finally:
                plt.close("all")

        timestamp_re = re.compile(r"(20\d{2}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:,\d{3})?)")
        metric_re = re.compile(r"([A-Za-z][A-Za-z0-9_ /&-]*?)=([-+]?\d*\.?\d+)")
        level_re = re.compile(r"\s-\s(DEBUG|INFO|WARNING|ERROR|CRITICAL)\s-")

        raw_x_values = []
        level_sequence = []
        logger_counts: Dict[str, int] = {}
        series: Dict[str, List[Tuple[int, float]]] = {}

        try:
            event_idx = -1
            with log_path.open("r", encoding="utf-8", errors="ignore") as f:
                for line_idx, line in enumerate(f):
                    text = line.strip()
                    if not text:
                        continue
                    if "_run_id=" in text and " - " not in text:
                        continue
                    event_idx += 1
                    ts_match = timestamp_re.search(text)
                    x = event_idx
                    if ts_match:
                        parsed = self._try_parse_time_text(ts_match.group(1))
                        if parsed is not None:
                            x = parsed
                    raw_x_values.append(x)

                    level_match = level_re.search(text)
                    if level_match:
                        level_sequence.append(level_match.group(1))
                    else:
                        level_sequence.append("INFO")

                    parts = text.split(" - ", 3)
                    if len(parts) == 4:
                        logger_name = parts[1].strip()
                        if logger_name:
                            logger_counts[logger_name] = logger_counts.get(logger_name, 0) + 1

                    found_metric = False
                    for key, raw in metric_re.findall(text):
                        num = self._to_float(raw)
                        if num is None:
                            continue
                        found_metric = True
                        label = re.sub(r"\s+", " ", key).strip()
                        series.setdefault(label, []).append((event_idx, num))
        except Exception as exc:
            self.logger.warning("Failed to read LOG %s for graph: %s", log_path, exc)
            _save_placeholder("Read error")
            return

        if not raw_x_values:
            _save_placeholder("No log events")
            return

        x_axis = list(range(len(raw_x_values)))
        level_names = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        level_colors = {
            "DEBUG": "#4e79a7",
            "INFO": "#59a14f",
            "WARNING": "#f28e2b",
            "ERROR": "#e15759",
            "CRITICAL": "#b51f2e",
        }
        cumulative: Dict[str, List[int]] = {name: [] for name in level_names}
        running = {name: 0 for name in level_names}
        for level in level_sequence:
            if level not in running:
                level = "INFO"
            running[level] += 1
            for name in level_names:
                cumulative[name].append(running[name])

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.logger.warning("matplotlib unavailable for LOG graph %s: %s", log_path, exc)
            return

        try:
            fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)

            # Panel 1: cumulative severity trend to ensure every log has a useful signal.
            for name in level_names:
                ys = cumulative[name]
                if not ys or ys[-1] == 0:
                    continue
                axes[0].plot(
                    x_axis,
                    ys,
                    linewidth=1.8,
                    label=name,
                    color=level_colors.get(name, "#4e79a7"),
                )
            axes[0].set_title(f"{log_path.stem} - Severity Trend")
            axes[0].set_xlabel("Log Event Index")
            axes[0].set_ylabel("Cumulative Count")
            axes[0].grid(True, linestyle="--", alpha=0.35)
            axes[0].legend(loc="upper left", fontsize=8)

            # Panel 2: numeric metrics if present, otherwise logger activity bars.
            if series:
                ranked = sorted(series.items(), key=lambda item: len(item[1]), reverse=True)
                for label, points in ranked[:6]:
                    xs = [x_axis[i] for i, _ in points if 0 <= i < len(x_axis)]
                    ys = [v for i, v in points if 0 <= i < len(x_axis)]
                    if xs and ys:
                        axes[1].plot(xs, ys, linewidth=1.6, label=label)
                axes[1].set_title("Top Numeric Metrics")
                axes[1].set_xlabel("Log Event Index")
                axes[1].set_ylabel("Value")
                axes[1].grid(True, linestyle="--", alpha=0.35)
                axes[1].legend(loc="best", fontsize=8)
            else:
                sorted_loggers = sorted(logger_counts.items(), key=lambda item: item[1], reverse=True)
                labels = [name for name, _ in sorted_loggers[:8]]
                values = [count for _, count in sorted_loggers[:8]]
                if not labels:
                    labels = ["events"]
                    values = [len(raw_x_values)]
                axes[1].bar(labels, values, color="#4e79a7")
                axes[1].set_title("Top Logger Activity")
                axes[1].set_xlabel("Logger")
                axes[1].set_ylabel("Events")
                axes[1].grid(True, axis="y", linestyle="--", alpha=0.35)
                axes[1].tick_params(axis="x", rotation=20)

            out_path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(out_path, dpi=160)
        except Exception as exc:
            self.logger.warning("Failed to save LOG graph %s: %s", out_path, exc)
        finally:
            plt.close("all")

    def _generate_per_file_graphs_from_csv_tree(self):
        if not self._runtime_csv_dir.exists():
            return
        for csv_path in sorted(self._runtime_csv_dir.rglob("*.csv")):
            out_path = self._image_path_for_source(
                source_path=csv_path,
                source_root=self._runtime_csv_dir,
                image_root=self._runtime_image_csv_dir,
            )
            self._plot_csv_file(csv_path, out_path)

    def _generate_per_file_graphs_from_log_tree(self):
        if not self._runtime_logs_dir.exists():
            return
        for log_path in sorted(self._runtime_logs_dir.rglob("*.log")):
            out_path = self._image_path_for_source(
                source_path=log_path,
                source_root=self._runtime_logs_dir,
                image_root=self._runtime_image_logs_dir,
            )
            self._plot_log_file(log_path, out_path)

    def _synchronize_operational_states(self):
        """Map low-level flight mode to high-level operational state."""
        with self._lock:
            for drone in self.drones.values():
                current_state = self.drone_state_manager.get_state(drone.drone_id)
                if (
                    drone.flight_mode == FlightMode.EMERGENCY_LANDING
                    or drone.emergency_return_active
                    or drone.emergency_status.active
                ):
                    # Emergency drones should be treated as non-active in control UI.
                    self._manual_goal_targets.pop(drone.drone_id, None)
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.IDLE)
                    continue
                if drone.flight_mode == FlightMode.IDLE:
                    self._manual_goal_targets.pop(drone.drone_id, None)
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.IDLE)
                    continue
                if drone.flight_mode == FlightMode.TAKEOFF:
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.TAKEOFF)
                    continue
                if drone.flight_mode == FlightMode.RETURNING_HOME:
                    self._manual_goal_targets.pop(drone.drone_id, None)
                    self.drone_state_manager.set_state(drone.drone_id, DroneOperationalState.RETURNING_HOME)
                    continue
                if drone.drone_id in self._avoidance_active_ids:
                    self.drone_state_manager.set_state(
                        drone.drone_id, DroneOperationalState.AVOIDING_DYNAMIC_OBSTACLE
                    )
                    continue
                if current_state == DroneOperationalState.GPS_ML_ACTIVE:
                    continue
                if current_state == DroneOperationalState.ACOUSTIC_TRACKING:
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
                self._record_critical_event(
                    drone_id=drone_id,
                    event_type="MISSION_COMPLETE",
                    payload={"target": {"x": target.x, "y": target.y, "z": target.z}},
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

            target = (
                self._mission_targets.get(drone.drone_id)
                or self._manual_goal_targets.get(drone.drone_id)
                or drone.target_position
            )
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
                if goal is None:
                    goal = self._manual_goal_targets.get(drone.drone_id)
                if (
                    goal is not None
                    and drone.flight_mode == FlightMode.HOVER
                    and drone.current_position.distance_to(goal) > 6.0
                ):
                    drone.goto(Position(goal.x, goal.y, goal.z))
                elif goal is not None and drone.current_position.distance_to(goal) <= 6.0:
                    self._manual_goal_targets.pop(drone.drone_id, None)
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
                self._latest_collision_avoidance_path_m = float(
                    math.sqrt(
                        (safe_target.x - drone.current_position.x) ** 2
                        + (safe_target.y - drone.current_position.y) ** 2
                        + (safe_target.z - drone.current_position.z) ** 2
                    )
                )
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
                self._record_critical_event(
                    drone_id=drone.drone_id,
                    event_type="ML_AVOIDANCE_EVENT",
                    payload={
                        "collision_probability": collision_prob,
                        "collision_cone_probability": cone_prob,
                        "ml_confidence": ml_confidence,
                        "fallback_mode": self.fallback_local_avoidance_mode,
                    },
                )
                if cone_prob >= 0.7:
                    self._record_critical_event(
                        drone_id=drone.drone_id,
                        event_type="COLLISION_CONE_HIGH_PROBABILITY",
                        payload={"collision_cone_probability": cone_prob},
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
        self._record_critical_event(
            drone_id=drone_id,
            event_type="DRONE_FAILURE",
            payload={"reason": reason},
        )
        
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
            self._last_leader_election_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._last_leader_election_outcome = "NO_DRONES"
            self._last_leader_election_candidates = 0
            self._last_leader_election_score = 0.0
            return
        
        self.election_in_progress = True
        self._last_leader_election_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._last_leader_election_outcome = "IN_PROGRESS"
        self._last_leader_election_candidates = 0
        self._last_leader_election_score = 0.0
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
            self._last_leader_election_outcome = "NO_CANDIDATE"
            self._last_leader_election_candidates = 0
            self._last_leader_election_score = 0.0
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
        self._last_leader_election_outcome = "ELECTED"
        self._last_leader_election_candidates = len(candidates)
        self._last_leader_election_score = float(candidates[new_leader_id])
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
        """Get movement-active drones (excludes idle/RTH/emergency/crashed)."""
        movement_modes = {FlightMode.TAKEOFF, FlightMode.FLYING, FlightMode.HOVER}
        return [
            drone for drone in self.drones.values()
            if drone.is_active and drone.flight_mode in movement_modes
        ]
    
    def get_swarm_status(self) -> dict:
        """Get complete swarm status"""
        drones_status = {}
        for drone_id, drone in self.drones.items():
            status = drone.get_status()
            status["swarm_state"] = self.drone_state_manager.get_state(drone_id).value
            drones_status[drone_id] = status

        ledger_heights = {drone_id: ledger.block_height() for drone_id, ledger in self.flying_ledgers.items()}
        ledger_integrity = all(ledger.integrity_ok() for ledger in self.flying_ledgers.values()) if self.flying_ledgers else True
        block_height = max(ledger_heights.values()) if ledger_heights else 0
        
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
            "ledger": {
                "block_height": int(block_height),
                "sync_state": self._ledger_last_sync_status,
                "integrity_ok": bool(ledger_integrity),
                "per_drone_height": ledger_heights,
            },
            "acoustic": {
                "enabled": bool(self.acoustic_detection_enabled),
                "confidence_threshold": float(self.acoustic_confidence_threshold),
                "latest_source": self._latest_acoustic_source,
                "latest_confidence": float(self._latest_acoustic_confidence),
                "local_only_mode": bool(self._latest_acoustic_local_only),
            },
            "wind": {
                "enabled": bool(self.wind_enabled),
                "speed_mps": float(self.wind_speed_mps),
                "direction_deg": float(self.wind_direction_deg),
                "gust_factor": float(self.wind_gust_factor),
            },
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
            self._record_critical_event(
                drone_id=drone.drone_id,
                event_type="EMERGENCY_LANDING",
                payload={"reason": reason},
            )

    def emergency_land_drone(self, drone_id: int, reason: str = "Personal emergency commanded") -> bool:
        """Emergency land a specific drone only"""
        drone = self.drones.get(drone_id)
        if not drone:
            self.logger.warning(f"Drone {drone_id} not found for personal emergency landing")
            return False
        drone.trigger_personal_emergency(reason)
        self.logger.error(f"Personal emergency landing triggered for Drone {drone_id}: {reason}")
        self._record_critical_event(
            drone_id=drone_id,
            event_type="EMERGENCY_LANDING",
            payload={"reason": reason},
        )
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
        track_mission: bool = True,
    ):
        """Public API: explicit leader command for movement."""
        self.leader_command_handler.issue_move_to_target(
            targets,
            gps_mode_map=gps_mode_map,
            track_mission=track_mission,
        )

    def leader_move_all_to_single_target(
        self,
        target: Position,
        gps_mode_map: Optional[Dict[int, bool]] = None,
    ):
        """Helper: move all active drones to the same target Y."""
        targets = {drone_id: Position(target.x, target.y, target.z) for drone_id in self.drones.keys()}
        self.leader_move_to_target(targets, gps_mode_map=gps_mode_map)

    def clear_mission_targets_for_drones(self, drone_ids):
        """Remove mission-tracked targets for specific drones."""
        with self._lock:
            for drone_id in (drone_ids or []):
                try:
                    parsed = int(drone_id)
                except Exception:
                    continue
                self._mission_targets.pop(parsed, None)
            if not self._mission_targets:
                self._mission_active = False

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
