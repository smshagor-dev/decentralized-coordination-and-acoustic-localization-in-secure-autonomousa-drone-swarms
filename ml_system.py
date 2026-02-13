"""
Machine Learning Module - Decision support for autonomous operations
Includes obstacle avoidance, path optimization, and formation maintenance
"""

import numpy as np
import logging
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import math
import os
import time
import csv
import json
from collections import deque

@dataclass
class Obstacle:
    """Obstacle representation"""
    x: float
    y: float
    z: float
    radius: float
    
    def distance_to_point(self, px: float, py: float, pz: float) -> float:
        """Calculate distance from obstacle center to point"""
        return math.sqrt((px - self.x)**2 + (py - self.y)**2 + (pz - self.z)**2)
    
    def is_collision(self, px: float, py: float, pz: float, safety_margin: float = 1.0) -> bool:
        """Check if point collides with obstacle"""
        return self.distance_to_point(px, py, pz) < (self.radius + safety_margin)

class MLDecisionSupport:
    """
    Machine Learning based decision support system
    Provides autonomous decision-making assistance
    """
    
    def __init__(self, owner_id: Optional[int] = None):
        self.owner_id = owner_id
        logger_name = (
            f"MLDecisionSupport_Drone{owner_id}"
            if owner_id is not None else "MLDecisionSupport"
        )
        self.logger = logging.getLogger(logger_name)
        self.setup_logging()
        
        # Obstacle database
        self.obstacles: List[Obstacle] = []
        
        # Learning parameters
        self.collision_weight = 10.0
        self.energy_weight = 1.0
        self.formation_weight = 2.0
        
        self.logger.info("ML Decision Support initialized")
    
    def setup_logging(self):
        """Configure logging"""
        if self.logger.handlers:
            return
        os.makedirs("logs", exist_ok=True)
        handler = logging.FileHandler('logs/ml_system.log')
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
    
    def add_obstacle(self, x: float, y: float, z: float, radius: float):
        """Add obstacle to environment"""
        obstacle = Obstacle(x, y, z, radius)
        self.obstacles.append(obstacle)
        self.logger.info(f"Obstacle added at ({x}, {y}, {z}) with radius {radius}")
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.logger.info("All obstacles cleared")
    
    def check_path_collision(self, start: Tuple[float, float, float], 
                            end: Tuple[float, float, float],
                            safety_margin: float = 2.0) -> bool:
        """
        Check if path from start to end collides with any obstacle
        Uses simple line-sphere intersection
        """
        sx, sy, sz = start
        ex, ey, ez = end
        
        # Direction vector
        dx = ex - sx
        dy = ey - sy
        dz = ez - sz
        length = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if length < 0.001:
            return False
        
        # Normalize
        dx /= length
        dy /= length
        dz /= length
        
        # Check each obstacle
        for obstacle in self.obstacles:
            # Vector from start to obstacle center
            fx = obstacle.x - sx
            fy = obstacle.y - sy
            fz = obstacle.z - sz
            
            # Project onto path direction
            t = max(0, min(length, dx * fx + dy * fy + dz * fz))
            
            # Closest point on path
            closest_x = sx + t * dx
            closest_y = sy + t * dy
            closest_z = sz + t * dz
            
            # Check collision
            if obstacle.is_collision(closest_x, closest_y, closest_z, safety_margin):
                return True
        
        return False
    
    def find_safe_path(self, start: Tuple[float, float, float],
                      end: Tuple[float, float, float],
                      max_iterations: int = 50) -> Optional[List[Tuple[float, float, float]]]:
        """
        Find safe path avoiding obstacles using simple potential field method
        Returns list of waypoints or None if no path found
        """
        if not self.check_path_collision(start, end):
            return [start, end]  # Direct path is safe
        
        # Generate intermediate waypoints using potential field
        waypoints = [start]
        current = np.array(start)
        target = np.array(end)
        
        step_size = 2.0
        
        for _ in range(max_iterations):
            if np.linalg.norm(current - target) < step_size:
                waypoints.append(tuple(target))
                return waypoints
            
            # Attractive force toward target
            direction = target - current
            distance = np.linalg.norm(direction)
            if distance > 0:
                attractive = direction / distance
            else:
                break
            
            # Repulsive forces from obstacles
            repulsive = np.zeros(3)
            for obstacle in self.obstacles:
                obs_pos = np.array([obstacle.x, obstacle.y, obstacle.z])
                diff = current - obs_pos
                dist = np.linalg.norm(diff)
                
                influence_radius = obstacle.radius + 10.0
                if dist < influence_radius and dist > 0.001:
                    # Stronger repulsion closer to obstacle
                    strength = (influence_radius - dist) / influence_radius
                    repulsive += (diff / dist) * strength * 2.0
            
            # Combined force
            total_force = attractive + repulsive
            norm = np.linalg.norm(total_force)
            if norm > 0:
                total_force = total_force / norm
            
            # Move in direction of force
            next_pos = current + total_force * step_size
            
            # Check if new position is valid (not inside obstacle)
            valid = True
            for obstacle in self.obstacles:
                if obstacle.is_collision(next_pos[0], next_pos[1], next_pos[2], 1.0):
                    valid = False
                    break
            
            if valid:
                current = next_pos
                waypoints.append(tuple(current))
            else:
                # Try alternative direction
                perpendicular = np.array([-total_force[1], total_force[0], 0])
                norm_perp = np.linalg.norm(perpendicular)
                if norm_perp > 0:
                    perpendicular = perpendicular / norm_perp
                    current = current + perpendicular * step_size
                    waypoints.append(tuple(current))
        
        self.logger.warning("Could not find safe path within iteration limit")
        return None
    
    def optimize_formation_position(self, drone_pos: Tuple[float, float, float],
                                   leader_pos: Tuple[float, float, float],
                                   desired_offset: Tuple[float, float, float],
                                   other_drones: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """
        Optimize position for formation flying
        Considers leader position, desired offset, and other drones
        """
        dx, dy, dz = desired_offset
        
        # Ideal position relative to leader
        ideal_x = leader_pos[0] + dx
        ideal_y = leader_pos[1] + dy
        ideal_z = leader_pos[2] + dz
        
        # Adjust for other drones (maintain spacing)
        min_spacing = 5.0
        adjustment = np.zeros(3)
        
        for other_pos in other_drones:
            diff = np.array([
                ideal_x - other_pos[0],
                ideal_y - other_pos[1],
                ideal_z - other_pos[2]
            ])
            distance = np.linalg.norm(diff)
            
            if distance < min_spacing and distance > 0:
                # Push away from other drone
                adjustment += (diff / distance) * (min_spacing - distance) * 0.5
        
        # Apply adjustment
        final_x = ideal_x + adjustment[0]
        final_y = ideal_y + adjustment[1]
        final_z = ideal_z + adjustment[2]
        
        return (final_x, final_y, final_z)
    
    def predict_collision_risk(self, position: Tuple[float, float, float],
                              velocity: Tuple[float, float, float],
                              time_horizon: float = 5.0) -> float:
        """
        Predict collision risk based on current trajectory
        Returns risk score 0-1 (0=safe, 1=imminent collision)
        """
        if not self.obstacles:
            return 0.0
        
        px, py, pz = position
        vx, vy, vz = velocity
        
        # Project future positions
        max_risk = 0.0
        steps = 10
        
        for i in range(1, steps + 1):
            t = (time_horizon / steps) * i
            future_x = px + vx * t
            future_y = py + vy * t
            future_z = pz + vz * t
            
            # Check against obstacles
            for obstacle in self.obstacles:
                distance = obstacle.distance_to_point(future_x, future_y, future_z)
                safe_distance = obstacle.radius + 3.0
                
                if distance < safe_distance:
                    risk = 1.0 - (distance / safe_distance)
                    max_risk = max(max_risk, risk)
        
        return max_risk
    
    def suggest_avoidance_maneuver(self, position: Tuple[float, float, float],
                                   velocity: Tuple[float, float, float],
                                   target: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Suggest velocity adjustment to avoid collision while moving toward target
        """
        risk = self.predict_collision_risk(position, velocity)
        
        if risk < 0.3:
            # Low risk - maintain course toward target
            return velocity
        
        # Find safe direction
        px, py, pz = position
        tx, ty, tz = target
        
        best_velocity = velocity
        min_risk = risk
        
        # Test different directions
        speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        
        for angle in np.linspace(0, 2*np.pi, 8):
            for elevation in [-0.3, 0, 0.3]:
                # Test velocity in this direction
                test_vx = speed * math.cos(angle)
                test_vy = speed * math.sin(angle)
                test_vz = speed * elevation
                
                test_risk = self.predict_collision_risk(position, (test_vx, test_vy, test_vz))
                
                # Also consider if this moves us toward target
                future_pos = (px + test_vx, py + test_vy, pz + test_vz)
                current_dist = math.sqrt((tx-px)**2 + (ty-py)**2 + (tz-pz)**2)
                future_dist = math.sqrt((tx-future_pos[0])**2 + (ty-future_pos[1])**2 + (tz-future_pos[2])**2)
                
                progress = current_dist - future_dist
                
                # Score combines low risk and progress toward target
                score = -test_risk + progress * 0.1
                
                if test_risk < min_risk:
                    min_risk = test_risk
                    best_velocity = (test_vx, test_vy, test_vz)
        
        return best_velocity
    
    def evaluate_landing_site(self, site: Tuple[float, float], 
                            drone_position: Tuple[float, float, float]) -> float:
        """
        Evaluate landing site safety
        Returns score 0-1 (higher is better)
        """
        sx, sy = site
        
        # Check if site is too close to obstacles
        for obstacle in self.obstacles:
            if obstacle.z < 1.0:  # Ground level obstacle
                distance = math.sqrt((sx - obstacle.x)**2 + (sy - obstacle.y)**2)
                if distance < obstacle.radius + 5.0:
                    return 0.0  # Unsafe
        
        # Prefer closer sites
        dx, dy, dz = drone_position
        distance_to_site = math.sqrt((sx - dx)**2 + (sy - dy)**2)
        distance_score = 1.0 / (1.0 + distance_to_site / 50.0)
        
        return distance_score


class PhysicalMLTrainer:
    """
    Advanced personal trainer for physical drone telemetry.
    Supports online ingestion plus CSV/JSON dataset import/export.
    Learns a regularized regression model with optional polynomial features.
    """

    def __init__(self, owner_id: Optional[int] = None, model_dir: str = "models"):
        self.owner_id = owner_id
        self.model_dir = model_dir
        os.makedirs(self.model_dir, exist_ok=True)

        logger_name = (
            f"PhysicalMLTrainer_Drone{owner_id}"
            if owner_id is not None else "PhysicalMLTrainer"
        )
        self.logger = logging.getLogger(logger_name)
        if not self.logger.handlers:
            os.makedirs("logs", exist_ok=True)
            handler = logging.FileHandler('logs/ml_system.log')
            handler.setFormatter(logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            ))
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)

        self.samples_x = deque(maxlen=10000)
        self.samples_y = deque(maxlen=10000)

        # Default telemetry schema used by drone.py sample collector.
        self.feature_names = [
            "battery_level",
            "signal_strength",
            "processing_capability",
            "altitude",
            "velocity_x",
            "velocity_y",
            "velocity_z",
        ]
        self.target_names = ["target_vx", "target_vy", "target_vz"]

        # Model params/state
        self.weights = None
        self.feature_mean = None
        self.feature_std = None
        self.poly_degree = 1
        self.regularization_alpha = 0.0
        self.training_metrics: Dict[str, Any] = {}
        self.trained_at = None

    def ingest_sample(self, sensor_features: List[float], target_controls: List[float]):
        """Add one physical sample (from real drone telemetry)."""
        if not sensor_features or not target_controls:
            return
        self.samples_x.append(np.array(sensor_features, dtype=float))
        self.samples_y.append(np.array(target_controls, dtype=float))

    def _poly_expand(self, x: np.ndarray, degree: int) -> np.ndarray:
        """Expand features up to polynomial degree 2 for richer model capacity."""
        if degree <= 1:
            return x
        features = [x]
        # Quadratic terms.
        features.append(x ** 2)
        # Pairwise interactions.
        interactions = []
        for i in range(x.shape[1]):
            for j in range(i + 1, x.shape[1]):
                interactions.append((x[:, i] * x[:, j]).reshape(-1, 1))
        if interactions:
            features.append(np.hstack(interactions))
        return np.hstack(features)

    def _prepare_inputs(self, x: np.ndarray, fit_stats: bool = False) -> np.ndarray:
        """Normalize and expand features."""
        if fit_stats or self.feature_mean is None or self.feature_std is None:
            self.feature_mean = x.mean(axis=0)
            self.feature_std = x.std(axis=0)
            self.feature_std[self.feature_std < 1e-9] = 1.0
        x_norm = (x - self.feature_mean) / self.feature_std
        return self._poly_expand(x_norm, self.poly_degree)

    def _fit_ridge(self, x: np.ndarray, y: np.ndarray, alpha: float) -> np.ndarray:
        """
        Fit ridge regression in closed form:
        W = (X'X + alpha*I)^-1 X'Y
        """
        ones = np.ones((x.shape[0], 1))
        x_aug = np.hstack([x, ones])
        cols = x_aug.shape[1]
        reg = np.eye(cols) * alpha
        reg[-1, -1] = 0.0  # do not regularize bias
        lhs = x_aug.T @ x_aug + reg
        rhs = x_aug.T @ y
        try:
            return np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            return np.linalg.pinv(lhs) @ rhs

    def _predict_matrix(self, x: np.ndarray) -> np.ndarray:
        if self.weights is None:
            return np.array([])
        x_proc = self._prepare_inputs(x, fit_stats=False)
        ones = np.ones((x_proc.shape[0], 1))
        x_aug = np.hstack([x_proc, ones])
        return x_aug @ self.weights

    def train(self, min_samples: int = 50, poly_degree: int = 2) -> bool:
        """Train model with holdout validation and alpha search."""
        if len(self.samples_x) < min_samples:
            self.logger.warning(
                f"Not enough physical samples to train: {len(self.samples_x)}/{min_samples}"
            )
            return False

        x = np.vstack(self.samples_x)
        y = np.vstack(self.samples_y)
        self.poly_degree = 2 if poly_degree >= 2 else 1

        # Deterministic split for repeatability.
        n = x.shape[0]
        split_idx = max(1, int(n * 0.8))
        x_train, x_val = x[:split_idx], x[split_idx:]
        y_train, y_val = y[:split_idx], y[split_idx:]
        if x_val.shape[0] == 0:
            x_val, y_val = x_train, y_train

        # Fit normalization on train set only.
        x_train_proc = self._prepare_inputs(x_train, fit_stats=True)
        x_val_proc = self._prepare_inputs(x_val, fit_stats=False)

        alphas = [0.0, 1e-4, 1e-3, 1e-2, 1e-1, 1.0]
        best_alpha = 0.0
        best_weights = None
        best_val_mse = float("inf")

        for alpha in alphas:
            weights = self._fit_ridge(x_train_proc, y_train, alpha)
            ones = np.ones((x_val_proc.shape[0], 1))
            val_pred = np.hstack([x_val_proc, ones]) @ weights
            val_mse = float(np.mean((val_pred - y_val) ** 2))
            if val_mse < best_val_mse:
                best_val_mse = val_mse
                best_alpha = alpha
                best_weights = weights

        self.weights = best_weights
        self.regularization_alpha = best_alpha

        # Metrics snapshot.
        train_pred = self._predict_matrix(x_train)
        val_pred = self._predict_matrix(x_val)
        train_mse = float(np.mean((train_pred - y_train) ** 2))
        val_mse = float(np.mean((val_pred - y_val) ** 2))
        y_centered = y_val - y_val.mean(axis=0, keepdims=True)
        baseline = float(np.mean(y_centered ** 2) + 1e-9)
        r2_like = max(0.0, 1.0 - (val_mse / baseline))
        self.training_metrics = {
            "samples": int(n),
            "train_samples": int(x_train.shape[0]),
            "val_samples": int(x_val.shape[0]),
            "poly_degree": int(self.poly_degree),
            "alpha": float(self.regularization_alpha),
            "train_mse": train_mse,
            "val_mse": val_mse,
            "val_r2_like": float(r2_like),
            "trained_at": time.time(),
        }
        self.trained_at = time.time()
        self.save_model()
        self.logger.info(
            "Physical ML model trained "
            f"drone={self.owner_id} samples={n} degree={self.poly_degree} "
            f"alpha={best_alpha} val_mse={best_val_mse:.6f}"
        )
        return True

    def predict(self, sensor_features: List[float]) -> Optional[np.ndarray]:
        """Predict control outputs from sensor features."""
        if self.weights is None:
            return None
        x = np.array(sensor_features, dtype=float).reshape(1, -1)
        pred = self._predict_matrix(x)
        if pred.size == 0:
            return None
        return pred[0]

    def export_dataset(self, output_path: str, file_format: str = "csv") -> bool:
        """Export in-memory samples to CSV or JSON."""
        if not self.samples_x:
            return False
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        x = np.vstack(self.samples_x)
        y = np.vstack(self.samples_y)
        file_format = (file_format or "csv").strip().lower()

        if file_format == "json":
            records = []
            for i in range(x.shape[0]):
                rec = {name: float(x[i, idx]) for idx, name in enumerate(self.feature_names)}
                rec.update({name: float(y[i, idx]) for idx, name in enumerate(self.target_names)})
                records.append(rec)
            with open(output_path, "w", encoding="utf-8") as fh:
                json.dump(records, fh, indent=2)
            return True

        # CSV default
        headers = self.feature_names + self.target_names
        with open(output_path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(headers)
            for i in range(x.shape[0]):
                writer.writerow(list(x[i]) + list(y[i]))
        return True

    def import_dataset(self, input_path: str, append: bool = False) -> int:
        """
        Import dataset from CSV/JSON.
        Returns number of valid samples imported.
        """
        if not os.path.exists(input_path):
            return 0
        if not append:
            self.samples_x.clear()
            self.samples_y.clear()

        ext = os.path.splitext(input_path)[1].lower()
        imported = 0

        def _read_record(record: Dict[str, Any]) -> bool:
            nonlocal imported
            try:
                feats = [float(record[name]) for name in self.feature_names]
                targs = [float(record[name]) for name in self.target_names]
            except Exception:
                return False
            self.ingest_sample(feats, targs)
            imported += 1
            return True

        if ext == ".json":
            with open(input_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, list):
                for item in data:
                    if isinstance(item, dict):
                        _read_record(item)
            return imported

        # CSV default
        with open(input_path, "r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                _read_record(row)
        return imported

    def train_from_dataset(
        self,
        input_path: str,
        min_samples: int = 50,
        poly_degree: int = 2,
        append: bool = False,
    ) -> bool:
        """Load a CSV/JSON dataset and train model from it."""
        imported = self.import_dataset(input_path, append=append)
        if imported == 0:
            self.logger.warning(f"No usable dataset rows in {input_path}")
            return False
        return self.train(min_samples=min_samples, poly_degree=poly_degree)

    @classmethod
    def generate_demo_dataset(
        cls, output_path: str, samples: int = 500, file_format: str = "csv"
    ) -> bool:
        """Create a synthetic telemetry dataset for quick personal training."""
        trainer = cls(owner_id=None)
        rng = np.random.default_rng(42)
        for _ in range(max(10, samples)):
            battery = float(rng.uniform(20.0, 100.0))
            signal = float(rng.uniform(40.0, 100.0))
            proc = float(rng.uniform(50.0, 100.0))
            alt = float(rng.uniform(0.0, 120.0))
            vx = float(rng.normal(0.0, 3.5))
            vy = float(rng.normal(0.0, 3.5))
            vz = float(rng.normal(0.0, 1.0))

            # Synthetic target with simple nonlinear dynamics.
            safety_factor = min(1.0, signal / 100.0) * min(1.0, battery / 100.0)
            tx = vx * (0.85 + 0.1 * safety_factor) - 0.0008 * alt * vx
            ty = vy * (0.85 + 0.1 * safety_factor) - 0.0008 * alt * vy
            tz = vz * 0.7 - (0.002 if battery < 30 else 0.0)
            trainer.ingest_sample(
                [battery, signal, proc, alt, vx, vy, vz],
                [tx, ty, tz],
            )
        return trainer.export_dataset(output_path, file_format=file_format)

    def model_path(self) -> str:
        suffix = f"drone_{self.owner_id}" if self.owner_id is not None else "global"
        return os.path.join(self.model_dir, f"physical_ml_{suffix}.npz")

    def save_model(self):
        """Persist trained model to disk."""
        if self.weights is None:
            return
        np.savez(
            self.model_path(),
            weights=self.weights,
            feature_mean=self.feature_mean,
            feature_std=self.feature_std,
            poly_degree=np.array([self.poly_degree], dtype=int),
            alpha=np.array([self.regularization_alpha], dtype=float),
            owner_id=-1 if self.owner_id is None else self.owner_id,
            sample_count=len(self.samples_x),
            training_metrics=np.array([json.dumps(self.training_metrics)], dtype=object),
        )

    def load_model(self) -> bool:
        """Load model from disk if available."""
        path = self.model_path()
        if not os.path.exists(path):
            return False
        data = np.load(path, allow_pickle=True)
        self.weights = data["weights"]
        if "feature_mean" in data and "feature_std" in data:
            self.feature_mean = data["feature_mean"]
            self.feature_std = data["feature_std"]
        if "poly_degree" in data:
            self.poly_degree = int(data["poly_degree"][0])
        if "alpha" in data:
            self.regularization_alpha = float(data["alpha"][0])
        if "training_metrics" in data:
            try:
                self.training_metrics = json.loads(str(data["training_metrics"][0]))
            except Exception:
                self.training_metrics = {}
        return True


class FormationController:
    """
    Controls drone formation using ML-based optimization
    """
    
    def __init__(self):
        self.logger = logging.getLogger("FormationController")
        self.formations = {
            "line": self._line_formation,
            "v": self._v_formation,
            "circle": self._circle_formation,
            "grid": self._grid_formation
        }
    
    def _line_formation(self, leader_pos: Tuple[float, float, float], 
                       num_followers: int) -> List[Tuple[float, float, float]]:
        """Calculate line formation positions"""
        positions = []
        spacing = 10.0

        slots = []
        for rank in range(1, num_followers + 1):
            slots.append(rank)
            slots.append(-rank)
        slots = slots[:num_followers]

        for i in range(num_followers):
            pos = (leader_pos[0], leader_pos[1] + slots[i] * spacing, leader_pos[2])
            positions.append(pos)
        
        return positions
    
    def _v_formation(self, leader_pos: Tuple[float, float, float],
                    num_followers: int) -> List[Tuple[float, float, float]]:
        """Calculate V formation positions"""
        positions = []
        spacing = 10.0
        angle = math.pi / 6  # 30 degrees
        
        for i in range(num_followers):
            side = 1 if i % 2 == 0 else -1
            row = (i // 2) + 1
            offset_x = -row * spacing * math.cos(angle)
            offset_y = side * row * spacing * math.sin(angle)
            
            pos = (
                leader_pos[0] + offset_x,
                leader_pos[1] + offset_y,
                leader_pos[2]
            )
            positions.append(pos)
        
        return positions
    
    def _circle_formation(self, leader_pos: Tuple[float, float, float],
                         num_followers: int) -> List[Tuple[float, float, float]]:
        """Calculate circle formation positions"""
        positions = []
        radius = 15.0
        angle_step = 2 * math.pi / num_followers
        
        for i in range(num_followers):
            angle = i * angle_step
            pos = (
                leader_pos[0] + radius * math.cos(angle),
                leader_pos[1] + radius * math.sin(angle),
                leader_pos[2]
            )
            positions.append(pos)
        
        return positions
    
    def _grid_formation(self, leader_pos: Tuple[float, float, float],
                       num_followers: int) -> List[Tuple[float, float, float]]:
        """Calculate grid formation positions"""
        positions = []
        spacing = 10.0
        side = int(math.ceil(math.sqrt(num_followers + 1)))
        if side % 2 == 0:
            side += 1
        half = side // 2

        slots = []
        for row in range(-half, half + 1):
            for col in range(-half, half + 1):
                if row == 0 and col == 0:
                    continue
                slots.append((row, col))
        slots.sort(key=lambda rc: (max(abs(rc[0]), abs(rc[1])), abs(rc[0]) + abs(rc[1]), rc[0], rc[1]))

        for i in range(num_followers):
            row, col = slots[i]
            pos = (
                leader_pos[0] + row * spacing,
                leader_pos[1] + col * spacing,
                leader_pos[2]
            )
            positions.append(pos)
        
        return positions
    
    def get_formation_positions(self, formation_type: str,
                               leader_pos: Tuple[float, float, float],
                               num_followers: int) -> List[Tuple[float, float, float]]:
        """Get target positions for formation"""
        if formation_type in self.formations:
            return self.formations[formation_type](leader_pos, num_followers)
        else:
            return self._line_formation(leader_pos, num_followers)
