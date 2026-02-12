"""
Machine Learning Module - Decision support for autonomous operations
Includes obstacle avoidance, path optimization, and formation maintenance
"""

import numpy as np
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass
import math
import os
import time
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
    Lightweight online trainer for physical drone telemetry.
    Learns a linear mapping from sensor features -> control target.
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
            handler = logging.FileHandler('logs/ml_system.log')
            handler.setFormatter(logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            ))
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)

        self.samples_x = deque(maxlen=10000)
        self.samples_y = deque(maxlen=10000)
        self.weights = None
        self.trained_at = None

    def ingest_sample(self, sensor_features: List[float], target_controls: List[float]):
        """Add one physical sample (from real drone telemetry)."""
        if not sensor_features or not target_controls:
            return
        self.samples_x.append(np.array(sensor_features, dtype=float))
        self.samples_y.append(np.array(target_controls, dtype=float))

    def train(self, min_samples: int = 50) -> bool:
        """Train linear model with least squares from accumulated physical data."""
        if len(self.samples_x) < min_samples:
            self.logger.warning(
                f"Not enough physical samples to train: {len(self.samples_x)}/{min_samples}"
            )
            return False

        x = np.vstack(self.samples_x)
        y = np.vstack(self.samples_y)

        # Add bias term for linear regression.
        ones = np.ones((x.shape[0], 1))
        x_aug = np.hstack([x, ones])
        self.weights, _, _, _ = np.linalg.lstsq(x_aug, y, rcond=None)
        self.trained_at = time.time()
        self.save_model()
        self.logger.info(
            f"Physical ML model trained for drone={self.owner_id} samples={len(self.samples_x)}"
        )
        return True

    def predict(self, sensor_features: List[float]) -> Optional[np.ndarray]:
        """Predict control outputs from sensor features."""
        if self.weights is None:
            return None
        x = np.array(sensor_features, dtype=float)
        x_aug = np.append(x, 1.0)
        return x_aug @ self.weights

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
            owner_id=-1 if self.owner_id is None else self.owner_id,
            sample_count=len(self.samples_x)
        )

    def load_model(self) -> bool:
        """Load model from disk if available."""
        path = self.model_path()
        if not os.path.exists(path):
            return False
        data = np.load(path)
        self.weights = data["weights"]
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
        
        for i in range(num_followers):
            offset = (i + 1) * spacing
            pos = (leader_pos[0] - offset, leader_pos[1], leader_pos[2])
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
        cols = int(math.ceil(math.sqrt(num_followers)))
        
        for i in range(num_followers):
            row = i // cols
            col = i % cols
            
            pos = (
                leader_pos[0] - (row + 1) * spacing,
                leader_pos[1] + (col - cols/2) * spacing,
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
