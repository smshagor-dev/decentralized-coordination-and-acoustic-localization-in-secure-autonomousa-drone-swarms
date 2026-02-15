import unittest

from drone import Drone, FlightMode, Position
from swarm_manager import SwarmManager


class DynamicFeatureTests(unittest.TestCase):
    def setUp(self):
        self.swarm = SwarmManager()
        self.swarm.dynamic_collision_threshold = 0.05
        self.drone = Drone(1, Position(0, 0, 0))
        self.drone.is_armed = True
        self.drone.flight_mode = FlightMode.HOVER
        self.swarm.add_drone(self.drone)

    def tearDown(self):
        self.swarm.stop()
        for drone in list(self.swarm.drones.values()):
            drone.stop()

    def test_single_moving_obstacle_crossing_path(self):
        self.drone.goto(Position(300, 0, 30))
        self.drone.velocity = Position(12.0, 0.0, 0.0)
        self.swarm.add_dynamic_obstacle(x=25, y=0, vx=0, vy=4, motion_type="linear", radius=20)
        self.swarm._apply_dynamic_obstacle_avoidance()
        self.assertIn(self.drone.drone_id, self.swarm._avoidance_active_ids)

    def test_static_front_obstacle_with_low_current_velocity_still_triggers_avoidance(self):
        self.drone.goto(Position(280, 0, 30))
        self.drone.velocity = Position(0.0, 0.0, 0.0)  # startup/transient case
        self.swarm.add_static_obstacle(x=35, y=0, radius=18)
        self.swarm._apply_dynamic_obstacle_avoidance()
        self.assertIn(self.drone.drone_id, self.swarm._avoidance_active_ids)

    def test_resume_mission_target_after_avoidance(self):
        goal = Position(300, 0, 30)
        self.swarm._mission_targets[self.drone.drone_id] = goal
        self.drone.flight_mode = FlightMode.HOVER
        self.drone.target_position = None
        self.drone.current_position = Position(40, 0, 12)
        self.drone.velocity = Position(0.0, 0.0, 0.0)
        self.swarm.add_static_obstacle(x=60, y=0, radius=16)
        self.swarm._apply_dynamic_obstacle_avoidance()
        self.assertIsNotNone(self.drone.target_position)

    def test_two_dynamic_obstacles(self):
        self.drone.goto(Position(300, 0, 30))
        self.drone.velocity = Position(11.0, 0.5, 0.0)
        self.swarm.add_dynamic_obstacle(x=22, y=-4, vx=0, vy=6, motion_type="linear", radius=18)
        self.swarm.add_dynamic_obstacle(x=28, y=5, vx=-3, vy=-1, motion_type="random_walk", radius=16)
        self.swarm._apply_dynamic_obstacle_avoidance()
        self.assertIn(self.drone.drone_id, self.swarm._avoidance_active_ids)

    def test_high_latency_spike(self):
        stats = self.swarm.simulate_latency_spike(520.0)
        self.assertGreater(stats["total_round_trip_ms"], stats["threshold_ms"])
        self.assertIn("total_round_trip_jitter_std_ms", stats)

    def test_latency_jitter_std_tracking(self):
        self.swarm.ml_bridge.round_trip(py_processing_seconds=0.002, net_one_way_seconds=0.001)
        self.swarm.ml_bridge.round_trip(py_processing_seconds=0.006, net_one_way_seconds=0.003)
        self.swarm.ml_bridge.round_trip(py_processing_seconds=0.004, net_one_way_seconds=0.002)
        stats = self.swarm.latency_monitor.get_stats()
        self.assertIn("total_round_trip_jitter_std_ms", stats)
        self.assertGreaterEqual(stats["total_round_trip_jitter_std_ms"], 0.0)

    def test_collision_cone_and_ml_confidence_available(self):
        self.drone.velocity = Position(10.0, 0.0, 0.0)
        self.swarm.add_dynamic_obstacle(x=20, y=1, vx=-1, vy=0, motion_type="linear", radius=15)
        obstacles = self.swarm.obstacle_manager.get_obstacles()
        result = self.swarm.dynamic_predictor.predict_for_drone(
            (self.drone.current_position.x, self.drone.current_position.y, self.drone.current_position.z),
            (self.drone.velocity.x, self.drone.velocity.y, self.drone.velocity.z),
            obstacles,
            self.swarm.trajectory_estimator,
        )
        self.assertIn("collision_cone_probability", result)
        self.assertIn("ml_confidence", result)
        self.assertGreaterEqual(result["ml_confidence"], 0.0)

    def test_mlbridge_watchdog_timeout_signal(self):
        self.swarm.ml_bridge.last_response_time -= (self.swarm.ml_bridge.watchdog_timeout_s + 0.2)
        self.assertTrue(self.swarm.ml_bridge.is_watchdog_timed_out())

    def test_ml_disabled_fallback_and_return_home(self):
        self.swarm.set_use_personal_ml_avoidance(False)
        self.swarm.set_personal_ml_enabled_all(False)
        self.drone.goto(Position(250, 0, 20))
        self.swarm.add_dynamic_obstacle(x=140, y=0, vx=0, vy=0, motion_type="linear", radius=20)
        self.swarm._apply_dynamic_obstacle_avoidance()
        self.assertIn(self.drone.drone_id, self.swarm._avoidance_active_ids)
        self.swarm._update_adaptive_latency_thresholds()
        status = self.swarm.get_swarm_status()
        self.assertIn(self.drone.drone_id, status["per_drone_latency_threshold_ms"])
        self.drone.return_to_home("test")
        self.assertEqual(self.drone.flight_mode, FlightMode.RETURNING_HOME)


if __name__ == "__main__":
    unittest.main()
