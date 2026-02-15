# Drone Swarm Management System

Production-ready multi-drone swarm simulation and control stack with:
- event-driven swarm coordination
- personal ML-based dynamic obstacle avoidance
- C++ <-> Python latency monitoring and fallback safety
- encrypted communication logs
- PyQt5 GUI control + visualization

## What Is Implemented

### Core Swarm Features
- Leader election and follower coordination
- Per-drone operational state tracking
- Return-to-home and emergency behaviors
- Motor-failure handling and degraded return logic

### Dynamic Obstacle + Personal ML
- Dynamic obstacle models: `linear`, `circular`, `random_walk`
- Short-horizon trajectory prediction (velocity + optional acceleration)
- Collision probability + collision-cone verification
- ML confidence scoring for avoidance decisions
- Automatic path replanning with smooth steering and acceleration limits
- New state: `AVOIDING_DYNAMIC_OBSTACLE`

### Latency Monitoring and Safety
- Monitors:
  - `T_cpp_to_py`
  - `T_py_processing`
  - `T_py_to_cpp`
  - `Total_round_trip`
- Rolling averages + jitter standard deviation
- Watchdog timeout for ML bridge response delays
- Adaptive per-drone latency thresholds
- Automatic fallback to local geometric avoidance when latency is unsafe

### GUI Improvements
- Add moving obstacle from GUI controls
- Obstacle motion visualization (direction, pulse, trail)
- Toggle static/dynamic obstacle visibility
- Toggle personal ML avoidance
- Latency panel (including RTT jitter)
- Swarm status table is always vertically scrollable

### Auto Dynamic Obstacle Field
- On startup (`python main.py`), system auto-generates **20-30 dynamic obstacles** in the operation frame.
- Manual obstacle add is still available from GUI.

## Actual Project Structure

```text
secure-drone-swarm/
├── main.py
├── gui.py
├── swarm_manager.py
├── drone.py
├── leader_follower_logic.py
├── dynamic_obstacles.py
├── latency_monitor.py
├── ml_system.py
├── ml_trainer.py
├── communication.py
├── test_dynamic_features.py
├── dronecontroller.h
├── dronecontroller.cpp
├── main_test.cpp
├── cmakelists.txt
├── requirements.txt
├── readme.md
├── assets/
│   ├── drone.svg
│   └── fields.svg
├── config/
│   └── swarm_config.json
├── datasets/
│   ├── personal_training.csv
│   ├── personal_training.json
│   └── personal_drone_1.csv
├── models/
├── logs/
└── build/
```

## Requirements

- Python 3.10+ (recommended)
- Windows/Linux/macOS
- Optional C++ toolchain + CMake (for native controller builds)

Install Python dependencies:

```bash
python -m venv venv
# Windows
venv\Scripts\activate
# Linux/macOS
# source venv/bin/activate

pip install -r requirements.txt
```

## Run Instructions

### 1. Start Full System (GUI)

```bash
python main.py
```

What happens on start:
- demo swarm is created
- swarm monitor/event loop starts
- GUI launches
- 20-30 dynamic obstacles are auto-populated

### 2. Run Tests (Single Entry Point)

```bash
python main.py --test
```

This runs all `test*.py` tests (including dynamic obstacle + latency tests).

### 3. Run Specific Test File (optional)

```bash
python -m unittest test_dynamic_features.py
```

## GUI Usage Quick Guide

1. Start with `python main.py`
2. In controls panel:
- Arm/Takeoff drones
- Send movement commands (`Leader Command X->Y` or manual move)
- Enable `Use Personal ML Avoidance`
- Add static/dynamic obstacles if needed
3. In `Swarm Status` panel:
- View ID/Role/Mode/Battery/Altitude/Status table
- Scroll vertically to see all drones
4. In logs panel:
- Monitor avoidance events, confidence, and latency warnings

## Environment Variables (Optional)

- `REAL_DRONE_ENABLED=1`
- `REAL_DRONE_CONNECTIONS=1=udpin://:14540,2=udpin://:14541`
- `REAL_GPS_REF_LAT=<value>`
- `REAL_GPS_REF_LON=<value>`

If these are unset, simulation mode is used.

## C++ Notes

C++ controller files are present:
- `dronecontroller.h`
- `dronecontroller.cpp`

Latency monitor types are also implemented on the C++ side.

If you use CMake, verify `cmakelists.txt` source paths match your local layout before building.

## Logs

Runtime logs are written under `logs/`, including:
- swarm manager
- per-drone logs
- ML system logs
- encrypted communication logs

## Troubleshooting

### Drone not avoiding obstacle
- Ensure drone is moving toward a target (`MOVE` / `goto` commanded)
- Keep `Use Personal ML Avoidance` checked
- Confirm obstacle is inside operation area and visible
- Check logs for `DYNAMIC_OBSTACLE_AVOIDANCE` events

### Latency fallback triggers too often
- Watch `RTT` and `RTT Jitter` in GUI
- Increase stability by reducing injected spikes/tests
- Review per-drone processing capability and current speed
