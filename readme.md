# Secure Drone Swarm System

End-to-end multi-drone swarm simulation/control platform with Python + C++ modules, event-driven orchestration, personal ML obstacle avoidance, latency-aware fallback safety, and a full-featured PyQt GUI.

## 1. System Summary

This project provides:
- Swarm leadership and follower coordination
- GPS/local mission assignment and execution
- Dynamic obstacle prediction and automatic rerouting
- Collision-risk + collision-cone based avoidance decisions
- C++ <-> Python latency monitoring, jitter analysis, watchdog timeout, and fallback mode
- Encrypted command/message telemetry logging
- GUI-driven operations with real-time visualization and control

## 2. Technology Stack

- Python: swarm logic, drone behavior, ML decision modules, GUI, secure communication
- C++: low-level controller interfaces and latency monitor types
- GUI: PyQt5 (`gui.py`)

Core files:
- `main.py`
- `swarm_manager.py`
- `drone.py`
- `leader_follower_logic.py`
- `dynamic_obstacles.py`
- `latency_monitor.py`
- `communication.py`
- `ml_system.py`
- `gui.py`
- `dronecontroller.h`, `dronecontroller.cpp`

## 3. Current Folder Structure

```text
secure-drone-swarm/
├── main.py
├── gui.py
├── swarm_manager.py
├── drone.py
├── leader_follower_logic.py
├── dynamic_obstacles.py
├── latency_monitor.py
├── communication.py
├── ml_system.py
├── ml_trainer.py
├── dronecontroller.h
├── dronecontroller.cpp
├── main_test.cpp
├── test_dynamic_features.py
├── requirements.txt
├── readme.md
├── config/
│   └── swarm_config.json
├── assets/
│   ├── drone.svg
│   └── fields.svg
├── datasets/
│   ├── personal_training.csv
│   ├── personal_training.json
│   └── personal_drone_1.csv
├── models/
├── logs/
└── build/
```

## 4. Full GUI Feature Description

`gui.py` contains the complete operator console. Major parts:

### 4.1 Visualization Tabs
- **Drone Visual**
  - Drone icons, labels, trails, heading markers
  - Home markers (`H<id>`)
  - Operation box/corners and X->Y plan overlay
  - Static + dynamic obstacle rendering
  - Dynamic obstacle pulse + motion trail + velocity direction line
- **Location Map**
  - 10km radius simplified map view
  - selected-drone highlighting

### 4.2 Controls Panel

#### Fleet Controls
- `Add Drone`
- `Remove Drone`

#### Flight Controls
- `Arm All`
- `Takeoff All`
- `Land All`
- `RTH All`
- `EMERGENCY LAND` (all)
- `EMERGENCY SELECTED` (single drone)
- `Leader Command X->Y`

#### Formation and Follow
- Formation: `line`, `v`, `circle`, `grid`
- Leader follow pattern: `v`, `line`
- Adjustable follow spacing

#### Fault/Stress Scenarios
- `Simulate Motor Failure`
- `Crash Leader`
- `Auto Fault Demo: ON/OFF`
- `Simulate Latency Spike`

#### Dynamic Obstacle Controls
- Add moving obstacle with:
  - start position (`x`, `y`)
  - velocity (`vx`, `vy`)
  - radius
  - motion type (`linear`, `circular`, `random_walk`)
- Add static obstacle
- Clear obstacles
- Toggle static visibility
- Toggle dynamic visibility
- Toggle `Use Personal ML Avoidance`

#### Selected Drone Manual Movement
- Up/Down/Left/Right/Hover buttons
- Keyboard shortcuts:
  - Arrow keys for movement
  - `Space`/`H` for hover

#### GPS Mission Panel (Selected or All)
- Reference GPS (`Ref Lat`, `Ref Lon`)
- Target GPS center (`Target Lat`, `Target Lon`)
- Mission radius
- Drone ID selector (`0 = All`)
- `Assign Mission`
- `Clear Mission`
- `Open Target in Google Maps`

### 4.3 Swarm Status Panel
- Total drones
- Active drones
- Leader ID
- Average battery
- Latency metrics:
  - C++->Py
  - Py processing
  - Py->C++
  - RTT
  - RTT jitter
- Drone table (always vertically scrollable):
  - ID, Role, Mode, Battery, Altitude, Status

### 4.4 Logs Panel
- System log stream
- Encrypted command/message lines
- Encrypted D2D traffic tail
- Avoidance events with probability/cone/confidence
- Latency warning events

## 5. Core Operational Logic

## 5.1 Drone Roles and States

Low-level flight modes (examples):
- `IDLE`, `TAKEOFF`, `HOVER`, `FLYING`, `RETURNING_HOME`, `LANDING`, `EMERGENCY_LANDING`, `CRASHED`

High-level swarm states:
- `IDLE`
- `TAKEOFF`
- `WAITING_FOR_COMMAND`
- `MOVING_TO_TARGET`
- `AVOIDING_DYNAMIC_OBSTACLE`
- `MISSION_COMPLETE`
- `RETURNING_HOME`
- `GPS_ML_ACTIVE`

## 5.2 Leader Election and Command Flow
- Swarm manager elects leader by suitability score
- Leader issues command events:
  - TAKEOFF
  - MOVE_TO_TARGET
  - RETURN_TO_HOME
- Followers execute commands through event bus, not uncontrolled autonomous drift

## 5.3 Mission X->Y and Auto Return
- Team takeoff from X slots
- Leader commands movement to Y slots
- On first mission-complete arrival:
  - encrypted team broadcast sent
  - all drones commanded to return to X/home

## 5.4 GPS Mission Logic
- Per-drone mission can be assigned from GUI
- Drone converts reference GPS <-> local coordinates
- Drone executes in-area loiter when reaching mission zone
- Drone status shows mission active/on_target/transit states

## 5.5 Dynamic Obstacle Prediction and Rerouting
- Obstacle state model includes:
  - position
  - velocity
  - optional acceleration
  - motion type
- Predictor computes:
  - future trajectory (short horizon)
  - collision probability
  - collision-cone probability
  - ML confidence
  - avoidance vector
- Swarm avoidance controller does:
  - smooth `v_new = v_goal + v_avoidance`
  - acceleration-limited steering
  - bypass target generation around blocking obstacle
  - reroute toward safe waypoint, then resume mission destination

## 5.6 Latency Monitoring and Fallback Safety
- ML bridge records simulated/bridged timestamps
- Metrics:
  - `T_cpp_to_py`
  - `T_py_processing`
  - `T_py_to_cpp`
  - `Total_round_trip`
- Rolling average + jitter stddev
- Watchdog timeout check for delayed bridge response
- Adaptive per-drone latency thresholds
- If latency unsafe:
  - fallback local geometric avoidance enabled
  - warning event logged and shown in GUI

## 5.7 Emergency and Fault Handling
- Motor failure detection (degraded return mode)
- Personal emergency return per drone
- Global emergency return/land
- Leader crash handling with re-election
- Battery-driven automatic return/emergency landing

## 5.8 Secure Communication and Encryption
- AES-based encrypted message framework in `communication.py`
- Controller->drone command logs are shown as encrypted payload hex
- Swarm event logs include encrypted command/message records
- Optional encrypted team broadcast on mission events

## 6. Startup Behavior

When running `python main.py`:
- demo swarm (5 drones) is created
- swarm monitoring/event loop starts
- GUI starts
- **20-30 dynamic obstacles are auto-populated** in operation frame

## 7. Installation

### 7.1 Python

```bash
python -m venv venv
# Windows
venv\Scripts\activate
# Linux/macOS
# source venv/bin/activate

pip install -r requirements.txt
```

### 7.2 Optional C++ Build

C++ controller files are present. If building manually, verify your CMake source paths match current repository layout.

## 8. Run Instructions

### 8.1 Run Full System

```bash
python main.py
```

### 8.2 Run Full Test Suite (Recommended)

```bash
python main.py --test
```

### 8.3 Run Specific Test File

```bash
python -m unittest test_dynamic_features.py
```

## 9. Environment Variables (Optional Real-Drone Mode)

- `REAL_DRONE_ENABLED=1`
- `REAL_DRONE_CONNECTIONS=1=udpin://:14540,2=udpin://:14541`
- `REAL_GPS_REF_LAT=<value>`
- `REAL_GPS_REF_LON=<value>`

If these are not set, simulation mode is used.

## 10. Logging and Outputs

Generated in `logs/`:
- per-drone logs
- swarm manager logs
- ML system logs
- encrypted communication logs

Model snapshots/training artifacts can update under `models/`.

## 11. Testing Coverage (Current)

`test_dynamic_features.py` validates:
- single moving obstacle crossing path
- two dynamic obstacles
- static-front obstacle with low current velocity
- reroute and mission-target resume behavior
- high latency spike
- latency jitter metric availability
- collision-cone + confidence output presence
- watchdog timeout signal
- ML-disabled fallback + return-home continuity

## 12. Troubleshooting

### Drone not rerouting
- Ensure drone has active target/destination
- Keep `Use Personal ML Avoidance` enabled
- Verify obstacle is inside active map area
- Check logs for `DYNAMIC_OBSTACLE_AVOIDANCE`

### Drone hovers after avoidance
- Confirm mission target is still active
- Check `SwarmManager` logs for reroute + resume entries

### Frequent fallback mode
- Inspect RTT and RTT jitter in status panel
- Reduce injected latency spikes
- Review per-drone threshold adaptation and processing load
