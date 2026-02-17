<!--
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
-->

# Developer Deployment Guide

This document is the developer-facing deployment and operations guide for the project:
`Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms`.

It covers:
- local development setup
- simulation execution
- test workflow
- optional real-drone deployment
- production/safety checklist

## 1. Repository Overview

Core runtime modules:
- `main.py`: system entrypoint
- `swarm_manager.py`: orchestration, mission flow, leader/follower control
- `drone.py`: per-drone state, movement, RTH, emergency handling
- `dynamic_obstacles.py`: trajectory prediction and avoidance
- `latency_monitor.py`: RTT/jitter/watchdog and fallback gating
- `acoustic_tracking.py`: GCC-PHAT, TDOA, and fusion
- `communication.py`: encrypted messaging
- `flying_ledger.py`: append-only signed ledger replication
- `ml_system.py`: personal ML support logic
- `gui.py`: PyQt5 operator interface
- `dronecontroller.cpp` / `dronecontroller.h`: optional low-level C++ layer

## 2. Environment Requirements

Minimum:
- Python 3.10+ (recommended: 3.11)
- pip + virtual environment support
- OS: Windows 10/11, Ubuntu 20.04+, or macOS 12+

Optional:
- C++17-compatible compiler (for native controller workflows)
- QGroundControl (real-flight ops)
- MAVSDK/PyMAVLink toolchain (real-drone integration)

## 3. Local Developer Setup

### 3.1 Clone and Virtual Environment

```bash
git clone [https://github.com/smshagor-dev/decentralized-coordination-and-acoustic-localization.git](https://github.com/smshagor-dev/decentralized-coordination-and-acoustic-localization.git)
cd "Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms"
python -m venv venv
```

Windows:

```bash
venv\Scripts\activate
```

Linux/macOS:

```bash
source venv/bin/activate
```

### 3.2 Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

If you use `.env`, ensure it exists and contains the variables your environment needs.

## 4. Run Modes

### 4.1 Full System (Simulation)

```bash
python main.py
```

Expected behavior:
- swarm manager starts
- GUI launches
- demo drones are initialized
- dynamic obstacle simulation is active

### 4.2 Test Mode

```bash
python main.py --test
```

### 4.3 Unit/Feature Tests

```bash
python -m unittest test_dynamic_features.py
```

## 5. Real-Drone Deployment (Optional)

Use this mode only after simulation is stable and safety procedures are defined.

### 5.1 Required Environment Variables

Set these variables before running:
- `REAL_DRONE_ENABLED=1`
- `REAL_DRONE_CONNECTIONS=1=udpin://:14540,2=udpin://:14541`
- `REAL_GPS_REF_LAT=<latitude>`
- `REAL_GPS_REF_LON=<longitude>`

If variables are absent, the project runs in simulation mode.

### 5.2 Flight Controller and Link Validation

Before arming:
- verify telemetry link stability
- verify GPS lock and home position
- verify per-drone battery and motor health
- verify geofence/failsafe parameters in FC stack

### 5.3 Operational Sequence

1. Start system (`python main.py`)
2. Confirm all drones connected in GUI
3. Arm all
4. Controlled takeoff
5. Execute mission and monitor:
   - RTT/jitter/fallback indicators
   - obstacle avoidance events
   - acoustic confidence and localization output
6. Return-to-home or land
7. Disarm and archive logs

## 6. Build and Native Components

The C++ controller files are present:
- `dronecontroller.cpp`
- `dronecontroller.h`

If you compile native components, use your local CMake/compiler workflow and keep generated artifacts inside `build/`.

## 7. Logs and Artifacts

Primary output directories:
- `logs/`: runtime logs (system, swarm, drone, communication, ML)
- `performance_graphs/csv/`: latency/stat CSV exports
- `performance_graphs/img/`: generated plots
- `performance_graphs/logs/`: merged logs for plotting
- `models/`: model artifacts/snapshots

Developer recommendation:
- archive logs and CSV/image outputs for every major run
- tag artifacts with test scenario and timestamp

## 8. Deployment Checklist (Developer Gate)

Pre-deployment gate:
- [ ] dependencies install cleanly in fresh venv
- [ ] simulation run completes without critical exceptions
- [ ] `test_dynamic_features.py` passes
- [ ] fallback mode triggers correctly under injected latency
- [ ] emergency and RTH controls verified
- [ ] logs generated and readable

Real-flight gate:
- [ ] FC firmware + params validated
- [ ] comm encryption configured
- [ ] GPS/home/geofence verified
- [ ] emergency landing zone confirmed
- [ ] manual override path available at all times

## 9. Troubleshooting

### 9.1 GUI not starting
- confirm PyQt dependencies installed from `requirements.txt`
- run inside activated virtual environment

### 9.2 Drone connection issues
- validate connection strings in `REAL_DRONE_CONNECTIONS`
- verify ports/serial permissions and FC telemetry baud settings

### 9.3 Frequent fallback activation
- inspect RTT/jitter in GUI/status logs
- reduce network latency or processing overload
- verify adaptive threshold logic in `swarm_manager.py`

### 9.4 Acoustic localization instability
- check sensor count and synchronization
- confirm sample rate and GCC-PHAT pipeline health
- compare confidence score against configured threshold

## 10. Developer Notes

- Keep architecture and behavior docs in sync with code changes.
- Update `readme.md` whenever deployment assumptions change.
- Prefer simulation-first validation before any hardware deployment.
- Never operate real drones without compliance to local aviation laws and site safety constraints.
