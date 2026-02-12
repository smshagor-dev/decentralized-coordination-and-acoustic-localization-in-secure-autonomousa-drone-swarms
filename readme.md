# Drone Swarm Management System

A complete, production-ready drone fleet management system with automatic leader election, fault tolerance, secure communication, and real drone integration capabilities.

## 🎯 Project Overview

This system implements a secure and fault-tolerant drone swarm with the following key features:

- **Automatic Leader Election**: When the leader drone fails, remaining drones automatically elect a new leader based on battery level, signal strength, and processing capability
- **Dynamic Battery Management**: Real-time battery monitoring starting at 100%, with automatic return-to-home at 30% and emergency landing at 20%
- **Emergency Landing System**: Intelligent emergency landing with return-to-home capabilities
- **Motor Failure Detection**: Automatic detection and handling of motor failures
- **Secure Communication**: AES-256 encrypted drone-to-drone communication
- **ML Decision Support**: Machine learning for obstacle avoidance and formation optimization
- **Real Drone Integration**: Ready for MAVLink/PX4 integration with real hardware

## 📁 Project Structure

```
drone_swarm_system/
├── main.py                      # Application entry point
├── requirements.txt             # Python dependencies
├── CMakeLists.txt              # C++ build configuration
├── README.md                   # This file
├── src/
│   ├── python/                 # Python components
│   │   ├── drone.py           # Core drone class with dynamic battery
│   │   ├── swarm_manager.py   # Swarm management and leader election
│   │   ├── communication.py   # Secure encrypted communication
│   │   ├── ml_system.py       # ML decision support
│   │   └── gui.py             # PyQt5 graphical interface
│   └── cpp/                   # C++ low-level control
│       ├── DroneController.h  # Drone controller interface
│       ├── DroneController.cpp # Controller implementation
│       └── main_test.cpp      # C++ test program
├── config/                    # Configuration files
├── logs/                      # System logs
└── tests/                     # Unit tests
```

## 🚀 Installation

### System Requirements

- Python 3.8+
- C++ compiler (GCC 7+ or Clang 5+)
- CMake 3.10+
- Qt5 development libraries

### Python Setup

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### C++ Compilation

```bash
# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Install (optional)
sudo make install
```

## 💻 Usage

### Running the System

```bash
# Start the complete system with GUI
python main.py
```

### GUI Controls

The graphical interface provides:

1. **Visualization Panel**: Real-time 3D-like view of drone positions
   - Gold diamond: Leader drone
   - Blue circles: Follower drones
   - Red: Emergency state
   - Green markers: Home positions

2. **Control Panel**:
   - Add/Remove drones
   - Arm/Disarm all
   - Takeoff/Land all
   - Return to home
   - Emergency land
   - Formation control (line, V, circle, grid)

3. **Status Panel**:
   - Total and active drones
   - Leader ID
   - Average battery
   - Detailed drone table

4. **Test Scenarios**:
   - Simulate motor failure
   - Test leader failure and re-election

## 🎮 Key Features

### 1. Dynamic Battery Management

All drones start with 100% battery and consume power based on activity:
- **Idle**: 0.1% per 100 seconds
- **Hover**: 1% per 100 seconds  
- **Flying**: 2% per 100 seconds
- **Emergency**: 0.5% per 100 seconds

**Automatic Actions**:
- Battery < 30%: Automatic return to home
- Battery < 20%: Emergency landing
- Battery = 0%: System shutdown

### 2. Leader Election Algorithm

When a leader fails, automatic election based on:
```
Score = 0.4 × Battery + 0.3 × Signal + 0.2 × Processing + 0.1 × Motors
```

The drone with the highest score becomes the new leader.

### 3. Emergency Landing

Triggered by:
- Critical battery (<20%)
- Motor failure (≥2 motors)
- Communication loss
- Manual emergency command

The drone:
1. Stops all waypoint following
2. Initiates controlled descent
3. Attempts to stabilize orientation
4. Lands at current position or returns home if possible

### 4. Return-to-Home (RTH)

Automatically activated when:
- Low battery (<30%)
- Signal loss
- Mission abort
- Manual command

Process:
1. Mark current position
2. Climb to safe altitude (if needed)
3. Fly direct path to home
4. Descend and land at home position

### 5. Secure Communication

- **AES-256-GCM encryption**
- **HMAC authentication**
- **Replay attack prevention**
- **No internet required** - direct multicast
- **Heartbeat system** with 5-second timeout

### 6. Formation Flight

Supported formations:
- **Line**: Single file behind leader
- **V-Formation**: Classic V-shape
- **Circle**: Circular perimeter around leader
- **Grid**: Rectangular grid pattern

## 🔧 Real Drone Integration

### MAVLink/PX4 Setup

For real drone deployment, update connection strings:

```python
# In drone.py
drone = Drone(
    drone_id=1,
    home_position=Position(lat, lon, alt),
    real_drone_connection="udp://:14540"  # PX4 SITL
    # or "serial:///dev/ttyUSB0:57600"    # Serial connection
)
```

### Hardware Requirements per Drone

- Flight Controller: PX4 or ArduPilot
- Companion Computer: Raspberry Pi 4+ or NVIDIA Jetson
- GPS Module: u-blox M8N or better
- Telemetry: 915MHz or 433MHz radio
- Battery: 4S LiPo (minimum)
- Motors: 4 brushless motors (quadcopter)

### Deployment Steps

1. **Flash PX4 Firmware** on flight controller
2. **Install MAVSDK** on companion computer:
   ```bash
   pip install mavsdk
   ```
3. **Configure Communication**:
   - Set up MAVLink connection
   - Configure telemetry radio
   - Set swarm encryption key

4. **Calibration**:
   - Compass calibration
   - Accelerometer calibration
   - ESC calibration
   - Radio calibration

5. **Test Flight**:
   - Manual stabilization test
   - Position hold test
   - RTH test
   - Formation flight test

## 🧪 Testing

### Unit Tests

```bash
# Run all tests
pytest tests/

# Run with coverage
pytest --cov=src tests/
```

### Simulation Testing

```bash
# Test leader election
python -c "from tests.test_swarm import test_leader_election; test_leader_election()"

# Test motor failure
python -c "from tests.test_drone import test_motor_failure; test_motor_failure()"

# Test battery management
python -c "from tests.test_drone import test_battery_management; test_battery_management()"
```

### Integration with PX4 SITL

```bash
# Start PX4 SITL
cd PX4-Autopilot
make px4_sitl gazebo

# In another terminal, run the system
python main.py
```

## 📊 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Interface (GUI)                  │
│                      (PyQt5)                            │
└────────────────┬────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────┐
│              Swarm Manager (Python)                      │
│  • Leader Election    • Fault Detection                 │
│  • Formation Control  • Mission Planning                │
└────┬─────────────────────────────────────────────┬──────┘
     │                                             │
┌────▼──────────────┐                  ┌──────────▼──────┐
│  Communication    │                  │   ML System     │
│  • AES Encryption │                  │   • Obstacle    │
│  • Heartbeat      │                  │   • Formation   │
│  • Message Queue  │                  │   • Path Plan   │
└────┬──────────────┘                  └─────────────────┘
     │
┌────▼─────────────────────────────────────────────────────┐
│              Individual Drones (Python + C++)             │
│  ┌──────────────┐  ┌─────────────┐  ┌─────────────────┐│
│  │ Drone State  │  │  Battery    │  │  Motor Control  ││
│  │ Management   │  │  Monitor    │  │  (C++)          ││
│  └──────────────┘  └─────────────┘  └─────────────────┘│
└────┬──────────────────────────────────────────────────────┘
     │
┌────▼──────────────────────────────────────────────────────┐
│          MAVLink / PX4 Interface (C++)                     │
│  • Flight Control  • Telemetry  • Safety Systems          │
└────┬───────────────────────────────────────────────────────┘
     │
┌────▼───────────────────────────────────────────────────────┐
│              Physical Drone Hardware                        │
│  • Flight Controller  • GPS  • Motors  • Sensors           │
└────────────────────────────────────────────────────────────┘
```

## 🔐 Security Features

1. **Encrypted Communication**: All drone-to-drone messages encrypted with AES-256
2. **Authentication**: HMAC-based message authentication
3. **Replay Protection**: Sequence numbers prevent replay attacks
4. **Access Control**: Only authenticated drones can join swarm
5. **Secure Key Exchange**: PBKDF2 key derivation from shared secret

## ⚙️ Configuration

### config/swarm_config.json

```json
{
  "swarm": {
    "encryption_key": "your_secure_key_here",
    "heartbeat_timeout": 5.0,
    "max_drones": 10
  },
  "battery": {
    "low_threshold": 30.0,
    "critical_threshold": 20.0,
    "consumption_rates": {
      "idle": 0.001,
      "hover": 0.01,
      "flying": 0.02
    }
  },
  "flight": {
    "max_speed": 15.0,
    "max_altitude": 120.0,
    "min_spacing": 5.0
  }
}
```

## 📝 Logging

All system events are logged to `logs/` directory:

- `system_YYYYMMDD_HHMMSS.log`: Main system log
- `drone_N.log`: Individual drone logs
- `swarm_manager.log`: Swarm management events
- `comm_drone_N.log`: Communication logs
- `ml_system.log`: ML decision logs

## 🐛 Troubleshooting

### GUI doesn't start
- Ensure PyQt5 is installed: `pip install PyQt5`
- Check X11 forwarding if on remote system

### Drones not communicating
- Check firewall settings for multicast
- Verify encryption keys match
- Check network connectivity

### Battery draining too fast
- Adjust consumption rates in configuration
- Check for motor failures
- Verify idle mode is working

### Motor failure not detected
- Check motor status update frequency
- Verify telemetry thread is running
- Ensure motor health monitoring is enabled

## 📚 API Reference

### Drone Class

```python
drone = Drone(drone_id, home_position, real_drone_connection)
drone.arm()                    # Arm motors
drone.takeoff()               # Takeoff to default altitude
drone.goto(position)          # Fly to position
drone.return_to_home(reason)  # Return to home
drone.emergency_land(reason)  # Emergency landing
drone.get_status()            # Get full status dict
```

### SwarmManager Class

```python
swarm = SwarmManager()
swarm.add_drone(drone)              # Add drone
swarm.remove_drone(drone_id)        # Remove drone
swarm.elect_leader()                # Force leader election
swarm.formation_flight(type)        # Formation control
swarm.emergency_land_all(reason)    # Emergency all
```

## 🤝 Contributing

This project is designed for academic and research purposes. For contributions:

1. Fork the repository
2. Create feature branch
3. Add tests for new features
4. Submit pull request

## 📜 License

This project is for educational and research purposes. See LICENSE file for details.

## 👥 Authors

- Md Shahanur Islam Shagor
- www.smshagor.com

- Bachelor of Science
- University: Voronezh State University of Forestry and Technology, Voronezh, Russia
- Department of Computer Science and Microelectronic Engineering

## 📞 Support

For issues and questions:
- Check logs in `logs/` directory
- Review troubleshooting section
- Contact project maintainers
- smshagor.ru@gmail.com or contact@smshagor.com
- Whatsapp https://wa.ma/+79954949836
- Telegram @smshagor1

## 🎓 Academic Context

This system was developed as part of a research project on:
- Fault-tolerant drone swarm systems
- Automatic leader replacement
- Secure drone-to-drone communication
- Machine learning for autonomous navigation

The implementation combines theoretical concepts with practical, deployable code suitable for real drone hardware.

---

**Status**: Production-Ready for Simulation | Integration-Ready for Real Drones
**Version**: 1.0.0
**Last Updated**: February 2026