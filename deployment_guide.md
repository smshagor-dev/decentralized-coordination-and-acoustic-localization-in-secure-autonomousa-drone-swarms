# Real Drone Deployment Guide

This guide explains how to deploy the Drone Swarm Management System on real hardware.

## 🛠️ Hardware Setup

### Per-Drone Requirements

1. **Flight Controller**
   - PX4 v1.12+ or ArduPilot 4.0+
   - Recommended: Pixhawk 4, Cube Orange, or Holybro Pixhawk 6C
   
2. **Companion Computer**
   - Raspberry Pi 4 (4GB+ RAM) or
   - NVIDIA Jetson Nano/Xavier
   - Running Ubuntu 20.04+ or Raspberry Pi OS

3. **Communication**
   - Telemetry Radio: 915MHz or 433MHz (SiK Radio)
   - WiFi Module or LoRa for swarm communication
   - GPS Module: u-blox M8N or better

4. **Power**
   - 4S LiPo battery (minimum 5000mAh)
   - Battery monitor/sensor
   - Power module for flight controller

5. **Frame and Motors**
   - Quadcopter frame (450mm-550mm)
   - 4x brushless motors (920KV-1000KV)
   - 4x ESCs (30A+)
   - Propellers (10-11 inch)

## 📡 Software Installation on Companion Computer

### 1. Operating System Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y python3-pip python3-dev git cmake \
    libqt5core5a libqt5gui5 libqt5widgets5 \
    python3-pyqt5 build-essential
```

### 2. Clone and Setup Project

```bash
# Clone repository
git clone <repository-url>
cd drone_swarm_system

# Run quick setup
chmod +x quickstart.sh
./quickstart.sh
```

### 3. Install MAVLink/MAVSDK

```bash
# Activate virtual environment
source venv/bin/activate

# Install MAVLink libraries
pip install pymavlink mavsdk

# For C++ (optional)
# Follow MAVSDK C++ installation: https://mavsdk.mavlink.io/
```

### 4. Configure Serial Connection

```bash
# Find flight controller serial port
ls -l /dev/ttyACM* /dev/ttyUSB*

# Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# Reboot for group changes to take effect
sudo reboot
```

## ⚙️ Flight Controller Configuration

### PX4 Configuration

1. **Flash PX4 Firmware**
   ```bash
   # Using QGroundControl or
   make px4_fmu-v5_default upload
   ```

2. **Enable MAVLink on Companion Port**
   - In QGroundControl:
     - Parameters → MAV_1_CONFIG → Set to TELEM2
     - MAV_1_MODE → Set to Onboard
     - SER_TEL2_BAUD → Set to 921600

3. **Enable Required Streams**
   ```
   MAV_1_RATE = 10000
   MAV_1_FORWARD = Enabled
   ```

### Connection Testing

```bash
# Test MAVLink connection
python3 << EOF
from pymavlink import mavutil

# Connect (adjust port as needed)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)

# Wait for heartbeat
master.wait_heartbeat()
print("Heartbeat received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")
EOF
```

## 🚁 Drone Configuration

### 1. Update Drone Class for Real Hardware

Edit `src/python/drone.py`:

```python
# Add real MAVLink connection
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

class Drone:
    def __init__(self, drone_id: int, home_position: Position, 
                 real_drone_connection: str):
        # ... existing init ...
        
        # Connect to real drone
        if real_drone_connection:
            self.system = System()
            asyncio.run(self._connect_real_drone(real_drone_connection))
    
    async def _connect_real_drone(self, connection_string):
        """Connect to real drone via MAVLink"""
        await self.system.connect(system_address=connection_string)
        
        async for state in self.system.core.connection_state():
            if state.is_connected:
                self.mavlink_connected = True
                self.logger.info(f"Drone {self.drone_id}: Connected to real hardware")
                break
```

### 2. Implement Real Flight Commands

```python
async def _real_takeoff(self, altitude: float):
    """Real takeoff via MAVLink"""
    await self.system.action.arm()
    await self.system.action.set_takeoff_altitude(altitude)
    await self.system.action.takeoff()

async def _real_land(self):
    """Real landing via MAVLink"""
    await self.system.action.land()

async def _real_goto(self, position: Position):
    """Real position command via MAVLink"""
    await self.system.offboard.set_position_ned(
        PositionNedYaw(
            position.x, position.y, -position.z, 0.0
        )
    )
```

### 3. Configure Connection String

In `main.py`:

```python
# For simulation (PX4 SITL)
connection_string = "udp://:14540"

# For real drone via serial
connection_string = "serial:///dev/ttyACM0:921600"

# For real drone via WiFi
connection_string = "udp://192.168.1.100:14550"

drone = Drone(
    drone_id=i,
    home_position=Position(lat, lon, alt),
    real_drone_connection=connection_string
)
```

## 🔐 Secure Communication Setup

### 1. Configure Swarm Network

For LoRa modules:
```python
# In communication.py
class SecureCommunication:
    def __init__(self, drone_id: int):
        # Configure LoRa parameters
        self.lora_frequency = 915.0  # MHz
        self.lora_bandwidth = 125000
        self.lora_spreading = 7
```

For WiFi mesh:
```bash
# Setup mesh network on each drone
sudo apt install batman-adv

# Configure batman-adv
sudo modprobe batman-adv
sudo batctl if add wlan0
sudo ifconfig bat0 up
```

### 2. Generate Encryption Keys

```bash
# Generate secure swarm key
python3 -c "import secrets; print(secrets.token_urlsafe(32))"

# Update config/swarm_config.json with generated key
```

## 🧪 Pre-Flight Testing

### 1. Ground Tests

```bash
# Test 1: Motor Test
python3 << EOF
from drone import Drone, Position
drone = Drone(1, Position(0, 0, 0), "serial:///dev/ttyACM0:921600")
drone.connect()
# Test each motor individually
for i in range(4):
    drone.system.action.test_motor(i, 10)  # 10% throttle
EOF
```

### 2. Communication Test

```bash
# Test swarm communication between 2 drones
# On Drone 1:
python3 test_communication.py --drone-id 1

# On Drone 2:
python3 test_communication.py --drone-id 2
```

### 3. Position Hold Test

```bash
# Test position hold and GPS accuracy
python3 << EOF
from drone import Drone, Position
drone = Drone(1, Position(0, 0, 0), "serial:///dev/ttyACM0:921600")
drone.arm()
drone.takeoff()
# Hold for 30 seconds and verify position stability
time.sleep(30)
drone.land()
EOF
```

## 🚀 Flight Operations

### Pre-Flight Checklist

- [ ] Battery fully charged (>95%)
- [ ] GPS lock (>10 satellites)
- [ ] Compass calibrated
- [ ] Accelerometer calibrated
- [ ] Radio link quality >80%
- [ ] Swarm communication tested
- [ ] Home position set
- [ ] Flight area clear
- [ ] Emergency landing zones identified
- [ ] Kill switch accessible

### Launch Procedure

1. **Initialize System**
   ```bash
   python main.py
   ```

2. **Verify All Drones Connected**
   - Check GUI for all drone connections
   - Verify GPS locks
   - Check battery levels

3. **Arm and Takeoff**
   - Use GUI "Arm All" button
   - Wait for confirmation
   - Use "Takeoff All" button
   - Monitor altitude

4. **Execute Mission**
   - Apply formation
   - Monitor swarm health
   - Watch for warnings

5. **Return and Land**
   - Use "RTH All" for controlled return
   - Monitor landing
   - Disarm after landing

### Emergency Procedures

**Lost Communication:**
- System automatically triggers RTH
- Monitor with GCS (QGroundControl)
- Be ready to take manual control

**Motor Failure:**
- System detects and initiates emergency landing
- Clear landing area
- Be ready to catch if needed (small drones)

**Low Battery:**
- System automatically returns home at 30%
- Emergency lands at 20%
- Monitor battery voltage

**Swarm Collision Risk:**
- ML system increases separation
- Manual control available
- Emergency land if needed

## 📊 Monitoring and Logging

### Real-Time Monitoring

```bash
# Monitor all logs in real-time
tail -f logs/system_*.log logs/drone_*.log

# Monitor specific drone
tail -f logs/drone_1.log

# Monitor swarm manager
tail -f logs/swarm_manager.log
```

### Post-Flight Analysis

```bash
# Analyze flight logs
python analyze_flight.py --log logs/system_20260211_143022.log

# Generate flight report
python generate_report.py --date 2026-02-11
```

## 🔧 Troubleshooting

### Connection Issues

**Problem:** Drone not connecting
```bash
# Check serial port
ls -l /dev/tty*

# Test connection manually
sudo minicom -D /dev/ttyACM0 -b 921600

# Check permissions
sudo chmod 666 /dev/ttyACM0
```

**Problem:** MAVLink timeout
- Check baud rate matches PX4 configuration
- Verify MAVLink streams are enabled
- Check cable connections

### GPS Issues

**Problem:** No GPS lock
- Ensure clear sky view
- Wait longer (can take 5+ minutes on first boot)
- Check GPS module is powered
- Verify GPS configuration in PX4

### Battery Issues

**Problem:** Battery reading incorrect
- Calibrate battery voltage sensor
- Check power module connection
- Verify battery parameters in PX4

## 📱 Remote Operation

### Ground Control Station Setup

```bash
# Install QGroundControl on ground station
sudo apt install qgroundcontrol

# Configure for multiple drones
# Set unique System IDs for each drone
```

### Remote Access

```bash
# Access companion computer remotely
ssh pi@drone1.local

# Start system remotely
ssh pi@drone1.local "cd drone_swarm_system && python main.py"

# View GUI over X11 forwarding
ssh -X pi@drone1.local
```

## 🛡️ Safety Features

### Geofence

Configure in PX4:
```
GF_ACTION = 2 (RTL)
GF_MAX_HOR_DIST = 500 (meters)
GF_MAX_VER_DIST = 120 (meters)
```

### Failsafes

Configure failsafes:
- RC loss: RTH after 5 seconds
- Data link loss: RTH after 10 seconds
- Low battery: RTH at 30%, Land at 20%
- Geofence breach: RTH

### Manual Override

Always maintain ability to:
- Take manual control via RC transmitter
- Trigger kill switch
- Use ground station for manual commands

## 📞 Support and Maintenance

### Regular Maintenance

- Clean propellers and motors
- Check all connections
- Update firmware monthly
- Calibrate sensors quarterly
- Replace props every 50 flights
- Check battery health weekly

### Logging Issues

For support, provide:
- System logs from `logs/` directory
- Flight controller logs (.ulg files)
- Video of issue if available
- System configuration

---

**Important:** Always follow local regulations for drone operations. Ensure you have proper licensing and permissions before flying.

**Safety First:** Never fly over people, near airports, or in restricted airspace.
## Update: February 17, 2026
- Added Differential Drone Immune System (Self-Healing Flight System) in dronecontroller.cpp.
- Added real-time motor health logic (RPM drop detection at >=10%), thrust redistribution, adaptive PID, and emergency return handling for 2+ degraded motors.
- Added structured immune logs, including [IMMUNE] Motor X degraded ... | Compensation Active and SWARM_ALERT behavior.
- Swarm Status drone table (ID, Role, Mode, Battery, Altitude, Status) is now fully dynamic in the GUI (defensive row updates, dynamic resizing, always-visible vertical scrollbar, smooth scrolling, and sortable columns).
- Latency indicators (C++->Py, Py Proc, Py->C++, RTT, RTT Jitter) are dynamically refreshed from runtime latency stats.
