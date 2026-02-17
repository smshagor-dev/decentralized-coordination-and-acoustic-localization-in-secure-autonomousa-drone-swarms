/*
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
*/
/**
 * DroneController.h
 * Low-level drone control interface for real hardware integration
 * Compatible with PX4/MAVLink protocol
 */

#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>
#include <deque>
#include <mutex>

namespace DroneSwarm {

// Flight modes matching PX4
enum class FlightMode {
    MANUAL = 0,
    STABILIZED,
    ALTITUDE_HOLD,
    POSITION_HOLD,
    AUTO_MISSION,
    AUTO_RTL,
    AUTO_LAND,
    AUTO_TAKEOFF,
    OFFBOARD,
    EMERGENCY_RETURN
};

// Motor status
struct MotorStatus {
    int motor_id;
    bool operational;
    bool degraded;
    float rpm;
    float vibration;
    float temperature;
    float current;
};

// Position structure
struct Position {
    double latitude;   // degrees
    double longitude;  // degrees
    float altitude;    // meters (MSL)
    float relative_alt; // meters (relative to home)
    
    Position() : latitude(0), longitude(0), altitude(0), relative_alt(0) {}
    Position(double lat, double lon, float alt) 
        : latitude(lat), longitude(lon), altitude(alt), relative_alt(alt) {}
};

// Velocity structure
struct Velocity {
    float vx;  // m/s north
    float vy;  // m/s east
    float vz;  // m/s down (NED frame)
    
    Velocity() : vx(0), vy(0), vz(0) {}
    Velocity(float x, float y, float z) : vx(x), vy(y), vz(z) {}
};

// Attitude structure
struct Attitude {
    float roll;   // radians
    float pitch;  // radians
    float yaw;    // radians
    
    Attitude() : roll(0), pitch(0), yaw(0) {}
};

// Battery status
struct BatteryStatus {
    float voltage;          // volts
    float current;          // amperes
    float remaining;        // percentage 0-100
    int32_t time_remaining; // seconds
    
    BatteryStatus() : voltage(0), current(0), remaining(100), time_remaining(-1) {}
};

// Telemetry data
struct Telemetry {
    Position position;
    Velocity velocity;
    Attitude attitude;
    BatteryStatus battery;
    FlightMode flight_mode;
    bool armed;
    bool connected;
    std::vector<MotorStatus> motors;
    uint64_t timestamp;
    
    Telemetry() : flight_mode(FlightMode::MANUAL), armed(false), connected(false), timestamp(0) {
        motors.resize(4); // Default quadcopter
        for (int i = 0; i < 4; i++) {
            motors[i].motor_id = i;
            motors[i].operational = true;
            motors[i].degraded = false;
            motors[i].rpm = 0.0f;
            motors[i].vibration = 0.0f;
            motors[i].temperature = 25.0f;
            motors[i].current = 0.0f;
        }
    }
};

struct LatencyMetrics {
    double cpp_to_py_ms = 0.0;
    double py_processing_ms = 0.0;
    double py_to_cpp_ms = 0.0;
    double total_round_trip_ms = 0.0;
    double threshold_ms = 220.0;
    size_t samples = 0;
    bool fallback_required = false;
};

class LatencyMonitor {
public:
    explicit LatencyMonitor(size_t window_size = 120, double threshold_ms = 220.0);

    void markCppSend(uint64_t ts_us);
    void markPyReceive(uint64_t ts_us);
    void markPySend(uint64_t ts_us);
    LatencyMetrics markCppReceive(uint64_t ts_us);
    LatencyMetrics getAverages() const;

private:
    struct Sample {
        double cpp_to_py_ms = 0.0;
        double py_processing_ms = 0.0;
        double py_to_cpp_ms = 0.0;
        double total_round_trip_ms = 0.0;
    };

    size_t window_size_;
    double threshold_ms_;
    mutable std::mutex mutex_;
    std::deque<Sample> samples_;

    uint64_t t_cpp_send_us_ = 0;
    uint64_t t_py_receive_us_ = 0;
    uint64_t t_py_send_us_ = 0;
};

/**
 * DroneController - Low-level interface for real drone control
 * This class provides the bridge between high-level Python logic
 * and actual drone hardware via MAVLink/PX4
 */
class DroneController {
public:
    /**
     * Constructor
     * @param connection_string MAVLink connection (e.g., "udp://:14540", "serial:///dev/ttyUSB0:57600")
     * @param drone_id Unique drone identifier
     */
    DroneController(const std::string& connection_string, int drone_id);
    
    /**
     * Destructor - ensures clean disconnection
     */
    ~DroneController();
    
    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Arming and mode control
    bool arm();
    bool disarm();
    bool setFlightMode(FlightMode mode);
    
    // Flight commands
    bool takeoff(float altitude);
    bool land();
    bool returnToLaunch();
    bool emergencyLand();
    
    // Position control
    bool gotoPosition(const Position& target);
    bool gotoPositionNED(float north, float east, float down);
    bool holdPosition();
    
    // Velocity control
    bool setVelocity(const Velocity& vel);
    bool setVelocityNED(float vn, float ve, float vd);
    
    // Telemetry
    Telemetry getTelemetry() const;
    Position getPosition() const;
    BatteryStatus getBatteryStatus() const;
    std::vector<MotorStatus> getMotorStatus() const;
    
    // Motor control (for testing)
    bool setMotorTest(int motor_id, float throttle_percent);
    bool disableMotor(int motor_id); // Simulate motor failure
    
    // Callbacks
    using TelemetryCallback = std::function<void(const Telemetry&)>;
    void setTelemetryCallback(TelemetryCallback callback);
    
    using EmergencyCallback = std::function<void(const std::string&)>;
    void setEmergencyCallback(EmergencyCallback callback);
    
    // Heartbeat
    void sendHeartbeat();
    uint64_t getLastHeartbeat() const;
    
private:
    class Impl; // Forward declaration for PIMPL
    std::unique_ptr<Impl> pImpl;
    
    // Internal methods
    void telemetryLoop();
    void checkMotorHealth();
    void detectMotorHealth();
    void activateSelfHealingMode(int failed_motor_index);
    void redistributeThrust(int failed_motor_index);
    void updateAdaptivePID();
    void updateBatteryStatus();
};

/**
 * MAVLinkInterface - Direct MAVLink protocol interface
 */
class MAVLinkInterface {
public:
    MAVLinkInterface(const std::string& connection_string);
    ~MAVLinkInterface();
    
    bool connect();
    void disconnect();
    
    // Send MAVLink commands
    bool sendCommand(uint16_t command, float param1 = 0, float param2 = 0,
                    float param3 = 0, float param4 = 0, float param5 = 0,
                    float param6 = 0, float param7 = 0);
    
    bool sendPositionTarget(float x, float y, float z, float vx, float vy, float vz);
    bool sendAttitudeTarget(float roll, float pitch, float yaw, float thrust);
    
    // Receive MAVLink messages
    bool receiveMessage(void* message, int timeout_ms = 100);
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

/**
 * Helper functions
 */
namespace Utils {
    // Convert GPS coordinates to local NED frame
    void gpsToNED(double lat, double lon, float alt,
                 double home_lat, double home_lon, float home_alt,
                 float& north, float& east, float& down);
    
    // Convert local NED to GPS
    void nedToGPS(float north, float east, float down,
                 double home_lat, double home_lon, float home_alt,
                 double& lat, double& lon, float& alt);
    
    // Distance between two GPS points
    float gpsDistance(double lat1, double lon1, double lat2, double lon2);
    
    // Get current timestamp in microseconds
    uint64_t getTimestampUs();
}

} // namespace DroneSwarm

#endif // DRONE_CONTROLLER_H
