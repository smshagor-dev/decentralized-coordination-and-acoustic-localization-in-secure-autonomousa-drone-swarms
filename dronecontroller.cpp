/**
 * DroneController.cpp
 * Implementation of low-level drone control
 */

#include "DroneController.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace DroneSwarm {

LatencyMonitor::LatencyMonitor(size_t window_size, double threshold_ms)
    : window_size_(std::max<size_t>(10, window_size)), threshold_ms_(threshold_ms) {}

void LatencyMonitor::markCppSend(uint64_t ts_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    t_cpp_send_us_ = ts_us;
}

void LatencyMonitor::markPyReceive(uint64_t ts_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    t_py_receive_us_ = ts_us;
}

void LatencyMonitor::markPySend(uint64_t ts_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    t_py_send_us_ = ts_us;
}

LatencyMetrics LatencyMonitor::markCppReceive(uint64_t ts_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (t_cpp_send_us_ == 0 || t_py_receive_us_ == 0 || t_py_send_us_ == 0) {
        LatencyMetrics metrics;
        metrics.threshold_ms = threshold_ms_;
        metrics.samples = samples_.size();
        return metrics;
    }

    auto clamp_ms = [](double us) { return std::max(0.0, us / 1000.0); };
    Sample sample;
    sample.cpp_to_py_ms = clamp_ms(static_cast<double>(t_py_receive_us_ - t_cpp_send_us_));
    sample.py_processing_ms = clamp_ms(static_cast<double>(t_py_send_us_ - t_py_receive_us_));
    sample.py_to_cpp_ms = clamp_ms(static_cast<double>(ts_us - t_py_send_us_));
    sample.total_round_trip_ms = clamp_ms(static_cast<double>(ts_us - t_cpp_send_us_));

    samples_.push_back(sample);
    while (samples_.size() > window_size_) {
        samples_.pop_front();
    }
    LatencyMetrics metrics;
    metrics.threshold_ms = threshold_ms_;
    metrics.samples = samples_.size();
    for (const auto& s : samples_) {
        metrics.cpp_to_py_ms += s.cpp_to_py_ms;
        metrics.py_processing_ms += s.py_processing_ms;
        metrics.py_to_cpp_ms += s.py_to_cpp_ms;
        metrics.total_round_trip_ms += s.total_round_trip_ms;
    }
    const double n = static_cast<double>(samples_.size());
    metrics.cpp_to_py_ms /= n;
    metrics.py_processing_ms /= n;
    metrics.py_to_cpp_ms /= n;
    metrics.total_round_trip_ms /= n;
    metrics.fallback_required = metrics.total_round_trip_ms > threshold_ms_;
    return metrics;
}

LatencyMetrics LatencyMonitor::getAverages() const {
    std::lock_guard<std::mutex> lock(mutex_);
    LatencyMetrics metrics;
    metrics.threshold_ms = threshold_ms_;
    metrics.samples = samples_.size();
    if (samples_.empty()) {
        return metrics;
    }

    for (const auto& s : samples_) {
        metrics.cpp_to_py_ms += s.cpp_to_py_ms;
        metrics.py_processing_ms += s.py_processing_ms;
        metrics.py_to_cpp_ms += s.py_to_cpp_ms;
        metrics.total_round_trip_ms += s.total_round_trip_ms;
    }
    const double n = static_cast<double>(samples_.size());
    metrics.cpp_to_py_ms /= n;
    metrics.py_processing_ms /= n;
    metrics.py_to_cpp_ms /= n;
    metrics.total_round_trip_ms /= n;
    metrics.fallback_required = metrics.total_round_trip_ms > threshold_ms_;
    return metrics;
}

// Implementation class (PIMPL pattern)
class DroneController::Impl {
public:
    std::string connection_string;
    int drone_id;
    bool connected;
    bool armed;
    FlightMode current_mode;
    
    Telemetry telemetry;
    std::mutex telemetry_mutex;
    
    TelemetryCallback telemetry_callback;
    EmergencyCallback emergency_callback;
    
    std::thread telemetry_thread;
    bool running;
    
    uint64_t last_heartbeat;
    Position home_position;
    
    Impl(const std::string& conn_str, int id)
        : connection_string(conn_str), drone_id(id), connected(false),
          armed(false), current_mode(FlightMode::MANUAL), running(false),
          last_heartbeat(0) {}
    
    ~Impl() {
        running = false;
        if (telemetry_thread.joinable()) {
            telemetry_thread.join();
        }
    }
};

// Constructor
DroneController::DroneController(const std::string& connection_string, int drone_id)
    : pImpl(std::make_unique<Impl>(connection_string, drone_id)) {
    std::cout << "DroneController initialized for Drone " << drone_id 
              << " with connection: " << connection_string << std::endl;
}

// Destructor
DroneController::~DroneController() {
    disconnect();
}

// Connect to drone
bool DroneController::connect() {
    if (pImpl->connected) {
        return true;
    }
    
    std::cout << "Connecting to drone " << pImpl->drone_id << "..." << std::endl;
    
    // In real implementation, establish MAVLink connection
    // For simulation, just mark as connected
    pImpl->connected = true;
    
    // Start telemetry thread
    pImpl->running = true;
    pImpl->telemetry_thread = std::thread(&DroneController::telemetryLoop, this);
    
    std::cout << "Drone " << pImpl->drone_id << " connected successfully" << std::endl;
    return true;
}

// Disconnect from drone
void DroneController::disconnect() {
    if (!pImpl->connected) {
        return;
    }
    
    std::cout << "Disconnecting drone " << pImpl->drone_id << std::endl;
    
    // Stop telemetry thread
    pImpl->running = false;
    if (pImpl->telemetry_thread.joinable()) {
        pImpl->telemetry_thread.join();
    }
    
    pImpl->connected = false;
}

bool DroneController::isConnected() const {
    return pImpl->connected;
}

// Arm motors
bool DroneController::arm() {
    if (!pImpl->connected) {
        std::cerr << "Cannot arm - drone not connected" << std::endl;
        return false;
    }
    
    if (pImpl->telemetry.battery.remaining < 10.0f) {
        std::cerr << "Cannot arm - battery too low" << std::endl;
        return false;
    }
    
    std::cout << "Arming drone " << pImpl->drone_id << std::endl;
    
    // In real implementation, send MAVLink ARM command
    pImpl->armed = true;
    pImpl->telemetry.armed = true;
    
    return true;
}

// Disarm motors
bool DroneController::disarm() {
    if (!pImpl->connected) {
        return false;
    }
    
    std::cout << "Disarming drone " << pImpl->drone_id << std::endl;
    
    pImpl->armed = false;
    pImpl->telemetry.armed = false;
    
    return true;
}

// Set flight mode
bool DroneController::setFlightMode(FlightMode mode) {
    if (!pImpl->connected) {
        return false;
    }
    
    std::cout << "Setting flight mode for drone " << pImpl->drone_id << std::endl;
    
    pImpl->current_mode = mode;
    pImpl->telemetry.flight_mode = mode;
    
    return true;
}

// Takeoff
bool DroneController::takeoff(float altitude) {
    if (!pImpl->armed) {
        std::cerr << "Cannot takeoff - drone not armed" << std::endl;
        return false;
    }
    
    std::cout << "Drone " << pImpl->drone_id << " taking off to " 
              << altitude << "m" << std::endl;
    
    setFlightMode(FlightMode::AUTO_TAKEOFF);
    
    // In real implementation, send MAVLink takeoff command
    // For simulation, just update mode
    
    return true;
}

// Land
bool DroneController::land() {
    if (!pImpl->connected) {
        return false;
    }
    
    std::cout << "Drone " << pImpl->drone_id << " landing" << std::endl;
    
    setFlightMode(FlightMode::AUTO_LAND);
    
    return true;
}

// Return to launch
bool DroneController::returnToLaunch() {
    if (!pImpl->connected) {
        return false;
    }
    
    std::cout << "Drone " << pImpl->drone_id << " returning to launch" << std::endl;
    
    setFlightMode(FlightMode::AUTO_RTL);
    
    return true;
}

// Emergency landing
bool DroneController::emergencyLand() {
    std::cout << "EMERGENCY LAND - Drone " << pImpl->drone_id << std::endl;
    
    if (pImpl->emergency_callback) {
        pImpl->emergency_callback("Emergency landing initiated");
    }
    
    setFlightMode(FlightMode::AUTO_LAND);
    
    return true;
}

// Go to position (GPS)
bool DroneController::gotoPosition(const Position& target) {
    if (!pImpl->armed) {
        return false;
    }
    
    std::cout << "Drone " << pImpl->drone_id << " going to position: "
              << target.latitude << ", " << target.longitude 
              << ", " << target.altitude << "m" << std::endl;
    
    setFlightMode(FlightMode::OFFBOARD);
    
    // In real implementation, send MAVLink position target
    
    return true;
}

// Go to position (local NED)
bool DroneController::gotoPositionNED(float north, float east, float down) {
    if (!pImpl->armed) {
        return false;
    }
    
    std::cout << "Drone " << pImpl->drone_id << " going to NED: "
              << north << ", " << east << ", " << down << std::endl;
    
    setFlightMode(FlightMode::OFFBOARD);
    
    return true;
}

// Hold position
bool DroneController::holdPosition() {
    if (!pImpl->armed) {
        return false;
    }
    
    setFlightMode(FlightMode::POSITION_HOLD);
    
    return true;
}

// Set velocity
bool DroneController::setVelocity(const Velocity& vel) {
    return setVelocityNED(vel.vx, vel.vy, vel.vz);
}

// Set velocity (NED)
bool DroneController::setVelocityNED(float vn, float ve, float vd) {
    if (!pImpl->armed) {
        return false;
    }
    
    setFlightMode(FlightMode::OFFBOARD);
    
    // In real implementation, send MAVLink velocity target
    
    return true;
}

// Get telemetry
Telemetry DroneController::getTelemetry() const {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    return pImpl->telemetry;
}

// Get position
Position DroneController::getPosition() const {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    return pImpl->telemetry.position;
}

// Get battery status
BatteryStatus DroneController::getBatteryStatus() const {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    return pImpl->telemetry.battery;
}

// Get motor status
std::vector<MotorStatus> DroneController::getMotorStatus() const {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    return pImpl->telemetry.motors;
}

// Motor test
bool DroneController::setMotorTest(int motor_id, float throttle_percent) {
    if (motor_id < 0 || motor_id >= pImpl->telemetry.motors.size()) {
        return false;
    }
    
    std::cout << "Testing motor " << motor_id << " at " 
              << throttle_percent << "%" << std::endl;
    
    // In real implementation, send MAVLink motor test command
    
    return true;
}

// Simulate motor failure
bool DroneController::disableMotor(int motor_id) {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    
    if (motor_id < 0 || motor_id >= pImpl->telemetry.motors.size()) {
        return false;
    }
    
    std::cout << "Disabling motor " << motor_id << " on drone " 
              << pImpl->drone_id << std::endl;
    
    pImpl->telemetry.motors[motor_id].operational = false;
    pImpl->telemetry.motors[motor_id].rpm = 0;
    
    // Trigger emergency if too many motors failed
    int operational = 0;
    for (const auto& motor : pImpl->telemetry.motors) {
        if (motor.operational) operational++;
    }
    
    if (operational < 3) {
        if (pImpl->emergency_callback) {
            pImpl->emergency_callback("Critical motor failure");
        }
        emergencyLand();
    }
    
    return true;
}

// Set telemetry callback
void DroneController::setTelemetryCallback(TelemetryCallback callback) {
    pImpl->telemetry_callback = callback;
}

// Set emergency callback
void DroneController::setEmergencyCallback(EmergencyCallback callback) {
    pImpl->emergency_callback = callback;
}

// Send heartbeat
void DroneController::sendHeartbeat() {
    pImpl->last_heartbeat = Utils::getTimestampUs();
}

// Get last heartbeat
uint64_t DroneController::getLastHeartbeat() const {
    return pImpl->last_heartbeat;
}

// Telemetry loop
void DroneController::telemetryLoop() {
    while (pImpl->running) {
        {
            std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
            
            // Update telemetry
            pImpl->telemetry.timestamp = Utils::getTimestampUs();
            pImpl->telemetry.connected = pImpl->connected;
            
            // In real implementation, receive and parse MAVLink messages
            // For simulation, update with simulated data
        }
        
        // Check motor health
        checkMotorHealth();
        
        // Update battery
        updateBatteryStatus();
        
        // Call callback if registered
        if (pImpl->telemetry_callback) {
            pImpl->telemetry_callback(pImpl->telemetry);
        }
        
        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Check motor health
void DroneController::checkMotorHealth() {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    
    for (auto& motor : pImpl->telemetry.motors) {
        if (motor.operational) {
            // Simulate normal motor operation
            motor.rpm = pImpl->armed ? 5000.0f : 0.0f;
            motor.temperature = 25.0f + (pImpl->armed ? 15.0f : 0.0f);
            motor.current = pImpl->armed ? 5.0f : 0.0f;
        }
    }
}

// Update battery status
void DroneController::updateBatteryStatus() {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    
    // In real implementation, read from battery management system
    // For simulation, slowly decrease based on flight state
    
    float discharge_rate = 0.0f;
    if (pImpl->armed) {
        if (pImpl->current_mode == FlightMode::AUTO_TAKEOFF) {
            discharge_rate = 0.02f; // 2% per second when taking off
        } else if (pImpl->current_mode == FlightMode::OFFBOARD) {
            discharge_rate = 0.01f; // 1% per second when flying
        } else {
            discharge_rate = 0.005f; // 0.5% per second when hovering
        }
    }
    
    // Apply discharge (would be real measurement in actual drone)
    // This is just for simulation
    static auto last_update = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_update).count();
    last_update = now;
    
    pImpl->telemetry.battery.remaining -= discharge_rate * dt;
    pImpl->telemetry.battery.remaining = std::max(0.0f, pImpl->telemetry.battery.remaining);
    
    // Estimate time remaining
    if (discharge_rate > 0) {
        pImpl->telemetry.battery.time_remaining = 
            static_cast<int32_t>(pImpl->telemetry.battery.remaining / discharge_rate);
    }
    
    // Voltage and current simulation
    pImpl->telemetry.battery.voltage = 11.1f + (pImpl->telemetry.battery.remaining / 100.0f) * 1.5f;
    pImpl->telemetry.battery.current = pImpl->armed ? 10.0f : 0.5f;
}

// Utility functions
namespace Utils {

void gpsToNED(double lat, double lon, float alt,
             double home_lat, double home_lon, float home_alt,
             float& north, float& east, float& down) {
    const double EARTH_RADIUS = 6378137.0; // meters
    
    double dlat = (lat - home_lat) * M_PI / 180.0;
    double dlon = (lon - home_lon) * M_PI / 180.0;
    
    north = static_cast<float>(dlat * EARTH_RADIUS);
    east = static_cast<float>(dlon * EARTH_RADIUS * std::cos(home_lat * M_PI / 180.0));
    down = -(alt - home_alt);
}

void nedToGPS(float north, float east, float down,
             double home_lat, double home_lon, float home_alt,
             double& lat, double& lon, float& alt) {
    const double EARTH_RADIUS = 6378137.0; // meters
    
    double dlat = north / EARTH_RADIUS;
    double dlon = east / (EARTH_RADIUS * std::cos(home_lat * M_PI / 180.0));
    
    lat = home_lat + (dlat * 180.0 / M_PI);
    lon = home_lon + (dlon * 180.0 / M_PI);
    alt = home_alt - down;
}

float gpsDistance(double lat1, double lon1, double lat2, double lon2) {
    const double EARTH_RADIUS = 6378137.0; // meters
    
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon/2) * std::sin(dlon/2);
    
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    
    return static_cast<float>(EARTH_RADIUS * c);
}

uint64_t getTimestampUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
        system_clock::now().time_since_epoch()
    ).count();
}

} // namespace Utils

} // namespace DroneSwarm
