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
#include <array>
#include <iomanip>
#include <numeric>
#include <fstream>
#include <unordered_map>
#include <cctype>
#include <cstdlib>

namespace DroneSwarm {
namespace {
constexpr size_t kMotorCount = 4;
constexpr size_t kRollingWindow = 20;
constexpr float kDegradedDropThreshold = 0.10f; // 10%
constexpr float kThrustToCurrentScale = 0.45f;

int oppositeMotorIndex(int idx) {
    return (idx + 2) % static_cast<int>(kMotorCount);
}

float clampNonNegative(float value) {
    return std::max(0.0f, value);
}

std::string trim(const std::string& value) {
    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
        start++;
    }
    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        end--;
    }
    return value.substr(start, end - start);
}

std::string normalizeKey(std::string key) {
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return key;
}

bool parseBool(const std::string& raw, bool default_value) {
    const std::string normalized = normalizeKey(trim(raw));
    if (normalized.empty()) {
        return default_value;
    }
    if (normalized == "1" || normalized == "TRUE" || normalized == "YES" || normalized == "ON") {
        return true;
    }
    if (normalized == "0" || normalized == "FALSE" || normalized == "NO" || normalized == "OFF") {
        return false;
    }
    return default_value;
}

int parseInt(const std::string& raw, int default_value) {
    try {
        return std::stoi(trim(raw));
    } catch (...) {
        return default_value;
    }
}

std::unordered_map<std::string, std::string> parseDotEnvFile(const std::string& path) {
    std::unordered_map<std::string, std::string> values;
    std::ifstream in(path);
    if (!in.is_open()) {
        return values;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos || eq == 0) {
            continue;
        }
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (val.size() >= 2) {
            const char first = val.front();
            const char last = val.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
                val = val.substr(1, val.size() - 2);
            }
        }
        if (!key.empty()) {
            values[normalizeKey(key)] = val;
        }
    }
    return values;
}

std::string readSetting(
    const std::unordered_map<std::string, std::string>& dotenv_values,
    const std::string& env_key,
    const std::string& default_value
) {
    const std::string key = normalizeKey(env_key);
    const char* env_val = std::getenv(key.c_str());
    if (env_val && *env_val) {
        return std::string(env_val);
    }
    auto it = dotenv_values.find(key);
    if (it != dotenv_values.end() && !it->second.empty()) {
        return it->second;
    }
    return default_value;
}

struct SensorEnvDescriptor {
    const char* sensor_name;
    const char* enabled_key;
    const char* connection_key;
    const char* rate_key;
    bool default_enabled;
    const char* default_connection;
    int default_rate_hz;
};

const std::array<SensorEnvDescriptor, 7> kSensorDescriptors = {{
    {"motor_rpm", "SENSOR_MOTOR_RPM_ENABLED", "SENSOR_MOTOR_RPM_CONNECTION", "SENSOR_MOTOR_RPM_RATE_HZ", true, "mavlink://esc_telemetry", 50},
    {"battery", "SENSOR_BATTERY_ENABLED", "SENSOR_BATTERY_CONNECTION", "SENSOR_BATTERY_RATE_HZ", true, "mavlink://battery_status", 10},
    {"gps", "SENSOR_GPS_ENABLED", "SENSOR_GPS_CONNECTION", "SENSOR_GPS_RATE_HZ", true, "mavlink://gps_raw_int", 10},
    {"imu", "SENSOR_IMU_ENABLED", "SENSOR_IMU_CONNECTION", "SENSOR_IMU_RATE_HZ", true, "mavlink://highres_imu", 100},
    {"barometer", "SENSOR_BAROMETER_ENABLED", "SENSOR_BAROMETER_CONNECTION", "SENSOR_BAROMETER_RATE_HZ", true, "mavlink://scaled_pressure", 25},
    {"magnetometer", "SENSOR_MAGNETOMETER_ENABLED", "SENSOR_MAGNETOMETER_CONNECTION", "SENSOR_MAGNETOMETER_RATE_HZ", true, "mavlink://raw_imu.mag", 25},
    {"acoustic", "SENSOR_ACOUSTIC_ENABLED", "SENSOR_ACOUSTIC_CONNECTION", "SENSOR_ACOUSTIC_RATE_HZ", false, "udp://0.0.0.0:16060", 48000},
}};
} // namespace

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
    std::array<std::deque<float>, kMotorCount> rpm_history;
    std::array<float, kMotorCount> motor_vibration;
    std::array<float, kMotorCount> motor_thrust;
    std::array<float, kMotorCount> thrust_lpf;
    float battery_drain_rate;
    float total_thrust;
    float baseline_total_thrust;
    bool self_healing_active;
    bool emergency_return_mode;
    bool swarm_alert_pending;
    std::string swarm_alert_message;
    float filtered_roll_comp;
    float filtered_yaw_comp;
    float pid_roll_p;
    float pid_roll_i;
    float pid_roll_d;
    float pid_yaw_p;
    float pid_yaw_i;
    float pid_yaw_d;
    std::chrono::steady_clock::time_point last_battery_sample;
    float last_battery_remaining;
    std::unordered_map<std::string, SensorConnectionConfig> sensor_connections;
    std::mutex sensor_config_mutex;
    
    Impl(const std::string& conn_str, int id)
        : connection_string(conn_str), drone_id(id), connected(false),
          armed(false), current_mode(FlightMode::MANUAL), running(false),
          battery_drain_rate(0.0f), total_thrust(0.0f), baseline_total_thrust(10.0f),
          self_healing_active(false), emergency_return_mode(false),
          swarm_alert_pending(false),
          filtered_roll_comp(0.0f), filtered_yaw_comp(0.0f),
          pid_roll_p(1.0f), pid_roll_i(0.0f), pid_roll_d(0.05f),
          pid_yaw_p(0.8f), pid_yaw_i(0.0f), pid_yaw_d(0.04f),
          last_battery_sample(std::chrono::steady_clock::now()),
          last_battery_remaining(100.0f), last_heartbeat(0) {
        motor_vibration.fill(0.0f);
        motor_thrust.fill(0.0f);
        thrust_lpf.fill(0.0f);
    }
    
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
    loadSensorConnectionsFromEnv(".env");
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

    loadSensorConnectionsFromEnv(".env");
    
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
    pImpl->telemetry.motors[motor_id].degraded = true;
    pImpl->telemetry.motors[motor_id].rpm = 0;
    pImpl->telemetry.motors[motor_id].vibration = 1.0f;
    pImpl->motor_vibration[motor_id] = 1.0f;
    pImpl->rpm_history[motor_id].push_back(0.0f);
    if (pImpl->rpm_history[motor_id].size() > kRollingWindow) {
        pImpl->rpm_history[motor_id].pop_front();
    }
    
    detectMotorHealth();
    
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

bool DroneController::loadSensorConnectionsFromEnv(const std::string& dotenv_path) {
    const auto dotenv_values = parseDotEnvFile(dotenv_path);
    std::unordered_map<std::string, SensorConnectionConfig> loaded;

    for (const auto& descriptor : kSensorDescriptors) {
        const std::string enabled_text = readSetting(
            dotenv_values,
            descriptor.enabled_key,
            descriptor.default_enabled ? "1" : "0"
        );
        const std::string connection_text = readSetting(
            dotenv_values,
            descriptor.connection_key,
            descriptor.default_connection
        );
        const std::string rate_text = readSetting(
            dotenv_values,
            descriptor.rate_key,
            std::to_string(descriptor.default_rate_hz)
        );

        SensorConnectionConfig cfg;
        cfg.sensor_name = descriptor.sensor_name;
        cfg.enabled = parseBool(enabled_text, descriptor.default_enabled);
        cfg.connection_uri = trim(connection_text);
        cfg.update_rate_hz = std::max(1, parseInt(rate_text, descriptor.default_rate_hz));
        loaded[cfg.sensor_name] = cfg;
    }

    {
        std::lock_guard<std::mutex> lock(pImpl->sensor_config_mutex);
        pImpl->sensor_connections = loaded;
    }

    std::cout << "[SENSOR-CONFIG] Drone " << pImpl->drone_id
              << " loaded " << loaded.size() << " sensor configs"
              << " from " << dotenv_path << std::endl;
    for (const auto& it : loaded) {
        const auto& cfg = it.second;
        std::cout << "  - " << cfg.sensor_name
                  << " enabled=" << (cfg.enabled ? "true" : "false")
                  << " rate_hz=" << cfg.update_rate_hz
                  << " conn=" << cfg.connection_uri
                  << std::endl;
    }
    return !loaded.empty();
}

std::unordered_map<std::string, SensorConnectionConfig> DroneController::getSensorConnections() const {
    std::lock_guard<std::mutex> lock(pImpl->sensor_config_mutex);
    return pImpl->sensor_connections;
}

SensorConnectionConfig DroneController::getSensorConnection(const std::string& sensor_name) const {
    std::lock_guard<std::mutex> lock(pImpl->sensor_config_mutex);
    const std::string key = normalizeKey(sensor_name);
    for (const auto& it : pImpl->sensor_connections) {
        if (normalizeKey(it.first) == key) {
            return it.second;
        }
    }
    SensorConnectionConfig missing;
    missing.sensor_name = sensor_name;
    return missing;
}

// Telemetry loop
void DroneController::telemetryLoop() {
    while (pImpl->running) {
        bool emit_swarm_alert = false;
        std::string swarm_alert_message;
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

        {
            std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
            if (pImpl->swarm_alert_pending) {
                emit_swarm_alert = true;
                swarm_alert_message = pImpl->swarm_alert_message;
                pImpl->swarm_alert_pending = false;
            }
        }

        if (emit_swarm_alert && pImpl->emergency_callback) {
            pImpl->emergency_callback(swarm_alert_message);
        }
        
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

    for (size_t i = 0; i < pImpl->telemetry.motors.size() && i < kMotorCount; i++) {
        auto& motor = pImpl->telemetry.motors[i];
        if (!pImpl->armed) {
            motor.rpm = 0.0f;
            motor.temperature = 25.0f;
            motor.current = 0.0f;
            motor.vibration = 0.0f;
            pImpl->motor_thrust[i] = 0.0f;
            pImpl->thrust_lpf[i] = 0.0f;
            continue;
        }

        const float nominal_rpm = 5000.0f;
        if (motor.operational) {
            if (motor.degraded) {
                motor.rpm = nominal_rpm * 0.85f;
                motor.vibration = 0.65f;
            } else {
                motor.rpm = nominal_rpm;
                motor.vibration = 0.18f;
            }
            pImpl->motor_vibration[i] = motor.vibration;
            motor.temperature = 40.0f + (motor.degraded ? 6.0f : 0.0f);
            motor.current = 5.0f + pImpl->motor_thrust[i] * kThrustToCurrentScale;
        } else {
            motor.rpm = 0.0f;
            motor.degraded = true;
            motor.vibration = 1.0f;
            pImpl->motor_vibration[i] = 1.0f;
            motor.temperature = 42.0f;
            motor.current = 0.0f;
        }

        pImpl->motor_thrust[i] = motor.rpm * 0.0005f;
    }

    pImpl->total_thrust = 0.0f;
    for (float thrust : pImpl->motor_thrust) {
        pImpl->total_thrust += thrust;
    }

    detectMotorHealth();
}

void DroneController::detectMotorHealth() {
    int degraded_count = 0;
    int first_failed_index = -1;

    for (size_t i = 0; i < pImpl->telemetry.motors.size() && i < kMotorCount; i++) {
        auto& motor = pImpl->telemetry.motors[i];

        if (!motor.operational) {
            motor.degraded = true;
            degraded_count++;
            if (first_failed_index < 0) {
                first_failed_index = static_cast<int>(i);
            }
            continue;
        }

        auto& history = pImpl->rpm_history[i];
        float rolling_avg = motor.rpm;
        if (!history.empty()) {
            const float sum = std::accumulate(history.begin(), history.end(), 0.0f);
            rolling_avg = sum / static_cast<float>(history.size());
        }

        history.push_back(motor.rpm);
        if (history.size() > kRollingWindow) {
            history.pop_front();
        }

        if (history.size() >= 5 && rolling_avg > 1.0f) {
            const float drop_pct = ((rolling_avg - motor.rpm) / rolling_avg) * 100.0f;
            if (drop_pct >= (kDegradedDropThreshold * 100.0f)) {
                if (!motor.degraded) {
                    std::cout << std::fixed << std::setprecision(1)
                              << "[IMMUNE] Motor " << i << " degraded | RPM drop: "
                              << drop_pct << "% | Compensation Active" << std::defaultfloat
                              << std::endl;
                }
                motor.degraded = true;
                degraded_count++;
                if (first_failed_index < 0) {
                    first_failed_index = static_cast<int>(i);
                }
            }
        }
    }

    if (degraded_count == 1 && first_failed_index >= 0) {
        activateSelfHealingMode(first_failed_index);
    } else if (degraded_count == 0) {
        pImpl->self_healing_active = false;
        updateAdaptivePID();
    }

    if (degraded_count >= 2) {
        if (!pImpl->emergency_return_mode) {
            pImpl->emergency_return_mode = true;
            pImpl->self_healing_active = false;
            pImpl->current_mode = FlightMode::EMERGENCY_RETURN;
            pImpl->telemetry.flight_mode = FlightMode::EMERGENCY_RETURN;
            pImpl->swarm_alert_pending = true;
            pImpl->swarm_alert_message = "SWARM_ALERT: 2+ motors degraded, EMERGENCY_RETURN enabled";
            std::cout << "[IMMUNE] SWARM_ALERT | 2+ motors degraded | Entering EMERGENCY_RETURN" << std::endl;
        }
    }

    if (pImpl->emergency_return_mode && pImpl->armed) {
        pImpl->telemetry.position.relative_alt = std::max(0.0f, pImpl->telemetry.position.relative_alt - 0.05f);
        pImpl->telemetry.position.altitude = std::max(0.0f, pImpl->telemetry.position.altitude - 0.05f);
    }
}

void DroneController::activateSelfHealingMode(int failed_motor_index) {
    if (failed_motor_index < 0 || failed_motor_index >= static_cast<int>(kMotorCount)) {
        return;
    }
    pImpl->self_healing_active = true;
    redistributeThrust(failed_motor_index);
    updateAdaptivePID();
}

void DroneController::redistributeThrust(int failed_motor_index) {
    if (!pImpl->armed) {
        return;
    }

    const float required_total = std::max(pImpl->baseline_total_thrust, pImpl->total_thrust);
    const float failed_motor_floor = required_total * 0.12f;
    const int opposite = oppositeMotorIndex(failed_motor_index);

    for (size_t i = 0; i < kMotorCount; i++) {
        if (static_cast<int>(i) == failed_motor_index) {
            pImpl->motor_thrust[i] = std::max(failed_motor_floor, pImpl->motor_thrust[i] * 0.45f);
        } else {
            pImpl->motor_thrust[i] = (required_total - pImpl->motor_thrust[failed_motor_index]) / 3.0f;
        }
    }

    pImpl->motor_thrust[opposite] *= 1.08f;

    // Re-normalize to preserve total thrust T = sum(T_i).
    float non_failed_sum = 0.0f;
    for (size_t i = 0; i < kMotorCount; i++) {
        if (static_cast<int>(i) != failed_motor_index) {
            non_failed_sum += pImpl->motor_thrust[i];
        }
    }
    const float target_non_failed_sum = required_total - pImpl->motor_thrust[failed_motor_index];
    const float renorm = non_failed_sum > 1e-3f ? (target_non_failed_sum / non_failed_sum) : 1.0f;
    for (size_t i = 0; i < kMotorCount; i++) {
        if (static_cast<int>(i) != failed_motor_index) {
            pImpl->motor_thrust[i] *= renorm;
        }
    }

    // Low-pass filtered torque compensation to avoid oscillation.
    const float roll_target = (pImpl->motor_thrust[1] + pImpl->motor_thrust[2]) -
                              (pImpl->motor_thrust[0] + pImpl->motor_thrust[3]);
    const float yaw_target = (pImpl->motor_thrust[0] + pImpl->motor_thrust[2]) -
                             (pImpl->motor_thrust[1] + pImpl->motor_thrust[3]);
    constexpr float lpf_alpha = 0.25f;
    pImpl->filtered_roll_comp = lpf_alpha * roll_target + (1.0f - lpf_alpha) * pImpl->filtered_roll_comp;
    pImpl->filtered_yaw_comp = lpf_alpha * yaw_target + (1.0f - lpf_alpha) * pImpl->filtered_yaw_comp;

    pImpl->motor_thrust[0] += 0.03f * pImpl->filtered_yaw_comp - 0.03f * pImpl->filtered_roll_comp;
    pImpl->motor_thrust[1] -= 0.03f * pImpl->filtered_yaw_comp + 0.03f * pImpl->filtered_roll_comp;
    pImpl->motor_thrust[2] += 0.03f * pImpl->filtered_yaw_comp + 0.03f * pImpl->filtered_roll_comp;
    pImpl->motor_thrust[3] -= 0.03f * pImpl->filtered_yaw_comp - 0.03f * pImpl->filtered_roll_comp;

    pImpl->total_thrust = 0.0f;
    for (size_t i = 0; i < kMotorCount; i++) {
        pImpl->motor_thrust[i] = clampNonNegative(pImpl->motor_thrust[i]);
        pImpl->thrust_lpf[i] = lpf_alpha * pImpl->motor_thrust[i] + (1.0f - lpf_alpha) * pImpl->thrust_lpf[i];
        pImpl->motor_thrust[i] = pImpl->thrust_lpf[i];
        pImpl->total_thrust += pImpl->motor_thrust[i];
    }
}

void DroneController::updateAdaptivePID() {
    int degraded_count = 0;
    for (size_t i = 0; i < pImpl->telemetry.motors.size() && i < kMotorCount; i++) {
        if (pImpl->telemetry.motors[i].degraded) {
            degraded_count++;
        }
    }

    const float adaptation = pImpl->self_healing_active ? 1.0f : 0.0f;
    pImpl->pid_roll_p = 1.0f + adaptation * 0.22f + 0.08f * degraded_count;
    pImpl->pid_roll_i = 0.0f;
    pImpl->pid_roll_d = 0.05f + adaptation * 0.02f + 0.01f * degraded_count;
    pImpl->pid_yaw_p = 0.8f + adaptation * 0.18f + 0.06f * degraded_count;
    pImpl->pid_yaw_i = 0.0f;
    pImpl->pid_yaw_d = 0.04f + adaptation * 0.015f + 0.01f * degraded_count;
}

// Update battery status
void DroneController::updateBatteryStatus() {
    std::lock_guard<std::mutex> lock(pImpl->telemetry_mutex);
    
    // In real implementation, read from battery management system
    // For simulation, slowly decrease based on flight state
    
    float discharge_rate = 0.0f;
    if (pImpl->armed) {
        const float thrust_load = std::clamp(pImpl->total_thrust / std::max(1.0f, pImpl->baseline_total_thrust), 0.5f, 2.0f);
        if (pImpl->current_mode == FlightMode::AUTO_TAKEOFF) {
            discharge_rate = 0.02f * thrust_load; // 2% per second baseline
        } else if (pImpl->current_mode == FlightMode::OFFBOARD) {
            discharge_rate = 0.01f * thrust_load; // 1% per second baseline
        } else if (pImpl->current_mode == FlightMode::EMERGENCY_RETURN) {
            discharge_rate = 0.012f * thrust_load;
        } else {
            discharge_rate = 0.005f * thrust_load; // 0.5% per second baseline
        }
    }
    
    // Apply discharge (would be real measurement in actual drone)
    // This is just for simulation
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - pImpl->last_battery_sample).count();
    pImpl->last_battery_sample = now;
    
    pImpl->telemetry.battery.remaining -= discharge_rate * dt;
    pImpl->telemetry.battery.remaining = std::max(0.0f, pImpl->telemetry.battery.remaining);

    if (dt > 1e-3f) {
        pImpl->battery_drain_rate = std::max(0.0f, (pImpl->last_battery_remaining - pImpl->telemetry.battery.remaining) / dt);
    } else {
        pImpl->battery_drain_rate = 0.0f;
    }
    pImpl->last_battery_remaining = pImpl->telemetry.battery.remaining;
    
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
