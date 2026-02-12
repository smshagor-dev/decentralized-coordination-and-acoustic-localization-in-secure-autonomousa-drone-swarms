/**
 * main_test.cpp
 * Test program for C++ DroneController
 */

#include "DroneController.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace DroneSwarm;

void printTelemetry(const Telemetry& telem) {
    std::cout << "\n=== Telemetry ===" << std::endl;
    std::cout << "Position: " << telem.position.latitude << ", " 
              << telem.position.longitude << ", " 
              << telem.position.altitude << "m" << std::endl;
    std::cout << "Battery: " << telem.battery.remaining << "%" << std::endl;
    std::cout << "Armed: " << (telem.armed ? "Yes" : "No") << std::endl;
    std::cout << "Connected: " << (telem.connected ? "Yes" : "No") << std::endl;
    
    std::cout << "Motors: ";
    for (const auto& motor : telem.motors) {
        std::cout << motor.motor_id << ":" 
                  << (motor.operational ? "OK" : "FAIL") << " ";
    }
    std::cout << std::endl;
}

void telemetryCallback(const Telemetry& telem) {
    static int counter = 0;
    if (++counter % 10 == 0) {  // Print every 1 second
        printTelemetry(telem);
    }
}

void emergencyCallback(const std::string& reason) {
    std::cout << "\n!!! EMERGENCY: " << reason << " !!!" << std::endl;
}

int main() {
    std::cout << "=====================================" << std::endl;
    std::cout << "  Drone Controller C++ Test" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << std::endl;
    
    // Create drone controller
    DroneController drone("udp://:14540", 1);
    
    // Set callbacks
    drone.setTelemetryCallback(telemetryCallback);
    drone.setEmergencyCallback(emergencyCallback);
    
    // Connect
    std::cout << "Connecting to drone..." << std::endl;
    if (!drone.connect()) {
        std::cerr << "Failed to connect!" << std::endl;
        return 1;
    }
    
    std::cout << "Connected successfully!" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Get initial telemetry
    Telemetry telem = drone.getTelemetry();
    printTelemetry(telem);
    
    // Test sequence
    std::cout << "\n=== Test Sequence ===" << std::endl;
    
    // 1. Arm
    std::cout << "\n1. Arming..." << std::endl;
    if (drone.arm()) {
        std::cout << "   ✓ Armed successfully" << std::endl;
    } else {
        std::cout << "   ✗ Arm failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 2. Takeoff
    std::cout << "\n2. Taking off to 10m..." << std::endl;
    if (drone.takeoff(10.0f)) {
        std::cout << "   ✓ Takeoff command sent" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 3. Go to position
    std::cout << "\n3. Flying to waypoint..." << std::endl;
    Position target;
    target.latitude = 47.3977;
    target.longitude = 8.5456;
    target.altitude = 15.0f;
    
    if (drone.gotoPosition(target)) {
        std::cout << "   ✓ Waypoint command sent" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 4. Set velocity
    std::cout << "\n4. Setting velocity..." << std::endl;
    if (drone.setVelocityNED(2.0f, 1.0f, 0.0f)) {
        std::cout << "   ✓ Velocity command sent" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 5. Hold position
    std::cout << "\n5. Holding position..." << std::endl;
    if (drone.holdPosition()) {
        std::cout << "   ✓ Position hold enabled" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 6. Simulate motor failure
    std::cout << "\n6. Simulating motor 2 failure..." << std::endl;
    if (drone.disableMotor(2)) {
        std::cout << "   ✓ Motor 2 disabled" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Check motors
    auto motors = drone.getMotorStatus();
    std::cout << "   Motor status: ";
    for (const auto& motor : motors) {
        std::cout << motor.motor_id << ":" 
                  << (motor.operational ? "OK" : "FAIL") << " ";
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 7. Return to launch
    std::cout << "\n7. Returning to launch..." << std::endl;
    if (drone.returnToLaunch()) {
        std::cout << "   ✓ RTL command sent" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 8. Land
    std::cout << "\n8. Landing..." << std::endl;
    if (drone.land()) {
        std::cout << "   ✓ Landing command sent" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 9. Disarm
    std::cout << "\n9. Disarming..." << std::endl;
    if (drone.disarm()) {
        std::cout << "   ✓ Disarmed successfully" << std::endl;
    }
    
    // Final telemetry
    std::cout << "\n=== Final Status ===" << std::endl;
    telem = drone.getTelemetry();
    printTelemetry(telem);
    
    // Disconnect
    std::cout << "\nDisconnecting..." << std::endl;
    drone.disconnect();
    
    std::cout << "\n=====================================" << std::endl;
    std::cout << "  Test Complete" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    return 0;
}