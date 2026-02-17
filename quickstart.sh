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
#!/bin/bash

# Quick Start Script for Drone Swarm Management System

echo "======================================================"
echo "  DRONE SWARM MANAGEMENT SYSTEM - QUICK START"
echo "======================================================"
echo ""

# Check Python version
echo "Checking Python version..."
python_version=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $python_version"
echo ""

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    echo "Virtual environment created."
else
    echo "Virtual environment already exists."
fi
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate
echo "Virtual environment activated."
echo ""

# Install dependencies
echo "Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt
echo "Dependencies installed."
echo ""

# Create necessary directories
echo "Creating required directories..."
mkdir -p logs
mkdir -p config
mkdir -p simulation
echo "Directories created."
echo ""

# Check if C++ build is needed
read -p "Do you want to build C++ components? (y/n) " build_cpp
if [ "$build_cpp" = "y" ]; then
    echo ""
    echo "Building C++ components..."
    mkdir -p build
    cd build
    cmake ..
    make
    cd ..
    echo "C++ components built successfully."
    echo ""
fi

# Create default config
if [ ! -f "config/swarm_config.json" ]; then
    echo "Creating default configuration..."
    cat > config/swarm_config.json << EOF
{
  "swarm": {
    "encryption_key": "default_swarm_key_change_in_production",
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
    "min_spacing": 5.0,
    "takeoff_altitude": 10.0
  },
  "communication": {
    "multicast_group": "224.0.0.251",
    "base_port": 5000
  }
}
EOF
    echo "Configuration file created."
else
    echo "Configuration file already exists."
fi
echo ""

# Run system check
echo "Running system check..."
python3 -c "
import sys
try:
    from PyQt5 import QtWidgets
    print('✓ PyQt5 installed')
except ImportError:
    print('✗ PyQt5 not found')
    sys.exit(1)

try:
    from cryptography.fernet import Fernet
    print('✓ Cryptography installed')
except ImportError:
    print('✗ Cryptography not found')
    sys.exit(1)

try:
    import numpy as np
    print('✓ NumPy installed')
except ImportError:
    print('✗ NumPy not found')
    sys.exit(1)

print('✓ All dependencies verified')
"

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================================"
    echo "  SETUP COMPLETE!"
    echo "======================================================"
    echo ""
    echo "To start the system:"
    echo "  1. Activate virtual environment: source venv/bin/activate"
    echo "  2. Run the application: python main.py"
    echo ""
    echo "For simulation testing:"
    echo "  python main.py"
    echo ""
    echo "For real drone deployment:"
    echo "  1. Update drone connections in main.py"
    echo "  2. Configure MAVLink in config/swarm_config.json"
    echo "  3. Run: python main.py"
    echo ""
    
    # Ask if user wants to start now
    read -p "Would you like to start the system now? (y/n) " start_now
    if [ "$start_now" = "y" ]; then
        echo ""
        echo "Starting Drone Swarm Management System..."
        echo ""
        python main.py
    fi
else
    echo ""
    echo "Setup failed. Please check the errors above."
    exit 1
fi