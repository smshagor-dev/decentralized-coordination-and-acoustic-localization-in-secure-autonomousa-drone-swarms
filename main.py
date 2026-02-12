"""
Main Application Entry Point
Drone Swarm Management System
"""

import sys
import os
import logging
from datetime import datetime

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'python'))

from drone import Drone, Position
from swarm_manager import SwarmManager
from communication import CommunicationManager
from ml_system import MLDecisionSupport, FormationController
from gui import start_gui

def setup_logging():
    """Setup logging configuration"""
    os.makedirs('logs', exist_ok=True)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(f'logs/system_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
            logging.StreamHandler()
        ]
    )

def create_demo_swarm():
    """Create a demonstration swarm"""
    swarm = SwarmManager()
    
    # Create 5 drones in a realistic, meter-based local world frame
    # (x=east meters, y=north meters). Spread remains within 10 km range.
    drone_positions = [
        Position(0, 0, 0),          # Drone 1
        Position(1200, 300, 0),     # Drone 2
        Position(-1400, -500, 0),   # Drone 3
        Position(800, -1500, 0),    # Drone 4
        Position(-900, 1600, 0)     # Drone 5
    ]
    
    for i, pos in enumerate(drone_positions, start=1):
        drone = Drone(i, pos)
        swarm.add_drone(drone)
    
    return swarm

def main():
    """Main application entry"""
    print("=" * 60)
    print("DRONE SWARM MANAGEMENT SYSTEM")
    print("Secure and Fault-Tolerant Multi-Drone System")
    print("=" * 60)
    print()
    
    # Setup logging
    setup_logging()
    logger = logging.getLogger("Main")
    
    logger.info("Starting Drone Swarm Management System")
    
    try:
        # Create swarm manager
        swarm_manager = create_demo_swarm()
        
        # Start swarm operations
        swarm_manager.start()
        
        logger.info(f"Swarm created with {len(swarm_manager.drones)} drones")
        logger.info(f"Leader elected: Drone {swarm_manager.leader_id}")
        
        # Start GUI
        logger.info("Starting GUI...")
        start_gui(swarm_manager)
        
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        # Cleanup
        if 'swarm_manager' in locals():
            swarm_manager.stop()
        logger.info("System shutdown complete")

if __name__ == "__main__":
    main()
