#!/usr/bin/env python3
"""
Example showing how to use Active Reply with FeedbackListener from Python
"""

import time
import sys
import signal
import threading
import traceback

# Import the compiled Python bindings
import myactuator_rmd_py as rmd

# Flag to control execution
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C by stopping loop"""
    global running
    print("\nStopping...")
    running = False

# Example callback function for feedback data
def on_feedback(feedback):
    print(f"Position: {feedback.position:.2f} deg, "
          f"Velocity: {feedback.velocity:.2f} dps, "
          f"Torque: {feedback.torque:.2f} A, "
          f"Temperature: {feedback.temperature} C")

# Example callback function for status data
def on_status(status):
    print(f"Status: Temperature: {status.temperature} C, "
          f"Voltage: {status.voltage:.1f} V, "
          f"Error Code: {status.error_code}")

def main():
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <CAN interface name>")
        print(f"Example: {sys.argv[0]} can0")
        return 1

    interface_name = sys.argv[1]
    print(f"Using CAN interface: {interface_name}")

    driver = None
    actuator = None
    listener = None
    
    try:
        # Set up CAN driver
        driver = rmd.CanDriver(interface_name)
        print("Driver initialized")
        
        # Create actuator interface for motor with ID 1
        actuator = rmd.ActuatorInterface(driver, 1)
        print("Actuator interface created")

        # Get initial motor status
        status = actuator.getMotorStatus2()
        print(f"Initial motor status: Temperature={status.temperature}°C, "
              f"Current={status.current:.2f}A, "
              f"Speed={status.shaft_speed:.2f}dps, "
              f"Position={status.shaft_angle:.2f}°")

        # Create feedback listener
        listener = rmd.FeedbackListener(driver, 1)
        print("Feedback listener created")

        # Register callbacks
        print("Registering callbacks...")
        listener.registerFeedbackCallback(on_feedback)
        listener.registerStatusCallback(on_status)

        # Register the feedback listener with the actuator interface
        print("Registering listener with actuator...")
        actuator.registerFeedbackListener(listener)
        
        # Start the listener thread
        print("Starting listener...")
        listener.start()

        # Enable active reply at 10Hz
        print("Enabling active reply...")
        actuator.configureActiveReply(True, 10)
        print("Active reply enabled")

        # Move the motor using different control modes
        print("Moving to absolute position 180 degrees...")
        actuator.sendPositionAbsoluteSetpoint(180.0, 500.0)
        print("Waiting for 2 seconds...")
        time.sleep(2)

        if not running:
            return 0

        print("Moving to single-turn position 90 degrees clockwise...")
        actuator.sendSingleTurnPositionSetpoint(90.0, 0, 500.0)
        print("Waiting for 2 seconds...")
        time.sleep(2)

        if not running:
            return 0

        print("Moving by incremental position +45 degrees...")
        actuator.sendIncrementalPositionSetpoint(45.0, 500.0)
        print("Waiting for 2 seconds...")
        time.sleep(2)

        print("Example completed successfully!")
        return 0
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
        return 1
    finally:
        # Clean up resources
        print("Starting cleanup...")
        
        if actuator:
            try:
                print("Disabling active reply...")
                actuator.configureActiveReply(False)
                print("Active reply disabled")
            except Exception as e:
                print(f"Error disabling active reply: {e}")
                traceback.print_exc()

        if listener:
            try:
                print("Stopping listener...")
                listener.stop()
                print("Listener stopped")
            except Exception as e:
                print(f"Error stopping listener: {e}")
                traceback.print_exc()

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"Unhandled exception: {e}")
        traceback.print_exc()
        sys.exit(1) 
