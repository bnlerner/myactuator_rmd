#!/usr/bin/env python3
"""
Simple script to monitor the CAN bus for messages from the MyActuator RMD motor
"""

import time
import sys
import signal
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
    
    try:
        # Set up CAN driver
        driver = rmd.CanDriver(interface_name)
        print("Driver initialized")
        
        # Create actuator interface for motor with ID 1
        actuator = rmd.ActuatorInterface(driver, 1)
        print("Actuator interface created")

        # Get the current motor status to verify communication
        print("Getting initial motor status...")
        status = actuator.getMotorStatus2()
        print(f"Initial Motor Status: Temperature: {status.temperature}°C, " 
              f"Current: {status.current:.2f}A, "
              f"Shaft Speed: {status.shaft_speed:.2f} dps, "
              f"Shaft Angle: {status.shaft_angle:.2f}°")

        # Enable active reply at 10Hz
        print("\nEnabling active reply...")
        actuator.configureActiveReply(True, 10)
        print("Active reply enabled")

        # Create a feedback listener
        print("\nCreating and starting feedback listener...")
        listener = rmd.FeedbackListener(driver, 1)
        
        def feedback_callback(feedback):
            print(f"\nFeedback: Position: {feedback.position:.2f}°, "
                  f"Velocity: {feedback.velocity:.2f} dps, "
                  f"Torque: {feedback.torque:.2f}A, "
                  f"Temperature: {feedback.temperature}°C")
        
        def status_callback(status):
            print(f"\nStatus: Temperature: {status.temperature}°C, "
                  f"Voltage: {status.voltage:.1f}V, "
                  f"Error Code: {status.error_code}")
        
        listener.registerFeedbackCallback(feedback_callback)
        listener.registerStatusCallback(status_callback)
        listener.start()

        # Function to directly query motor status and print it
        def get_and_print_status():
            try:
                # Get current motor status
                status = actuator.getMotorStatus2()
                print(f"\nDirect query - Motor Status: Temperature: {status.temperature}°C, "
                      f"Current: {status.current:.2f}A, "
                      f"Shaft Speed: {status.shaft_speed:.2f} dps, "
                      f"Shaft Angle: {status.shaft_angle:.2f}°")
                
                return True
            except Exception as e:
                print(f"Error getting motor status: {e}")
                return False

        # Wait for feedback messages
        print("\nMonitoring for 20 seconds (or Ctrl+C to stop)...")
        print("You should see both callback messages from active reply AND direct queries")
        start_time = time.time()
        last_status_time = 0
        
        while running and (time.time() - start_time) < 20:
            # Query status every 2 seconds as a fallback
            if time.time() - last_status_time > 2.0:
                if get_and_print_status():
                    last_status_time = time.time()
            
            # Sleep a bit to reduce CPU usage
            time.sleep(0.1)
            
        print("\nMonitoring complete")
        
        # Stop the listener
        print("Stopping listener...")
        listener.stop()
        print("Listener stopped")
        
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

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"Unhandled exception: {e}")
        traceback.print_exc()
        sys.exit(1) 
