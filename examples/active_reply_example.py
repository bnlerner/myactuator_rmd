#!/usr/bin/env python3
#
# Example showing how to use Active Reply with FeedbackListener
# This demonstrates receiving motor status updates through Active Reply
#

import sys
import time
import signal
import myactuator_rmd_py as rmd

def feedback_callback(feedback):
    print(f"Feedback: Temp={feedback.temperature}°C, Current={feedback.current}A, "
          f"Speed={feedback.speed}rpm, Position={feedback.encoder}°")

def status_callback(status):
    print(f"Status: Temp={status.getTemperature()}°C, Torque={status.getTorque()}A, "
          f"Speed={status.getVelocity()}rpm, Position={status.getEncoder()}°")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <can_interface>")
        print("Example: python3 active_reply_example.py can0")
        return 1
    
    interface_name = sys.argv[1]
    
    # Create the CAN driver and actuator interface
    driver = rmd.CanDriver(interface_name)
    actuator = rmd.ActuatorInterface(driver, 1)  # using actuator with ID 1
    
    # Get and display initial motor status
    print("Initial motor status:")
    status = actuator.getMotorStatus2()
    print(f"Temperature: {status.getTemperature()}°C")
    print(f"Current: {status.getTorque()}A")
    print(f"Speed: {status.getVelocity()}rpm")
    print(f"Position: {status.getEncoder()}°")
    
    # Create and start the feedback listener
    listener = rmd.FeedbackListener(driver, 1)
    
    # Register the feedback listener with the actuator interface
    actuator.registerFeedbackListener(listener)
    
    # Register callbacks for both feedback and status data
    listener.registerFeedbackCallback(feedback_callback)
    listener.registerStatusCallback(status_callback)
    
    # Start the listener
    listener.start()
    
    # Enable active reply with a frequency of 10Hz
    print("Enabling Active Reply at 10Hz...")
    actuator.configureActiveReply(True, 10)
    
    # Setup a clean exit handler
    def handle_exit(signum, frame):
        print("\nDisabling Active Reply and stopping listener...")
        actuator.configureActiveReply(False, 0)
        listener.stop()
        print("Exiting...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, handle_exit)
    
    # Move the motor to different positions
    try:
        print("Moving motor to different positions...")
        for target in range(0, 361, 90):
            print(f"Moving to position {target}°...")
            actuator.sendPositionAbsoluteSetpoint(target, 30)  # position in degrees, max speed 30rpm
            time.sleep(3)  # Wait for the motor to reach the position
    
    finally:
        # Ensure Active Reply is disabled and listener is stopped
        print("Disabling Active Reply and stopping listener...")
        actuator.configureActiveReply(False, 0)
        listener.stop()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 
