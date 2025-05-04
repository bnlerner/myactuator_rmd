#!/usr/bin/env python3
"""
Simple test to verify FeedbackListener callback registration works correctly
"""

import myactuator_rmd_py as rmd
import time
import sys

def on_feedback(feedback):
    print(f"Feedback received: {feedback}")

def on_status(status):
    print(f"Status received: {status}")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <CAN interface name>")
        print(f"Example: {sys.argv[0]} can0")
        return 1

    interface_name = sys.argv[1]
    print(f"Using CAN interface: {interface_name}")

    try:
        # Set up CAN driver
        driver = rmd.CanDriver(interface_name)
        
        # Create actuator interface for motor with ID 1
        actuator = rmd.ActuatorInterface(driver, 1)

        # Create feedback listener
        listener = rmd.FeedbackListener(driver, 1)

        # Verify callback registration works without errors
        print("Registering feedback callback...")
        listener.registerFeedbackCallback(on_feedback)
        
        print("Registering status callback...")
        listener.registerStatusCallback(on_status)
        
        print("Starting listener...")
        listener.start()
        
        print("Enabling active reply at 10Hz...")
        actuator.configureActiveReply(True, 10)
        
        print("Waiting for messages for 5 seconds...")
        time.sleep(5)
        
        print("Disabling active reply...")
        actuator.configureActiveReply(False)
        
        print("Stopping listener...")
        listener.stop()
        
        print("Test completed successfully!")
        return 0
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 
