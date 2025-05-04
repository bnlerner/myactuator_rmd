/**
 * \file active_reply_example.cpp
 * \mainpage
 *    Example showing how to use Active Reply with FeedbackListener
 * \author
 *    Implementation based on the feedback listener plan
*/

#include <chrono>
#include <iostream>
#include <thread>

#include "myactuator_rmd/can/node.hpp"
#include "myactuator_rmd/driver/can_driver.hpp"
#include "myactuator_rmd/actuator_constants.hpp"
#include "myactuator_rmd/actuator_interface.hpp"
#include "myactuator_rmd/feedback_listener.hpp"

// Example callback function for feedback data
void onFeedback(const myactuator_rmd::Feedback& feedback) {
  std::cout << "Position: " << feedback.position << " deg, "
            << "Velocity: " << feedback.velocity << " dps, "
            << "Torque: " << feedback.torque << " A, "
            << "Temperature: " << feedback.temperature << " C" << std::endl;
}

// Example callback function for status data
void onStatus(const myactuator_rmd::MotorStatus1& status) {
  std::cout << "Status: Temperature: " << status.temperature << " C, "
            << "Voltage: " << status.voltage << " V, "
            << "Error Code: " << static_cast<int>(status.error_code) << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <CAN interface name>" << std::endl;
    std::cerr << "Example: " << argv[0] << " can0" << std::endl;
    return 1;
  }

  std::string const interface_name{argv[1]};
  std::cout << "Using CAN interface: " << interface_name << std::endl;

  try {
    // Set up CAN driver
    myactuator_rmd::CanDriver driver{interface_name};
    
    // Create actuator interface for motor with ID 1
    myactuator_rmd::ActuatorInterface actuator{driver, 1};

    // Create feedback listener
    myactuator_rmd::FeedbackListener listener{driver, 1};

    // Register callbacks
    listener.registerFeedbackCallback(onFeedback);
    listener.registerStatusCallback(onStatus);

    // Start the listener thread
    listener.start();

    // Enable active reply at 10Hz
    std::cout << "Enabling active reply..." << std::endl;
    actuator.configureActiveReply(true, 10);

    // Move the motor using different control modes
    std::cout << "Moving to absolute position 180 degrees..." << std::endl;
    actuator.sendPositionAbsoluteSetpoint(180.0f, 100.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving to single-turn position 90 degrees clockwise..." << std::endl;
    actuator.sendSingleTurnPositionSetpoint(90.0f, 0, 100.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Moving by incremental position +45 degrees..." << std::endl;
    actuator.sendIncrementalPositionSetpoint(45.0f, 100.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Disable active reply
    std::cout << "Disabling active reply..." << std::endl;
    actuator.configureActiveReply(false);

    // Stop the listener thread
    listener.stop();

    std::cout << "Example completed successfully!" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
} 
