/**
 * \file feedback_listener.cpp
 * \mainpage
 *    Contains the implementation of the FeedbackListener class
 * \author
 *    Implementation based on the feedback listener plan
*/

#include "myactuator_rmd/feedback_listener.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/protocol/command_type.hpp"
#include "myactuator_rmd/protocol/responses.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/driver/can_driver.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/exceptions.hpp"

namespace myactuator_rmd {

  FeedbackListener::FeedbackListener(Driver& driver, std::uint32_t actuator_id)
    : driver_{driver}, actuator_id_{actuator_id}, running_{false} {
    // Add the actuator ID to the driver to configure appropriate CAN filters
    driver_.addId(actuator_id);
  }

  FeedbackListener::~FeedbackListener() {
    stop();
  }

  void FeedbackListener::registerFeedbackCallback(FeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    feedback_callbacks_.push_back(callback);
  }

  void FeedbackListener::registerStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    status_callbacks_.push_back(callback);
  }

  void FeedbackListener::start() {
    if (running_) {
      return;  // Already running
    }
    
    running_ = true;
    listener_thread_ = std::thread(&FeedbackListener::processMessages, this);
  }

  void FeedbackListener::stop() {
    running_ = false;
    
    if (listener_thread_.joinable()) {
      // Add a timeout for joining the thread to avoid hanging
      auto future = std::async(std::launch::async, [this]() {
        if (listener_thread_.joinable()) {
          listener_thread_.join();
        }
      });
      
      // Wait for join with timeout
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
        std::cerr << "Warning: FeedbackListener thread join timed out after 2 seconds\n";
        // If timeout occurs, detach the thread instead of forcing termination
        if (listener_thread_.joinable()) {
          listener_thread_.detach();
        }
      }
    }
  }

  std::uint32_t FeedbackListener::getReplyId() const {
    // Return the CAN ID for replies from the motor
    // Typically this would be the motor's receive ID + offset
    return 0x141; // Example for motor ID 1 with offset 0x140
  }

  void FeedbackListener::setActiveReplyEnabled(bool enabled) {
    active_reply_enabled_ = enabled;
  }

  void FeedbackListener::processFrame(const can::Frame& frame) {
    // Process the frame directly without going through the CAN driver
    // This allows us to process frames received from active reply
    
    std::array<std::uint8_t, 8> data = frame.getData();
    
    // Determine message type from the first byte (command type)
    CommandType cmd_type = static_cast<CommandType>(data[0]);
    
    // Store callbacks locally to avoid locking during processing
    std::vector<FeedbackCallback> feedback_cbs;
    std::vector<StatusCallback> status_cbs;
    
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      feedback_cbs = feedback_callbacks_;
      status_cbs = status_callbacks_;
    }
    
    // Process based on command type
    if (cmd_type == CommandType::READ_MOTOR_STATUS_2 && !status_cbs.empty()) {
      // This is a MotorStatus2 frame - create and pass to callbacks
      MotorStatus2 status(data);
      
      for (const auto& cb : status_cbs) {
        if (cb) {
          cb(status);
        }
      }
      
      // Also convert to Feedback for feedback callbacks
      if (!feedback_cbs.empty()) {
        Feedback feedback;
        feedback.temperature = status.getTemperature();
        feedback.current = status.getTorque();
        feedback.speed = status.getVelocity();
        feedback.encoder = status.getEncoder();
        
        for (const auto& cb : feedback_cbs) {
          if (cb) {
            cb(feedback);
          }
        }
      }
    }
    else {
      // For other response types, create a generic Feedback object
      // Different command responses have different data formats,
      // so we'll try to extract common fields when possible
      
      // Only process if we have callbacks registered
      if (!feedback_cbs.empty()) {
        Feedback feedback;
        
        // Try to extract common fields, assuming a standard format
        // Note: This is a generic approach and may not work for all command types
        feedback.temperature = static_cast<int>(data[1]);
        feedback.current = static_cast<float>(static_cast<std::int16_t>((data[3] << 8) | data[2])) * 0.01f;
        feedback.speed = static_cast<float>(static_cast<std::int16_t>((data[5] << 8) | data[4]));
        feedback.encoder = static_cast<float>(static_cast<std::int16_t>((data[7] << 8) | data[6]));
        
        for (const auto& cb : feedback_cbs) {
          if (cb) {
            cb(feedback);
          }
        }
      }
    }
  }

  void FeedbackListener::processMessages() {
    while (running_) {
      try {
        // Just sleep to limit CPU usage since we now process frames directly via processFrame
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Store callbacks locally to avoid locking during processing
        std::vector<FeedbackCallback> feedback_cbs;
        std::vector<StatusCallback> status_cbs;
        
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          feedback_cbs = feedback_callbacks_;
          status_cbs = status_callbacks_;
        }
        
        // The loop continues but doesn't try to read frames directly
        // Now frames are processed through processFrame method
        
      } catch (const std::exception& e) {
        std::cerr << "Error in feedback listener: " << e.what() << std::endl;
      }
    }
  }

} 
