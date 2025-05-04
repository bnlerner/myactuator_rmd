/**
 * \file feedback_listener.hpp
 * \mainpage
 *    Contains a class for listening to asynchronous feedback from the actuator
 * \author
 *    Implementation based on the feedback listener plan
*/

#ifndef MYACTUATOR_RMD__FEEDBACK_LISTENER
#define MYACTUATOR_RMD__FEEDBACK_LISTENER
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/can/frame.hpp"

namespace myactuator_rmd {

  /**\class FeedbackListener
   * \brief
   *    Listens for unsolicited CAN messages from the motor when Active Reply is enabled
  */
  class FeedbackListener {
    public:
      using FeedbackCallback = std::function<void(const Feedback&)>;
      using StatusCallback = std::function<void(const MotorStatus1&)>;
      
      /**\fn FeedbackListener
       * \brief
       *    Class constructor
       * 
       * \param[in] driver
       *    The driver communicating over the network interface
       * \param[in] actuator_id
       *    The actuator id [1, 32]
      */
      FeedbackListener(Driver& driver, std::uint32_t actuator_id);
      
      /**\fn ~FeedbackListener
       * \brief
       *    Class destructor, stops the listener thread if running
      */
      ~FeedbackListener();
      
      /**\fn registerFeedbackCallback
       * \brief
       *    Register a callback for feedback data (position, velocity, torque)
       * 
       * \param[in] callback
       *    The callback function to invoke when feedback is received
      */
      void registerFeedbackCallback(FeedbackCallback callback);
      
      /**\fn registerStatusCallback
       * \brief
       *    Register a callback for motor status data (temperature, voltage, error codes)
       * 
       * \param[in] callback
       *    The callback function to invoke when status is received
      */
      void registerStatusCallback(StatusCallback callback);
      
      /**\fn start
       * \brief
       *    Start the listener thread
      */
      void start();
      
      /**\fn stop
       * \brief
       *    Stop the listener thread
      */
      void stop();
      
      /**\fn processFrame
       * \brief
       *    Process a CAN frame that has been received from the actuator
       *    This method is public so it can be called directly from elsewhere
       *    in the code to feed in frames from Active Reply
       * 
       * \param[in] frame
       *    The CAN frame to process
      */
      void processFrame(const can::Frame& frame);
      
      /**\fn getReplyId
       * \brief
       *    Get the CAN ID that the actuator will use for active reply messages
       * 
       * \return
       *    The CAN ID for active reply messages
      */
      std::uint32_t getReplyId() const;
      
      /**\fn setActiveReplyEnabled
       * \brief
       *    Sets the active reply status.
       * 
       * \param[in] enabled
       *    Whether active reply is enabled or not
      */
      void setActiveReplyEnabled(bool enabled);
      
    private:
      Driver& driver_;
      std::uint32_t actuator_id_;
      std::atomic<bool> running_{false};
      std::thread listener_thread_;
      std::mutex callback_mutex_;
      std::vector<FeedbackCallback> feedback_callbacks_;
      std::vector<StatusCallback> status_callbacks_;
      bool active_reply_enabled_{false};
      
      /**\fn processMessages
       * \brief
       *    Thread function that processes incoming messages
      */
      void processMessages();
  };

}

#endif // MYACTUATOR_RMD__FEEDBACK_LISTENER 
