#include "myactuator_rmd/actuator_interface.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/control_mode.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/feedback_listener.hpp"
#include "myactuator_rmd/protocol/requests.hpp"
#include "myactuator_rmd/protocol/responses.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  ActuatorInterface::ActuatorInterface(Driver& driver, std::uint32_t const actuator_id)
  : driver_{driver}, actuator_id_{actuator_id} {
    driver.addId(actuator_id); // Make the actuator listen to the responses
    return;
  }

  void ActuatorInterface::registerFeedbackListener(FeedbackListener* listener) {
    feedback_listener_ = listener;
  }

  std::int32_t ActuatorInterface::getAcceleration() {
    GetAccelerationRequest const request {};
    GetAccelerationResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getAcceleration();
  }

  std::uint16_t ActuatorInterface::getCanId() {
    GetCanIdRequest const request {};
    GetCanIdResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getCanId();
  }

  Gains ActuatorInterface::getControllerGains() {
    GetControllerGainsRequest const request {};
    GetControllerGainsResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getGains();
  }

  ControlMode ActuatorInterface::getControlMode() {
    GetControlModeRequest const request {};
    GetControlModeResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getMode();
  }

  std::string ActuatorInterface::getMotorModel() {
    GetMotorModelRequest const request {};
    GetMotorModelResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getModel();
  }

  float ActuatorInterface::getMotorPower() {
    GetMotorPowerRequest const request {};
    GetMotorPowerResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getPower();
  }

  MotorStatus1 ActuatorInterface::getMotorStatus1() {
    GetMotorStatus1Request const request {};
    GetMotorStatus1Response const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  MotorStatus2 ActuatorInterface::getMotorStatus2() {
    GetMotorStatus2Request const request {};
    GetMotorStatus2Response const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  MotorStatus3 ActuatorInterface::getMotorStatus3() {
    GetMotorStatus3Request const request {};
    GetMotorStatus3Response const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  float ActuatorInterface::getMultiTurnAngle() {
    GetMultiTurnAngleRequest const request {};
    GetMultiTurnAngleResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getAngle();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderPosition() {
    GetMultiTurnEncoderPositionRequest const request {};
    GetMultiTurnEncoderPositionResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getPosition();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderOriginalPosition() {
    GetMultiTurnEncoderOriginalPositionRequest const request {};
    GetMultiTurnEncoderOriginalPositionResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getPosition();
  }

  std::int32_t ActuatorInterface::getMultiTurnEncoderZeroOffset() {
    GetMultiTurnEncoderZeroOffsetRequest const request {};
    GetMultiTurnEncoderZeroOffsetResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getPosition();
  }

  std::chrono::milliseconds ActuatorInterface::getRuntime() {
    GetSystemRuntimeRequest const request {};
    GetSystemRuntimeResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getRuntime();
  }

  float ActuatorInterface::getSingleTurnAngle() {
    GetSingleTurnAngleRequest const request {};
    GetSingleTurnAngleResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getAngle();
  }

  std::int16_t ActuatorInterface::getSingleTurnEncoderPosition() {
    GetSingleTurnEncoderPositionRequest const request {};
    GetSingleTurnEncoderPositionResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getPosition();
  }

  std::uint32_t ActuatorInterface::getVersionDate() {
    GetVersionDateRequest const request {};
    GetVersionDateResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getVersion();
  }

  void ActuatorInterface::lockBrake() {
    LockBrakeRequest const request {};
    [[maybe_unused]] LockBrakeResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::releaseBrake() {
    ReleaseBrakeRequest const request {};
    [[maybe_unused]] ReleaseBrakeResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::reset() {
    ResetRequest const request {};
    driver_.send(request, actuator_id_);
    return;
  }

  Feedback ActuatorInterface::sendCurrentSetpoint(float const current) {
    SetTorqueRequest const request {current};
    SetTorqueResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  Feedback ActuatorInterface::sendPositionAbsoluteSetpoint(float const position, float const max_speed) {
    SetPositionAbsoluteRequest const request {position, max_speed};
    SetPositionAbsoluteResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  Feedback ActuatorInterface::sendTorqueSetpoint(float const torque, float const torque_constant) {
    auto const current {torque/torque_constant};
    return sendCurrentSetpoint(current);
  }

  Feedback ActuatorInterface::sendVelocitySetpoint(float const speed) {
    if ((speed < -32767.0f) || (speed > +32767.0f)) {
      throw ValueRangeException("Speed is not in the valid range!");
    }
    SetVelocityRequest const request {speed};
    SetVelocityResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  void ActuatorInterface::setAcceleration(std::uint32_t const acceleration, AccelerationType const mode) {
    SetAccelerationRequest const request {acceleration, mode};
    [[maybe_unused]] SetAccelerationResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::setCanId(std::uint16_t const can_id) {
    SetCanIdRequest const request {can_id};
    [[maybe_unused]] SetCanIdResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  std::int32_t ActuatorInterface::setCurrentPositionAsEncoderZero() {
    SetCurrentPositionAsEncoderZeroRequest const request {};
    SetCurrentPositionAsEncoderZeroResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getEncoderZero();
  }

  void ActuatorInterface::setEncoderZero(std::int32_t const encoder_offset) {
    SetEncoderZeroRequest const request {encoder_offset};
    [[maybe_unused]] SetEncoderZeroResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::setCanBaudRate(CanBaudRate const baud_rate) {
    SetCanBaudRateRequest const request {baud_rate};
    driver_.send(request, actuator_id_);
    return;
  }

  Gains ActuatorInterface::setControllerGains(Gains const& gains, bool const is_persistent) {
    if (is_persistent) {
      SetControllerGainsPersistentlyRequest const request {gains};
      SetControllerGainsPersistentlyResponse const response {driver_.sendRecv(request, actuator_id_)};
      return response.getGains();
    } else {
      SetControllerGainsRequest const request {gains};
      SetControllerGainsResponse const response {driver_.sendRecv(request, actuator_id_)};
      return response.getGains();
    }
  }

  void ActuatorInterface::setTimeout(std::chrono::milliseconds const& timeout) {
    SetTimeoutRequest const request {timeout};
    [[maybe_unused]] SetTimeoutResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::shutdownMotor() {
    ShutdownMotorRequest const request {};
    [[maybe_unused]] ShutdownMotorResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  void ActuatorInterface::stopMotor() {
    StopMotorRequest const request {};
    [[maybe_unused]] StopMotorResponse const response {driver_.sendRecv(request, actuator_id_)};
    return;
  }

  Feedback ActuatorInterface::sendSingleTurnPositionSetpoint(float const position, std::uint8_t const direction, float const max_speed) {
    if (position < 0.0f || position >= 360.0f) {
      throw ValueRangeException("Position is out of range [0°, 359.99°]!");
    }
    
    if (direction != 0 && direction != 1) {
      throw ValueRangeException("Direction must be 0 (clockwise) or 1 (counter-clockwise)!");
    }
    
    if (max_speed < 0.0f) {
      throw ValueRangeException("Maximum speed must be positive!");
    }
    
    SingleTurnPositionControlRequest const request{position, direction, max_speed};
    SingleTurnPositionControlResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }

  Feedback ActuatorInterface::sendIncrementalPositionSetpoint(float const position_increment, float const max_speed) {
    if (max_speed < 0.0f) {
      throw ValueRangeException("Maximum speed must be positive!");
    }
    
    IncrementalPositionControlRequest const request{position_increment, max_speed};
    IncrementalPositionControlResponse const response {driver_.sendRecv(request, actuator_id_)};
    return response.getStatus();
  }
  
  void ActuatorInterface::configureActiveReply(bool const enable, std::uint8_t const frequency) {
    if (enable && (frequency == 0 || frequency > 100)) {
      throw ValueRangeException("Frequency must be between 1 and 100 Hz when active reply is enabled!");
    }
    
    ActiveReplyFunctionRequest const request{enable, frequency};
    
    // Use the regular sendRecv method
    [[maybe_unused]] ActiveReplyFunctionResponse const response {driver_.sendRecv(request, actuator_id_)};
    
    // Notify the feedback listener if one is registered
    if (feedback_listener_ != nullptr) {
      feedback_listener_->setActiveReplyEnabled(enable);
    }
    return;
  }

  // Modified method for Actuator Interface to pass frames to feedback listener
  // This uses friend access to the driver to intercept frames
  void ActuatorInterface::passFrameToFeedbackListener(const can::Frame& frame) {
    if (feedback_listener_ != nullptr) {
      feedback_listener_->processFrame(frame);
    }
  }

}
