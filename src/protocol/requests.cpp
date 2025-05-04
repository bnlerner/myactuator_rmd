/**
 * \file requests.cpp
 * \mainpage
 *    Contains implementations of new request classes
 * \author
 *    Implementation based on plan
*/

#include "myactuator_rmd/protocol/requests.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  bool CanIdRequest::isWrite() const noexcept {
    return (getAs<std::uint8_t>(2) == 0) ? true : false;
  }

  GetCanIdRequest::GetCanIdRequest()
  : CanIdRequest{}  {
    setAt(static_cast<std::uint8_t>(1), 2);
    return;
  }

  SetCanIdRequest::SetCanIdRequest(std::uint16_t const can_id)
  : CanIdRequest{}  {
    setAt(static_cast<std::uint8_t>(0), 2);
    setAt(static_cast<std::uint8_t>(can_id), 6);
    return;
  }

  std::uint16_t SetCanIdRequest::getCanId() const noexcept {
    return static_cast<std::uint16_t>(getAs<std::uint8_t>(7));
  }

  SetAccelerationRequest::SetAccelerationRequest(std::uint32_t const acceleration, AccelerationType const mode)
  : SingleMotorRequest{} {
    if ((acceleration != 0) && ((acceleration < 100) || (acceleration > 60000))) {
      throw ValueRangeException("Acceleration value '" + std::to_string(acceleration) + "' out of range [100, 60000]");
    }
    auto const acceleration_type {static_cast<std::uint8_t>(mode)};
    setAt(acceleration_type, 1);
    setAt(acceleration, 4);
    return;
  }

  std::uint32_t SetAccelerationRequest::getAcceleration() const noexcept {
    return getAs<std::uint32_t>(4);
  }

  AccelerationType SetAccelerationRequest::getMode() const noexcept {
    return static_cast<AccelerationType>(getAs<std::uint8_t>(1));
  }

  SetCanBaudRateRequest::SetCanBaudRateRequest(CanBaudRate const baud_rate)
  : SingleMotorRequest{} {
    setAt(static_cast<std::uint8_t>(baud_rate), 7);
    return;
  }

  CanBaudRate SetCanBaudRateRequest::getBaudRate() const noexcept {
    return static_cast<CanBaudRate>(getAs<std::uint8_t>(7));
  }

  SetEncoderZeroRequest::SetEncoderZeroRequest(std::int32_t const encoder_offset)
  : SingleMotorRequest{} {
    setAt(encoder_offset, 4);
    return;
  }

  std::int32_t SetEncoderZeroRequest::getEncoderZero() const noexcept {
    return getAs<std::int32_t>(4);
  }

  SetPositionAbsoluteRequest::SetPositionAbsoluteRequest(float const position, float const max_speed)
  : SingleMotorRequest{} {
    auto const v {static_cast<std::uint16_t>(max_speed)};
    auto const pos {static_cast<std::int32_t>(position*100.0f)};
    setAt(v, 2);
    setAt(pos, 4);
    return;
  }

  float SetPositionAbsoluteRequest::getMaxSpeed() const noexcept {
    return static_cast<float>(getAs<std::uint16_t>(2));
  }

  float SetPositionAbsoluteRequest::getPosition() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4)/100.0f);
  }

  SetTorqueRequest::SetTorqueRequest(float const current)
  : SingleMotorRequest{} {
    auto const c {static_cast<std::int16_t>(current/0.01f)};
    setAt(c, 4);
    return;
  }

  float SetTorqueRequest::getTorqueCurrent() const noexcept {
    return static_cast<float>(getAs<std::int16_t>(4))*0.01f;
  }

  SetTimeoutRequest::SetTimeoutRequest(std::chrono::milliseconds const& timeout) {
    setAt(static_cast<std::uint32_t>(timeout.count()), 4);
    return;
  }

  std::chrono::milliseconds SetTimeoutRequest::getTimeout() const noexcept {
    std::chrono::milliseconds const timeout {getAs<std::uint32_t>(4)};
    return timeout;
  }

  SetVelocityRequest::SetVelocityRequest(float const speed)
  : SingleMotorRequest{} {
    auto const s {static_cast<std::int32_t>(speed*100.0f)};
    setAt(s, 4);
    return;
  }

  float SetVelocityRequest::getSpeed() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4))/100.0f;
  }

  // Implementation of ActiveReplyFunctionRequest
  ActiveReplyFunctionRequest::ActiveReplyFunctionRequest(bool const enable, std::uint8_t const frequency)
    : SingleMotorRequest{} {
    data_[1] = enable ? 0x01 : 0x00;
    data_[2] = frequency;
    // Other data fields remain 0
  }

  bool ActiveReplyFunctionRequest::isEnabled() const noexcept {
    return (data_[1] == 0x01);
  }

  std::uint8_t ActiveReplyFunctionRequest::getFrequency() const noexcept {
    return data_[2];
  }
  
  // Implementation of SingleTurnPositionControlRequest
  SingleTurnPositionControlRequest::SingleTurnPositionControlRequest(float const position,
                                                                   std::uint8_t const direction, 
                                                                   float const max_speed)
    : SingleMotorRequest{} {
    data_[1] = direction;
    
    std::uint16_t const speed_int{static_cast<std::uint16_t>(max_speed)};
    data_[2] = static_cast<std::uint8_t>(speed_int & 0xFF);
    data_[3] = static_cast<std::uint8_t>((speed_int >> 8) & 0xFF);
    
    std::uint16_t const position_int{static_cast<std::uint16_t>(position * 100.0f)};
    data_[4] = static_cast<std::uint8_t>(position_int & 0xFF);
    data_[5] = static_cast<std::uint8_t>((position_int >> 8) & 0xFF);
    
    // data_[6] and data_[7] remain 0
  }

  float SingleTurnPositionControlRequest::getPosition() const noexcept {
    std::uint16_t const position_int{static_cast<std::uint16_t>((data_[5] << 8) | data_[4])};
    return static_cast<float>(position_int) * 0.01f;
  }

  std::uint8_t SingleTurnPositionControlRequest::getDirection() const noexcept {
    return data_[1];
  }

  float SingleTurnPositionControlRequest::getMaxSpeed() const noexcept {
    std::uint16_t const speed_int{static_cast<std::uint16_t>((data_[3] << 8) | data_[2])};
    return static_cast<float>(speed_int);
  }

  // Implementation of IncrementalPositionControlRequest
  IncrementalPositionControlRequest::IncrementalPositionControlRequest(float const position_increment,
                                                                     float const max_speed)
    : SingleMotorRequest{} {
    std::uint16_t const speed_int{static_cast<std::uint16_t>(max_speed)};
    data_[2] = static_cast<std::uint8_t>(speed_int & 0xFF);
    data_[3] = static_cast<std::uint8_t>((speed_int >> 8) & 0xFF);
    
    std::int32_t const position_int{static_cast<std::int32_t>(position_increment * 100.0f)};
    data_[4] = static_cast<std::uint8_t>(position_int & 0xFF);
    data_[5] = static_cast<std::uint8_t>((position_int >> 8) & 0xFF);
    data_[6] = static_cast<std::uint8_t>((position_int >> 16) & 0xFF);
    data_[7] = static_cast<std::uint8_t>((position_int >> 24) & 0xFF);
  }

  float IncrementalPositionControlRequest::getPositionIncrement() const noexcept {
    std::int32_t const position_int{static_cast<std::int32_t>(
      (data_[7] << 24) | (data_[6] << 16) | (data_[5] << 8) | data_[4]
    )};
    return static_cast<float>(position_int) * 0.01f;
  }

  float IncrementalPositionControlRequest::getMaxSpeed() const noexcept {
    std::uint16_t const speed_int{static_cast<std::uint16_t>((data_[3] << 8) | data_[2])};
    return static_cast<float>(speed_int);
  }
}
