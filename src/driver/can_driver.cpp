#include "myactuator_rmd/driver/can_driver.hpp"

#include <array>
#include <cstdint>

#include "myactuator_rmd/actuator_interface.hpp"
#include "myactuator_rmd/can/frame.hpp"
#include "myactuator_rmd/protocol/message.hpp"

namespace myactuator_rmd {

  std::array<std::uint8_t,8> CanDriver::sendRecv(Message const& request, std::uint32_t const actuator_id, ActuatorInterface* interface) {
    auto const can_send_id = getCanSendId(actuator_id);
    write(can_send_id, request.getData());
    can::Frame const frame {can::Node::read()};
    
    // Pass the frame to the actuator interface if provided
    if (interface != nullptr) {
      interface->passFrameToFeedbackListener(frame);
    }
    
    return frame.getData();
  }

} 
