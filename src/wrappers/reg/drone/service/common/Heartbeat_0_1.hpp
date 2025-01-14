/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Florin Berger
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_UAVCAN_TYPES_REG_DRONE_SERVICE_COMMON_HEARTBEAT_0_1_HPP_
#define ARDUINO_UAVCAN_TYPES_REG_DRONE_SERVICE_COMMON_HEARTBEAT_0_1_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include <types/reg/drone/service/common/Heartbeat_0_1.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace reg {
namespace drone {
namespace service {
namespace common {

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/
template<CanardPortID ID>
class Heartbeat_0_1
{

public:

  reg_drone_service_common_Heartbeat_0_1 data;

  static constexpr CanardPortID       PORT_ID = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = reg_drone_service_common_Heartbeat_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND = CanardTransferKindMessage;


  Heartbeat_0_1()
  {
    reg_drone_service_common_Heartbeat_0_1_initialize_(&data);
  }

  Heartbeat_0_1(Heartbeat_0_1 const &other)
  {
    memcpy(&data, &other.data, sizeof(data));
  }

  static Heartbeat_0_1 deserialize(CanardTransfer const &transfer)
  {
    Heartbeat_0_1 b;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    reg_drone_service_common_Heartbeat_0_1_deserialize_(&b.data, (uint8_t * )(transfer.payload),
                                                                         &inout_buffer_size_bytes);
    return b;
  }

  size_t serialize(uint8_t *payload) const
  {
    size_t inout_buffer_size_bytes = Heartbeat_0_1::MAX_PAYLOAD_SIZE;
    return (reg_drone_service_common_Heartbeat_0_1_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common */
} /* service */
} /* drone */
} /* reg */

#endif /* ARDUINO_UAVCAN_TYPES_REG_DRONE_SERVICE_COMMON_HEARTBEAT_0_1_HPP_ */
