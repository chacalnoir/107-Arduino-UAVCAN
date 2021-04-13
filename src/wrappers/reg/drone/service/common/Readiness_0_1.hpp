/**
 * This software is distributed under the terms of the MIT License.
 * Author: Joel Dunham
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef ARDUINO_TRANSFER_REG_DRONE_SERVICE_COMMON_READINESS_0_1_HPP_
#define ARDUINO_TRANSFER_REG_DRONE_SERVICE_COMMON_READINESS_0_1_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <libcanard/canard.h>

#include <types/reg/drone/service/common/Readiness_0_1.h>

#include <utility/convert.hpp>

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

template <CanardPortID ID>
class Readiness_0_1
{

public:

  enum class Readiness : uint8_t
  {
    SLEEP      = reg_drone_service_common_Readiness_0_1_SLEEP,
    STANDBY    = reg_drone_service_common_Readiness_0_1_STANDBY,
    ENGAGED    = reg_drone_service_common_Readiness_0_1_ENGAGED,
  };

  reg_drone_service_common_Readiness_0_1 data;

	static constexpr CanardPortID       PORT_ID          = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = reg_drone_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND = CanardTransferKindMessage;

  Readiness_0_1()
  {
    reg_drone_service_common_Readiness_0_1_initialize_(&data);
  }

  Readiness_0_1(Readiness_0_1 const & other)
  {
    memcpy(&data, &other.data, sizeof(data));
  }

  static Readiness_0_1 deserialize(CanardTransfer const & transfer)
  {
    Readiness_0_1 h;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    reg_drone_service_common_Readiness_0_1_deserialize_(&h.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
    return h;
  }

  size_t serialize(uint8_t * payload) const
  {
    size_t inout_buffer_size_bytes = Readiness_0_1::MAX_PAYLOAD_SIZE;
    return (reg_drone_service_common_Readiness_0_1_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }

  void operator = (Readiness const readiness)
  {
    data.value = arduino::_107_::uavcan::to_integer(readiness);
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common */
} /* service */
} /* drone */
} /* reg */

#endif /* ARDUINO_TRANSFER_REG_DRONE_SERVICE_COMMON_READINESS_0_1_HPP_ */
