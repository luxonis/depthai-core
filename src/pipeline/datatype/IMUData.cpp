#include "depthai/pipeline/datatype/IMUData.hpp"

namespace dai {
span<const uint8_t> IMUData::getRecordData() const {
    return {(uint8_t*)packets.data(), packets.size() * sizeof(IMUPacket)};
}
}  // namespace dai
