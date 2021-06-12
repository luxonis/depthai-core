#pragma once

// standard
#include <memory>

// libraries
#include <XLink/XLinkPublicDefines.h>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {
class StreamPacketParser {
   public:
    static std::shared_ptr<RawBuffer> parsePacket(streamPacketDesc_t* packet);
    static std::shared_ptr<ADatatype> parsePacketToADatatype(streamPacketDesc_t* packet);
    static std::vector<std::uint8_t> serializeMessage(const std::shared_ptr<const RawBuffer>& data);
    static std::vector<std::uint8_t> serializeMessage(const RawBuffer& data);
    static std::vector<std::uint8_t> serializeMessage(const std::shared_ptr<const ADatatype>& data);
    static std::vector<std::uint8_t> serializeMessage(const ADatatype& data);
};
}  // namespace dai
