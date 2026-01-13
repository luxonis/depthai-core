#pragma once

// standard
#include <memory>

// libraries
#include <XLink/XLinkPublicDefines.h>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {
class StreamMessageParser {
   public:
    /**
     * Parse a message from a stream packet descriptor.
     */
    static std::shared_ptr<ADatatype> parseMessage(StreamPacketDesc packet);
    /**
     * Parse a message from a raw stream packet descriptor.
     */
    static std::shared_ptr<ADatatype> parseMessage(streamPacketDesc_t* const packet);
    // static std::vector<std::uint8_t> serializeMessage(const std::shared_ptr<const ADatatype>& data);
    // static std::vector<std::uint8_t> serializeMessage(const ADatatype& data);
    /**
     * Serialize metadata for a shared message.
     */
    static std::vector<std::uint8_t> serializeMetadata(const std::shared_ptr<const ADatatype>& data);
    /**
     * Serialize metadata for a message.
     */
    static std::vector<std::uint8_t> serializeMetadata(const ADatatype& data);
};
}  // namespace dai
