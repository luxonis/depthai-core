#pragma once

#include <XLink/XLinkPublicDefines.h>

#include <memory>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/xlink/XLinkStream.hpp"

namespace dai {
class StreamMessageParser {
    static std::shared_ptr<ADatatype> parseMessage(streamPacketDesc_t* packet, const std::shared_ptr<Memory>& data);

   public:
    static std::shared_ptr<ADatatype> parseMessage(StreamPacketDesc packet);
    static std::shared_ptr<ADatatype> parseMessage(std::vector<uint8_t> packet);
    static std::shared_ptr<ADatatype> parseMessage(streamPacketDesc_t* const packet);
    static std::vector<std::uint8_t> serializeMetadata(const std::shared_ptr<const ADatatype>& data);
    static std::vector<std::uint8_t> serializeMetadata(const ADatatype& data);
};
}  // namespace dai
