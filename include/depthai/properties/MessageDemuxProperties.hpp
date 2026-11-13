#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for MessageDemux.
 */
struct MessageDemuxProperties : PropertiesSerializable<Properties, MessageDemuxProperties> {
    /**
     * Which processor should execute the node.
     */
    ProcessorType processor = ProcessorType::LEON_CSS;

    ~MessageDemuxProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(MessageDemuxProperties, processor);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::MessageDemuxProperties>;
