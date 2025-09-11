#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * Specify benchmark properties (number of messages to send/receive)
 */
struct BenchmarkOutProperties : PropertiesSerializable<Properties, BenchmarkOutProperties> {
    /**
     * Number of messages to send
     */
    int numMessages = -1;

    /**
     * FPS for sending, 0 means as fast as possible
     */
    float fps = 0;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(BenchmarkOutProperties, numMessages, fps);

}  // namespace dai
