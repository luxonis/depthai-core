#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify benchmark properties (number of messages to send/receive)
 */
struct BenchmarkPropertiesOut : PropertiesSerializable<Properties, BenchmarkPropertiesOut> {
    /**
     * Number of messages to send
     */
    int numMessages = 50;

    /**
     * FPS for sending, 0 means as fast as possible
     */
    float fps = 0;
};

DEPTHAI_SERIALIZE_EXT(BenchmarkPropertiesOut, numMessages, fps);

}  // namespace dai
