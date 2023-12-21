#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify benchmark properties (number of messages to send/receive)
 */
struct BenchmarkPropertiesIn : PropertiesSerializable<Properties, BenchmarkPropertiesIn> {
    /**
     * Number of messages to send
     */
    int numMessages = 50;
};

DEPTHAI_SERIALIZE_EXT(BenchmarkPropertiesIn, numMessages);

}  // namespace dai
