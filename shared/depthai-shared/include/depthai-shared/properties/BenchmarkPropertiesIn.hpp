#pragma once

#include "depthai-shared/common/ProcessorType.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/properties/Properties.hpp"

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
