#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * Specify benchmark properties (number of messages to send/receive)
 */
struct BenchmarkInProperties : PropertiesSerializable<Properties, BenchmarkInProperties> {
    /**
     * Specify how many messages to measure for each report
     */
    uint32_t reportEveryNMessages = 50;

    /**
     * Specify whether the latenices are attached to the report individually
     */
    bool attachLatencies = false;

    /**
     * Send the reports also as logger warnings
     */
    bool logReportsAsWarnings = true;
};
#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(BenchmarkInProperties, reportEveryNMessages, attachLatencies, logReportsAsWarnings);

}  // namespace dai
