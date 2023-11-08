#pragma once

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawAprilTags structure
struct RawBenchmarkReport : public RawBuffer {
    float fps;
    float timeTotal;  // seconds
    float numMessagesReceived;
    float averageLatency;
    std::vector<float> latencies;

    // TODO Add jitter, timestamps for start/end, possibly a vector of timestamps for all messages

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::BenchmarkReport;
    };

    DEPTHAI_SERIALIZE(RawBenchmarkReport, fps, timeTotal, numMessagesReceived, averageLatency, latencies);
};

}  // namespace dai