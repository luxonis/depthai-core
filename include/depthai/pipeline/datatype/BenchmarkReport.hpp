#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

// TODO(before mainline) - API not supported on RVC2
/**
 * BenchmarkReport message.
 */
class BenchmarkReport : public Buffer {
   public:
    BenchmarkReport() = default;
    virtual ~BenchmarkReport() = default;

    float fps;
    float timeTotal;  // seconds
    float numMessagesReceived;
    float averageLatency;
    std::vector<float> latencies;
    // TODO Add jitter, timestamps for start/end, possibly a vector of timestamps for all messages
    // TODO BEFORE MAINLINE add setters and getters

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::BenchmarkReport;
    };

    DEPTHAI_SERIALIZE(BenchmarkReport, sequenceNum, ts, tsDevice, fps, timeTotal, numMessagesReceived, averageLatency, latencies);
};

}  // namespace dai
