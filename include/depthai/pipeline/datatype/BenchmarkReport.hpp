#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * BenchmarkReport message.
 */
class BenchmarkReport : public Buffer {
   public:
    BenchmarkReport() = default;

    float fps = 0.0f;
    float timeTotal = 0.0f;  // seconds
    float numMessagesReceived = 0;
    float averageLatency = 0.0f;  // seconds

    // Only filled if measureIndividualLatencies is set to true
    std::vector<float> latencies;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::BenchmarkReport;
    };

    DEPTHAI_SERIALIZE(BenchmarkReport, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, fps, timeTotal, numMessagesReceived, averageLatency, latencies);
};

}  // namespace dai
