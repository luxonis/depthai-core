#pragma once

#include "depthai-shared/datatype/RawBenchmarkReport.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

// TODO(before mainline) - API not supported on RVC2
/**
 * BenchmarkReport message.
 */
class BenchmarkReport : public Buffer {
    Serialized serialize() const override;
    RawBenchmarkReport& benchmarkReport;

   public:
    /**
     * Construct BenchmarkReport message.
     */
    BenchmarkReport();
    explicit BenchmarkReport(std::shared_ptr<RawBenchmarkReport> ptr);
    virtual ~BenchmarkReport() = default;

    float& fps;
    float& timeTotal;  // seconds
    float& numMessagesReceived;
    float& averageLatency;
    std::vector<float>& latencies;
};

}  // namespace dai
