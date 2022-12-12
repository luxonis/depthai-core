#pragma once

#include "depthai-shared/datatype/RawBenchmarkReport.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * BenchmarkReport message. Carries fps recorded.
 */
class BenchmarkReport : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
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
};

}  // namespace dai
