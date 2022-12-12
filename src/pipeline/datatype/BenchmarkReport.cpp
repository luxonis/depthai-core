#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

namespace dai {

std::shared_ptr<RawBuffer> BenchmarkReport::serialize() const {
    return raw;
}

BenchmarkReport::BenchmarkReport()
    : Buffer(std::make_shared<RawBenchmarkReport>()),
      benchmarkReport(*dynamic_cast<RawBenchmarkReport*>(raw.get())),
      fps(benchmarkReport.fps),
      timeTotal(benchmarkReport.timeTotal),
      numMessagesReceived(benchmarkReport.numMessagesReceived) {}

BenchmarkReport::BenchmarkReport(std::shared_ptr<RawBenchmarkReport> ptr)
    : Buffer(std::move(ptr)),
      benchmarkReport(*dynamic_cast<RawBenchmarkReport*>(raw.get())),
      fps(benchmarkReport.fps),
      timeTotal(benchmarkReport.timeTotal),
      numMessagesReceived(benchmarkReport.numMessagesReceived) {}

}  // namespace dai
