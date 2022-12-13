#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

namespace dai {

BenchmarkReport::Serialized BenchmarkReport::serialize() const {
    return {data, raw};
}

BenchmarkReport::BenchmarkReport()
    : Buffer(std::make_shared<RawBenchmarkReport>()),
      benchmarkReport(*dynamic_cast<RawBenchmarkReport*>(raw.get())),
      fps(benchmarkReport.fps),
      timeTotal(benchmarkReport.timeTotal),
      numMessagesReceived(benchmarkReport.numMessagesReceived),
      averageLatency(benchmarkReport.averageLatency),
      latencies(benchmarkReport.latencies) {}

BenchmarkReport::BenchmarkReport(std::shared_ptr<RawBenchmarkReport> ptr)
    : Buffer(std::move(ptr)),
      benchmarkReport(*dynamic_cast<RawBenchmarkReport*>(raw.get())),
      fps(benchmarkReport.fps),
      timeTotal(benchmarkReport.timeTotal),
      numMessagesReceived(benchmarkReport.numMessagesReceived),
      averageLatency(benchmarkReport.averageLatency),
      latencies(benchmarkReport.latencies) {}

}  // namespace dai
