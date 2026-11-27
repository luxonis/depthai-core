#pragma once

#include <string>
#include <utility>

namespace system_resource_counters {

struct Values {
    double gpuUtilizationPercent = -1.0;
    double dspHvxActivityPercent = -1.0;
    double dspHmxActivityPercent = -1.0;
    double dspQ6LoadKcps = -1.0;
    double powerSystemWatts = -1.0;
    double powerProcessorWatts = -1.0;
    double powerPoeWatts = -1.0;
    double powerTofWatts = -1.0;
};

class Collector {  // Read-only: does not write settings or reset cumulative counters
   public:
    explicit Collector(std::string sysfsRoot = "/sys", std::string debugfsRoot = "/sys/kernel/debug")
        : sysfsRoot(std::move(sysfsRoot)), debugfsRoot(std::move(debugfsRoot)) {}

    Values read() const;

   private:
    std::string sysfsRoot;
    std::string debugfsRoot;
};

}  // namespace system_resource_counters
