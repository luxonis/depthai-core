#include "SystemLogger.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

#include "Measurements.hpp"
#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"
#include "pipeline/NodeTiming.hpp"

namespace dai {
namespace gate {
void SystemLogger::run() {
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    duration<float> sleepPeriod(1.0f / getRate());

    Measurements measurements(logger);

    // Needed for measuring CPU load
    std::vector<Measurements::cpuData> entriesStart;
    std::vector<Measurements::cpuData> entriesEnd;
    Measurements::cpuData totalStart;
    Measurements::cpuData totalEnd;

    measurements.readStatsCpu(entriesEnd, totalEnd);

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        auto tGotInput = tAbsoluteBeginning;
        auto pSysInfo = std::make_shared<dai::SystemInformationRVC4>();

        // Measure CPU load
        entriesStart.swap(entriesEnd);
        totalStart = totalEnd;
        std::this_thread::sleep_for(sleepPeriod);
        measurements.readStatsCpu(entriesEnd, totalEnd);
        measurements.readSystemInformation(*pSysInfo, sleepPeriod, entriesStart, entriesEnd, totalStart, totalEnd);

        // Send out system information
        auto tProcessed = steady_clock::now();
        out.send(pSysInfo);
        auto tAbsoluteEnd = steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace gate
}  // namespace dai
