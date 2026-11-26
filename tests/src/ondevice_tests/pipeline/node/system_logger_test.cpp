#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"

void testSystemLoggerNode(const dai::SystemInformation& info) {
    // DDR memory
    REQUIRE(info.ddrMemoryUsage.used > 0);
    REQUIRE(info.ddrMemoryUsage.used < info.ddrMemoryUsage.total);
    REQUIRE(info.ddrMemoryUsage.remaining < info.ddrMemoryUsage.total);

    // CMX Process memory
    REQUIRE(info.cmxMemoryUsage.used > 0);
    REQUIRE(info.cmxMemoryUsage.used < info.cmxMemoryUsage.total);
    REQUIRE(info.cmxMemoryUsage.remaining < info.cmxMemoryUsage.total);

    // LeonCSS heap usage
    REQUIRE(info.leonCssMemoryUsage.used > 0);
    REQUIRE(info.leonCssMemoryUsage.used < info.leonCssMemoryUsage.total);
    REQUIRE(info.leonCssMemoryUsage.remaining < info.leonCssMemoryUsage.total);

    // LeonMSS heap usage
    REQUIRE(info.leonMssMemoryUsage.used > 0);
    REQUIRE(info.leonMssMemoryUsage.used < info.leonMssMemoryUsage.total);
    REQUIRE(info.leonMssMemoryUsage.remaining < info.leonMssMemoryUsage.total);

    // Average CPU Usage
    REQUIRE(info.leonCssCpuUsage.average > 0.0f);
    REQUIRE(info.leonCssCpuUsage.average < 1.0f);
    REQUIRE(info.leonMssCpuUsage.average > 0.0f);
    REQUIRE(info.leonMssCpuUsage.average < 1.0f);

    // Temperatures (-40.96°C indicates non working sensor)
    REQUIRE(info.chipTemperature.css > -40.0f);
    REQUIRE(info.chipTemperature.mss > -40.0f);
    REQUIRE(info.chipTemperature.upa > -40.0f);
    REQUIRE(info.chipTemperature.dss > -40.0f);

    float total = info.chipTemperature.css + info.chipTemperature.mss + info.chipTemperature.upa + info.chipTemperature.dss;
    REQUIRE(info.chipTemperature.average == Catch::Approx(total / 4.0f).epsilon(0.01));
}

void testSystemLoggerNode(const dai::SystemInformationRVC4& info) {
    const auto isUnavailableOrInRange = [](double value, double upperBound) { return value == -1 || (value >= 0 && value <= upperBound); };

    // DDR memory
    REQUIRE(info.ddrMemoryUsage.used > 0);
    REQUIRE(info.ddrMemoryUsage.used < info.ddrMemoryUsage.total);
    REQUIRE(info.ddrMemoryUsage.remaining < info.ddrMemoryUsage.total);

    // Device Process
    REQUIRE(info.processMemoryUsage > 0);
    REQUIRE(info.processMemoryUsage < info.ddrMemoryUsage.used / 1024);
    REQUIRE(info.processMemoryUsage < info.ddrMemoryUsage.total / 1024);
    REQUIRE(info.processCpuAvgUsage.average > 0.0f);
    REQUIRE(info.processCpuAvgUsage.average < 1.0f);

    // Average CPU Usage
    REQUIRE(info.cpuAvgUsage.average > 0.0f);
    REQUIRE(info.cpuAvgUsage.average < 1.0f);

    // Temperatures (-40.96°C indicates non working sensor)
    REQUIRE(info.chipTemperature.cpuss > -40.0f);
    REQUIRE(info.chipTemperature.gpuss > -40.0f);
    REQUIRE(info.chipTemperature.mdmss > -40.0f);
    REQUIRE(info.chipTemperature.video > -40.0f);
    REQUIRE(info.chipTemperature.ddr > -40.0f);
    REQUIRE(info.chipTemperature.camera > -40.0f);

    float total = info.chipTemperature.cpuss + info.chipTemperature.gpuss + info.chipTemperature.mdmss + info.chipTemperature.video + info.chipTemperature.ddr
                  + info.chipTemperature.camera;
    REQUIRE(info.chipTemperature.average == Catch::Approx(total / 6.0f).epsilon(0.01));

    // Vendor counters may be unavailable on a platform/firmware combination.
    REQUIRE(isUnavailableOrInRange(info.gpuUtilizationPercent, 100));
    REQUIRE(isUnavailableOrInRange(info.dspHvxActivityPercent, 100));
    REQUIRE(isUnavailableOrInRange(info.dspHmxActivityPercent, 100));
    REQUIRE(isUnavailableOrInRange(info.dspQ6LoadKcps, std::numeric_limits<double>::max()));
    REQUIRE(isUnavailableOrInRange(info.powerSystemWatts, std::numeric_limits<double>::max()));
    REQUIRE(isUnavailableOrInRange(info.powerProcessorWatts, std::numeric_limits<double>::max()));
    REQUIRE(isUnavailableOrInRange(info.powerPoeWatts, std::numeric_limits<double>::max()));
    REQUIRE(isUnavailableOrInRange(info.powerTofWatts, std::numeric_limits<double>::max()));
}

namespace {

struct ResourceMetric {
    const char* name;
    double dai::SystemInformationRVC4::*value;
    bool required = false;
    bool observed = false;
};

std::array<ResourceMetric, 8> getResourceMetrics(bool requireMetrics) {
    std::array<ResourceMetric, 8> metrics{{
        {"gpuUtilizationPercent", &dai::SystemInformationRVC4::gpuUtilizationPercent},
        {"dspHvxActivityPercent", &dai::SystemInformationRVC4::dspHvxActivityPercent},
        {"dspHmxActivityPercent", &dai::SystemInformationRVC4::dspHmxActivityPercent},
        {"dspQ6LoadKcps", &dai::SystemInformationRVC4::dspQ6LoadKcps},
        {"powerSystemWatts", &dai::SystemInformationRVC4::powerSystemWatts},
        {"powerProcessorWatts", &dai::SystemInformationRVC4::powerProcessorWatts},
        {"powerPoeWatts", &dai::SystemInformationRVC4::powerPoeWatts},
        {"powerTofWatts", &dai::SystemInformationRVC4::powerTofWatts},
    }};
    const char* expected = std::getenv("DEPTHAI_TEST_SYSTEM_LOGGER_REQUIRED_METRICS");
    bool hasRequiredMetrics = false;
    if(expected != nullptr) {
        std::istringstream names(expected);
        std::string name;
        while(names >> name) {
            bool found = false;
            for(auto& metric : metrics) {
                if(name == metric.name) {
                    metric.required = true;
                    found = true;
                }
            }
            INFO("Unknown SystemLogger metric: " << name);
            REQUIRE(found);
            hasRequiredMetrics = true;
        }
    }
    if(requireMetrics) {
        INFO("Set DEPTHAI_TEST_SYSTEM_LOGGER_REQUIRED_METRICS for this camera/OS; see tests/hardware/system_logger.md");
        REQUIRE(hasRequiredMetrics);
    }

    return metrics;
}

void runSystemLoggerTest(bool requireMetrics) {
    auto metrics = getResourceMetrics(requireMetrics);
    const bool hasRequiredMetrics = std::any_of(metrics.begin(), metrics.end(), [](const ResourceMetric& metric) { return metric.required; });
    dai::Pipeline p;
    auto logger = p.create<dai::node::SystemLogger>();
    auto loggerQueue = logger->out.createOutputQueue(4, false);
    const auto platform = p.getDefaultDevice()->getPlatform();
    if(hasRequiredMetrics) REQUIRE(platform == dai::Platform::RVC4);

    p.start();
    for(int i = 0; i < 10; i++) {
        if(platform == dai::Platform::RVC2) {
            auto sysInfo = loggerQueue->get<dai::SystemInformation>();
            REQUIRE(sysInfo != nullptr);
            testSystemLoggerNode(*sysInfo);
        } else {
            auto sysInfo = loggerQueue->get<dai::SystemInformationRVC4>();
            REQUIRE(sysInfo != nullptr);
            testSystemLoggerNode(*sysInfo);
            for(auto& metric : metrics) {
                metric.observed = metric.observed || (*sysInfo).*metric.value >= 0;
            }
        }
    }
    // Allow initial unavailable samples, but never accept a missing counter for the whole run.
    for(const auto& metric : metrics) {
        if(metric.required) {
            INFO("Expected SystemLogger metric stayed unavailable: " << metric.name);
            CHECK(metric.observed);
        }
    }
}

}  // namespace

TEST_CASE("Test SystemLogger Node") {
    runSystemLoggerTest(false);
}

TEST_CASE("SystemLogger reports expected RVC4 metrics", "[.][systemlogger-availability]") {
    runSystemLoggerTest(true);
}
