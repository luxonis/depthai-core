#include <catch2/catch_all.hpp>

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
    REQUIRE(info.cpuAvgUsage.average > info.processCpuAvgUsage.average);

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
}

TEST_CASE("Test SystemLogger Node") {
    dai::Pipeline p;
    auto logger = p.create<dai::node::SystemLogger>();
    auto loggerQueue = logger->out.createOutputQueue(4, false);
    auto platform = p.getDefaultDevice()->getPlatform();

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
        }
    }
}