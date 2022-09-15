#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>
#include "depthai/depthai.hpp"

using namespace dai;
static spdlog::level::level_enum logLevelToSpdlogLevel(LogLevel level, spdlog::level::level_enum defaultValue = spdlog::level::off) {
    switch(level) {
        case LogLevel::TRACE:
            return spdlog::level::trace;
        case LogLevel::DEBUG:
            return spdlog::level::debug;
        case LogLevel::INFO:
            return spdlog::level::info;
        case LogLevel::WARN:
            return spdlog::level::warn;
        case LogLevel::ERR:
            return spdlog::level::err;
        case LogLevel::CRITICAL:
            return spdlog::level::critical;
        case LogLevel::OFF:
            return spdlog::level::off;
    }
    // Default
    return defaultValue;
}

int test(dai::LogLevel logLevel) {
    // Create pipeline
    dai::Pipeline pipeline;

    auto xIn = pipeline.create<dai::node::XLinkIn>();
    auto script = pipeline.create<dai::node::Script>();
    auto xOut = pipeline.create<dai::node::XLinkOut>();

    xIn->setStreamName("input");
    xOut->setStreamName("output");

    // Link xin node to script node
    xIn->out.link(script->inputs["log"]);
    script->outputs["out"].link(xOut->input);

    script->setScript(R"(
    while True:
        _ = node.io["log"].get()
        node.trace("TRACE")
        node.debug("DEBUG")
        node.info("INFO")
        node.warn("WARN")
        node.error("ERROR")

        message = Buffer(10)
        node.io["out"].send(message)
    )");

    dai::Device device(pipeline);

    auto in = device.getInputQueue("input");
    auto out = device.getOutputQueue("output");
    dai::Buffer message;  // Arbitrary message, used only to control flow

    device.setLogLevel(logLevel);
    device.setLogOutputLevel(logLevel);
    auto spdSetLevel = logLevelToSpdlogLevel(logLevel);

    // -2 below is for critical and no_error, which shouldn't arrive
    std::array<bool, spdlog::level::n_levels - 2> arrivedLogs;
    for(auto& level: arrivedLogs){
        level = false;
    }
    bool testPassed = true;
    auto callbackSink = [&testPassed, &arrivedLogs, spdSetLevel](dai::LogMessage message) {
        // Convert message to spd for easier comparison
        auto spdLevel = logLevelToSpdlogLevel(message.level);
        REQUIRE(spdLevel >= spdSetLevel);
        if(spdLevel < arrivedLogs.size()) {
            arrivedLogs[static_cast<int>(spdLevel)] = true;
        } else {
            FAIL();
        }
    };

    device.addLogCallback(callbackSink);
    in->send(message);
    out->get();  // Wait for the device to send the log(s)
    using namespace std::chrono;
    std::this_thread::sleep_for(milliseconds(200));  // Wait for the logs to arrive

    for(int i = 0; i < arrivedLogs.size(); i++) {
        if(i < logLevelToSpdlogLevel(logLevel)){
            REQUIRE(!arrivedLogs[i]);
        } else {
            REQUIRE(arrivedLogs[i]);
        }
    }
    device.setLogLevel(dai::LogLevel::WARN);
    device.setLogOutputLevel(dai::LogLevel::WARN);
    // Exit with success error code
    return 0;
}

TEST_CASE("TRACE") {
    test(dai::LogLevel::TRACE);
}

TEST_CASE("DEBUG") {
    test(dai::LogLevel::DEBUG);
}

TEST_CASE("INFO") {
    test(dai::LogLevel::INFO);
}

TEST_CASE("WARN") {
    test(dai::LogLevel::WARN);
}

TEST_CASE("ERROR") {
    test(dai::LogLevel::ERR);
}

// Not yet implemented in the script node
// TEST_CASE("CRITICAL") {
//     test(dai::LogLevel::CRITICAL);
// }

TEST_CASE("OFF") {
    test(dai::LogLevel::OFF);
}
