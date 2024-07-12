#include <spdlog/spdlog.h>

#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

int test(dai::LogLevel logLevel) {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    while True:
        _ = node.io["log"].get()
        node.trace("TRACE")
        node.debug("DEBUG")
        node.info("INFO")
        node.warn("WARN")
        node.error("ERROR")
        node.critical("CRITICAL")

        message = Buffer(10)
        node.io["out"].send(message)
    )");

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.getDefaultDevice()->setLogLevel(logLevel);
    pipeline.getDefaultDevice()->setLogOutputLevel(logLevel);
    pipeline.start();

    // -1 below is for no_error, which cannot arrive
    std::array<bool, spdlog::level::n_levels - 1> arrivedLogs;
    arrivedLogs.fill(false);

    bool testPassed = true;
    auto logLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(logLevel);
    auto callbackSink = [&testPassed, &arrivedLogs, logLevelConverted](dai::LogMessage message) {
        // Convert message to spd for easier comparison
        auto messageLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(message.level);
        REQUIRE(messageLevelConverted >= logLevelConverted);
        if(messageLevelConverted < arrivedLogs.size()) {
            arrivedLogs[messageLevelConverted] = true;
        } else {
            FAIL();
        }
    };
    pipeline.getDefaultDevice()->addLogCallback(callbackSink);

    // Send message and wait for response
    in->send(std::make_shared<dai::Buffer>());
    out->get();

    // Wait for logs to arrive
    using namespace std::chrono;
    std::this_thread::sleep_for(milliseconds(200));

    for(int i = 0; i < arrivedLogs.size(); i++) {
        if(i < logLevelConverted) REQUIRE(!arrivedLogs[i]);
        if(i >= logLevelConverted) REQUIRE(arrivedLogs[i]);
    }

    // Restore default log levels
    pipeline.getDefaultDevice()->setLogLevel(dai::LogLevel::WARN);
    pipeline.getDefaultDevice()->setLogOutputLevel(dai::LogLevel::WARN);

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

TEST_CASE("CRITICAL") {
    test(dai::LogLevel::CRITICAL);
}

TEST_CASE("OFF") {
    test(dai::LogLevel::OFF);
}
