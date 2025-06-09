#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/log/LogLevel.hpp"
#include "depthai/pipeline/InputQueue.hpp"

class LoggingTestFixture {
   public:
    // Book-keeping
    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::Script> script;
    std::shared_ptr<dai::InputQueue> in;
    std::shared_ptr<dai::MessageQueue> out;
    std::array<bool, static_cast<int>(dai::LogLevel::OFF)> arrivedLogs;
    std::shared_ptr<dai::Device> device;

    LoggingTestFixture() {
        device = pipeline.getDefaultDevice();
    }

    void setupPipeline() {
        script = pipeline.create<dai::node::Script>();
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

        in = script->inputs["log"].createInputQueue();
        out = script->outputs["out"].createOutputQueue();
    }

    void verifyLogs(dai::LogLevel logLevel) {
        std::string expectedNodeIdName = "Script(" + std::to_string(script->id) + ")";
        arrivedLogs.fill(false);

        auto logLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(logLevel);
        auto callbackSink = [this, logLevelConverted, &expectedNodeIdName](dai::LogMessage message) {
            // Accept logs only from our Script node
            if(message.nodeIdName != expectedNodeIdName) return;

            auto messageLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(message.level);
            REQUIRE(messageLevelConverted >= logLevelConverted);
            if(messageLevelConverted < arrivedLogs.size()) {
                arrivedLogs[messageLevelConverted] = true;
            } else {
                FAIL();
            }
        };

        // Add callback function
        int callbackId = pipeline.getDefaultDevice()->addLogCallback(callbackSink);

        in->send(std::make_shared<dai::Buffer>());
        out->get();

        using namespace std::chrono;
        std::this_thread::sleep_for(milliseconds(200));

        // Remove callback function
        pipeline.getDefaultDevice()->removeLogCallback(callbackId);

        for(int i = 0; i < arrivedLogs.size(); i++) {
            if(i < logLevelConverted) REQUIRE(!arrivedLogs[i]);
            if(i >= logLevelConverted) REQUIRE(arrivedLogs[i]);
        }
    }

    void restoreLogLevels() {
        device->setLogLevel(dai::LogLevel::WARN);
        device->setLogOutputLevel(dai::LogLevel::WARN);
    }
};

void testDeviceLogLevel(dai::LogLevel logLevel) {
    LoggingTestFixture fixture;
    fixture.setupPipeline();

    // Set log level before start
    fixture.device->setLogLevel(logLevel);
    fixture.device->setLogOutputLevel(logLevel);

    // Make sure the log level was set correctly
    REQUIRE(fixture.device->getLogLevel() == logLevel);
    REQUIRE(fixture.script->getLogLevel() == logLevel);
    REQUIRE(fixture.device->getNodeLogLevel(fixture.script->id) == logLevel);

    // Start pipeline
    fixture.pipeline.start();

    // The log level should be set correctly after start
    REQUIRE(fixture.device->getLogLevel() == logLevel);
    REQUIRE(fixture.script->getLogLevel() == logLevel);
    REQUIRE(fixture.device->getNodeLogLevel(fixture.script->id) == logLevel);

    // Verify longs
    fixture.verifyLogs(logLevel);

    // Restore default log levels
    fixture.restoreLogLevels();
}

void testNodeLogLevel(dai::LogLevel logLevel, bool beforeStart) {
    // Setup pipeline
    LoggingTestFixture fixture;
    fixture.setupPipeline();
    if(beforeStart) {
        fixture.pipeline.start();
    }

    // Set log level to CRITICAL for the whole device
    fixture.device->setLogLevel(dai::LogLevel::CRITICAL);
    REQUIRE(fixture.device->getLogLevel() == dai::LogLevel::CRITICAL);
    REQUIRE(fixture.script->getLogLevel() == dai::LogLevel::CRITICAL);

    // Set log level before start
    fixture.script->setLogLevel(logLevel);
    fixture.device->setLogOutputLevel(logLevel);

    // Make sure the log level was set correctly
    REQUIRE(fixture.script->getLogLevel() == logLevel);
    REQUIRE(fixture.device->getNodeLogLevel(fixture.script->id) == logLevel);
    REQUIRE(fixture.device->getLogLevel() == dai::LogLevel::CRITICAL);

    if(!beforeStart) {
        fixture.pipeline.start();
    }

    // The log level should be set correctly after start
    REQUIRE(fixture.script->getLogLevel() == logLevel);
    REQUIRE(fixture.device->getNodeLogLevel(fixture.script->id) == logLevel);
    REQUIRE(fixture.device->getLogLevel() == dai::LogLevel::CRITICAL);

    // Verify logs
    fixture.verifyLogs(logLevel);

    // Verify that we can set the log level dynamically = when the pipeline is running
    fixture.script->setLogLevel(dai::LogLevel::WARN);
    fixture.device->setLogOutputLevel(dai::LogLevel::WARN);
    fixture.verifyLogs(dai::LogLevel::WARN);

    // Setting log back to ERROR should affect all nodes
    fixture.device->setLogLevel(dai::LogLevel::ERR);
    REQUIRE(fixture.device->getLogLevel() == dai::LogLevel::ERR);
    REQUIRE(fixture.script->getLogLevel() == dai::LogLevel::ERR);
    REQUIRE(fixture.device->getNodeLogLevel(fixture.script->id) == dai::LogLevel::ERR);

    // Restore default log levels
    fixture.restoreLogLevels();
}

/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
TEST_CASE("Device TRACE") {
    testDeviceLogLevel(dai::LogLevel::TRACE);
}

TEST_CASE("Device DEBUG") {
    testDeviceLogLevel(dai::LogLevel::DEBUG);
}

TEST_CASE("Device INFO") {
    testDeviceLogLevel(dai::LogLevel::INFO);
}

TEST_CASE("Device WARN") {
    testDeviceLogLevel(dai::LogLevel::WARN);
}

TEST_CASE("Device ERR") {
    testDeviceLogLevel(dai::LogLevel::ERR);
}

TEST_CASE("Device CRITICAL") {
    testDeviceLogLevel(dai::LogLevel::CRITICAL);
}

TEST_CASE("Device OFF") {
    testDeviceLogLevel(dai::LogLevel::OFF);
}

/**********************************************************************************************************/
/**********************************************************************************************************/
/**********************************************************************************************************/
TEST_CASE("Node TRACE") {
    testNodeLogLevel(dai::LogLevel::TRACE, true);
}

TEST_CASE("Node DEBUG") {
    testNodeLogLevel(dai::LogLevel::DEBUG, false);
}

TEST_CASE("Node INFO") {
    testNodeLogLevel(dai::LogLevel::INFO, true);
}

TEST_CASE("Node WARN") {
    testNodeLogLevel(dai::LogLevel::WARN, false);
}

TEST_CASE("Node ERR") {
    testNodeLogLevel(dai::LogLevel::ERR, false);
}

TEST_CASE("Node CRITICAL") {
    testNodeLogLevel(dai::LogLevel::CRITICAL, true);
}

TEST_CASE("Node OFF") {
    testNodeLogLevel(dai::LogLevel::OFF, true);
}