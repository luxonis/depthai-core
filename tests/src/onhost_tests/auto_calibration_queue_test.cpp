#include <catch2/catch_all.hpp>
#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AutoCalibration.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai::node {

struct AutoCalibrationTestAccess {
    struct Result {
        std::shared_ptr<CalibrationHandler> calibration;
        std::vector<std::pair<double, double>> coverages;
    };

    static Result recalibrate(AutoCalibration& node) {
        AutoCalibration::Report report;
        auto calibration = node.getNewCalibration(1, report);
        return {std::move(calibration), std::move(report.coveragesAcquired)};
    }

    static std::shared_ptr<MessageQueue> connect(AutoCalibration& node) {
        node.dynamicCalibration->calibrationOutput.link(node.dynamicCalibrationQueue);
        node.dynamicCalibration->coverageOutput.link(node.coverageQueue);
        return node.dynamicCalibrationCommandQueue.createOutputQueue(4, false);
    }

    static void sendResult(AutoCalibration& node, const std::shared_ptr<DynamicCalibrationResult>& result) {
        node.dynamicCalibration->calibrationOutput.send(result);
    }

    static void sendCoverage(AutoCalibration& node, const std::shared_ptr<CoverageData>& coverage) {
        node.dynamicCalibration->coverageOutput.send(coverage);
    }

    static bool hasPendingResult(AutoCalibration& node) {
        return node.dynamicCalibrationQueue.has();
    }

    static AutoCalibrationProperties& properties(AutoCalibration& node) {
        return node.getProperties();
    }

    static void setGateFps(AutoCalibration& node, int fps) {
        node.gate->initialConfig->fps = fps;
    }

    static void buildStage1(AutoCalibration& node) {
        node.buildStage1();
    }

    static bool dynamicCalibrationRunsOnHost(AutoCalibration& node) {
        return node.dynamicCalibration->runOnHost();
    }
};

}  // namespace dai::node

namespace {

using Access = dai::node::AutoCalibrationTestAccess;
using namespace std::chrono_literals;

class RecalibrationConsumer : public dai::node::AutoCalibration {
   public:
    RecalibrationConsumer() {
        // This test drives the calibration message protocol without a device or camera threads.
        initialConfig->dataConfidenceThreshold = 0.85f;
        initialConfig->maxImagesPerRecalibration = 2;
        buildInternal();
    }

    void run() override {
        try {
            completed.set_value(Access::recalibrate(*this));
        } catch(...) {
            completed.set_exception(std::current_exception());
        }
    }

    std::promise<Access::Result> completed;
};

struct ConsumerFixture {
    dai::Pipeline pipeline{false};
    std::shared_ptr<RecalibrationConsumer> consumer = std::make_shared<RecalibrationConsumer>();

    ConsumerFixture() {
        pipeline.add(consumer);
    }

    ~ConsumerFixture() {
        consumer->stop();
        consumer->wait();
    }
};

}  // namespace

TEST_CASE("AutoCalibration starts recalibration with fresh result and coverage messages", "[AutoCalibration]") {
    const auto stale = GENERATE(std::make_pair(false, false), std::make_pair(true, false), std::make_pair(false, true), std::make_pair(true, true));
    CAPTURE(stale.first, stale.second);

    ConsumerFixture fixture;
    auto& consumer = *fixture.consumer;
    auto commands = Access::connect(consumer);
    if(stale.first) {
        // Rejected validation LoadImage commands emit this result, while loadData consumes only coverage.
        Access::sendResult(consumer, std::make_shared<dai::DynamicCalibrationResult>("Image set skipped: markers not seen by two cameras"));
    }
    if(stale.second) {
        auto coverage = std::make_shared<dai::CoverageData>();
        coverage->coverageAcquired = 0.1f;
        coverage->dataAcquired = 0.2f;
        Access::sendCoverage(consumer, coverage);
    }

    auto completed = consumer.completed.get_future();
    consumer.start();
    bool timedOut = false;
    auto command = commands->get<dai::DynamicCalibrationControl>(5s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(command != nullptr);
    REQUIRE(std::holds_alternative<dai::DynamicCalibrationControl::Commands::StartCalibration>(command->command));

    // Before sending the new pair, let the consumer either discard or consume the old result. Otherwise the
    // size-one nonblocking queue could overwrite it with the new result and hide the regression.
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while(Access::hasPendingResult(consumer) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE_FALSE(Access::hasPendingResult(consumer));

    auto coverage = std::make_shared<dai::CoverageData>();
    coverage->coverageAcquired = 0.75f;
    coverage->dataAcquired = 0.9f;
    Access::sendCoverage(consumer, coverage);

    dai::EepromData eeprom;
    eeprom.boardName = "fresh-calibration";
    dai::DynamicCalibrationResult::Data data;
    data.newCalibration = dai::CalibrationHandler(eeprom);
    data.dataConfidence = 1.0f;
    Access::sendResult(consumer, std::make_shared<dai::DynamicCalibrationResult>(data, "Calibration succeeded"));
    // A successful DynamicCalibration stops producing. No additional coverage will unblock a shifted consumer.
    const bool returned = completed.wait_for(5s) == std::future_status::ready;
    consumer.stop();
    consumer.wait();

    REQUIRE(returned);
    const auto result = completed.get();
    REQUIRE(result.calibration != nullptr);
    REQUIRE(result.calibration->getEepromData().boardName == "fresh-calibration");
    REQUIRE(result.coverages.size() == 1);
    REQUIRE(result.coverages.front().first == Catch::Approx(0.75f));
    REQUIRE(result.coverages.front().second == Catch::Approx(0.9f));
}

// The RVC4 firmware constructs the node from these properties and starts from their config; the construction itself
// needs the firmware build (no subnodes) and is covered by the on-device auto_calibration_test.
TEST_CASE("AutoCalibration serializes its current config and gate rate in the properties", "[AutoCalibration]") {
    const auto type = GENERATE(dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK);
    CAPTURE(static_cast<int>(type));

    dai::Pipeline pipeline{false};
    auto node = pipeline.create<dai::node::AutoCalibration>();
    // Replace the config after create(), as the Python property and the pipeline's implicit node setup do, and
    // edit it once more through the pointer.
    node->initialConfig = std::make_shared<dai::AutoCalibrationConfig>(dai::AutoCalibrationConfig::CONTINUOUS, 7, 0.8, 0.6, 3, 4, 2, false);
    node->initialConfig->maxIterations = 9;
    Access::setGateFps(*node, 3);

    std::vector<std::uint8_t> data;
    Access::properties(*node).serialize(data, type);
    dai::AutoCalibrationProperties decoded;
    REQUIRE(dai::utility::deserialize(data, decoded, type));

    const auto& config = decoded.initialConfig;
    REQUIRE(config.mode == dai::AutoCalibrationConfig::CONTINUOUS);
    REQUIRE(config.sleepingTime == 7);
    REQUIRE(config.calibrationConfidenceThreshold == Catch::Approx(0.8));
    REQUIRE(config.dataConfidenceThreshold == Catch::Approx(0.6));
    REQUIRE(config.maxIterations == 9);
    REQUIRE(config.maxImagesPerRecalibration == 4);
    REQUIRE(config.validationSetSize == 2);
    REQUIRE_FALSE(config.flashCalibration);
    REQUIRE(decoded.gateFps == 3);
}

TEST_CASE("AutoCalibration keeps the lens dependent threshold unresolved until it runs", "[AutoCalibration]") {
    dai::Pipeline pipeline{false};
    auto node = pipeline.create<dai::node::AutoCalibration>();
    // Resolved in run(), on the side where the node runs: a node created before a device is bound must not
    // serialize a fallback value.
    REQUIRE(node->initialConfig->dataConfidenceThreshold < 0.0);
    REQUIRE(Access::properties(*node).initialConfig.dataConfidenceThreshold < 0.0);
}

TEST_CASE("AutoCalibration runs on the host by default and its DynamicCalibration follows", "[AutoCalibration]") {
    dai::Pipeline pipeline{false};
    auto node = pipeline.create<dai::node::AutoCalibration>();
    REQUIRE(node->runOnHost());
    Access::buildStage1(*node);
    REQUIRE(Access::dynamicCalibrationRunsOnHost(*node));

    // Without a device the platform is checked when the pipeline is built.
    node->setRunOnHost(false);
    REQUIRE_FALSE(node->runOnHost());
    Access::buildStage1(*node);
    REQUIRE_FALSE(Access::dynamicCalibrationRunsOnHost(*node));

    node->setRunOnHost(true);
    Access::buildStage1(*node);
    REQUIRE(node->runOnHost());
    REQUIRE(Access::dynamicCalibrationRunsOnHost(*node));
}
