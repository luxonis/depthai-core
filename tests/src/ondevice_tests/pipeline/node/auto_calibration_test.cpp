#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {
dai::Pipeline makePipeline(const std::shared_ptr<dai::Device>& device, std::shared_ptr<dai::node::AutoCalibration>& autoCalibration) {
    // Construct pipeline bound to the device
    dai::Pipeline p(device);

    // Cameras via .build(socket)
    auto camLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    autoCalibration = p.create<dai::node::AutoCalibration>()->build(camLeft, camRight);

    return p;
}
}  // namespace

TEST_CASE("AutoCalibration: Do not crash") {
    std::vector<dai::AutoCalibrationConfig> configs = {dai::AutoCalibrationConfig{dai::AutoCalibrationConfig::ON_START, 10, 0.9f, 0.7f, 20, 20, 2, true},
                                                       dai::AutoCalibrationConfig{dai::AutoCalibrationConfig::CONTINUOUS, 10, 0.9f, 0.7f, 20, 20, 2, false},
                                                       dai::AutoCalibrationConfig{dai::AutoCalibrationConfig::CONTINUOUS, 0, 0.9f, 0.7f, 20, 20, 2, true}};
    for(auto config : configs) {
        auto device = std::make_shared<dai::Device>();
        REQUIRE(device != nullptr);

        std::atomic<bool> sawWarnOrError{false};
        device->setLogLevel(dai::LogLevel::WARN);
        device->addLogCallback([&](const dai::LogMessage& m) {
            if(m.level >= dai::LogLevel::WARN) sawWarnOrError = true;
        });

        std::shared_ptr<dai::node::AutoCalibration> autoCalibration;
        auto pipeline = makePipeline(device, autoCalibration);
        autoCalibration->initialConfig = std::make_shared<dai::AutoCalibrationConfig>(config);
        REQUIRE(autoCalibration);

        // Queues
        auto output = autoCalibration->output.createOutputQueue();

        pipeline.start();

        std::this_thread::sleep_for(10s);

        pipeline.stop();
        pipeline.wait();
    }
}

TEST_CASE("AutoCalibration: pipeline returns status") {
    auto device = std::make_shared<dai::Device>();
    REQUIRE(device != nullptr);

    std::shared_ptr<dai::node::AutoCalibration> autoCalibration;
    auto pipeline = makePipeline(device, autoCalibration);
    REQUIRE(autoCalibration);

    autoCalibration->initialConfig = std::make_shared<dai::AutoCalibrationConfig>(
        dai::AutoCalibrationConfig{dai::AutoCalibrationConfig::ON_START, 10, 0.9f, 0.7f, 20, 20, 2, true});

    pipeline.start();

    std::optional<dai::AutoCalibrationStatus> status;
    constexpr auto timeout = 10s;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        status = pipeline.getAutoCalibrationStatus();
        if(status.has_value()) {
            break;
        }
        std::this_thread::sleep_for(100ms);
    }

    REQUIRE(status.has_value());
    REQUIRE(status->dataConfidence >= 0.0);
    REQUIRE(status->dataConfidence <= 1.0);
    REQUIRE(status->calibrationConfidence >= 0.0);
    REQUIRE(status->calibrationConfidence <= 1.0);

    const auto st = status->status;
    const bool validStatus = st == dai::AutoCalibrationExecutionStatus::IDLE || st == dai::AutoCalibrationExecutionStatus::RUNNING
                             || st == dai::AutoCalibrationExecutionStatus::VALIDATING_INPUT
                             || st == dai::AutoCalibrationExecutionStatus::CALIBRATING || st == dai::AutoCalibrationExecutionStatus::TIMEOUT
                             || st == dai::AutoCalibrationExecutionStatus::INVALID_INPUT || st == dai::AutoCalibrationExecutionStatus::SUCCEEDED
                             || st == dai::AutoCalibrationExecutionStatus::FAILED || st == dai::AutoCalibrationExecutionStatus::STOPPED;
    REQUIRE(validStatus);

    pipeline.stop();
    pipeline.wait();
}
