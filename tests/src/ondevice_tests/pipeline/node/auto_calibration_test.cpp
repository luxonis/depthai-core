#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <iomanip>
#include <memory>
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
