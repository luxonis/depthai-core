#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <depthai/depthai.hpp>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {
constexpr std::pair<std::uint32_t, std::uint32_t> kAutoCalibrationRequiredResolution{1280u, 800u};

bool hasResolution(const std::shared_ptr<dai::Device>& device, dai::CameraBoardSocket socket, std::pair<std::uint32_t, std::uint32_t> resolution) {
    const auto features = device->getConnectedCameraFeatures();
    for(const auto& feature : features) {
        if(feature.socket != socket) continue;
        for(const auto& cfg : feature.configs) {
            if(cfg.width == resolution.first && cfg.height == resolution.second) {
                return true;
            }
        }
    }
    return false;
}

dai::Pipeline makePipeline(const std::shared_ptr<dai::Device>& device, std::shared_ptr<dai::node::AutoCalibration>& autoCalibration) {
    // Construct pipeline bound to the device
    dai::Pipeline p(device);

    // Cameras via .build(socket)
    auto camLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, kAutoCalibrationRequiredResolution);
    auto camRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, kAutoCalibrationRequiredResolution);

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

        if(!hasResolution(device, dai::CameraBoardSocket::CAM_B, kAutoCalibrationRequiredResolution)
           || !hasResolution(device, dai::CameraBoardSocket::CAM_C, kAutoCalibrationRequiredResolution)) {
            SKIP("Skipping AutoCalibration test: CAM_B/C does not advertise 1280x800 required by AutoCalibration.");
        }

        std::atomic<bool> sawUnsupportedResolutionWarning{false};
        device->setLogLevel(dai::LogLevel::WARN);
        device->addLogCallback([&](const dai::LogMessage& m) {
            if(m.level >= dai::LogLevel::WARN && m.payload.find("supports only sensors with 1280x800 resolution") != std::string::npos) {
                sawUnsupportedResolutionWarning = true;
            }
        });

        std::shared_ptr<dai::node::AutoCalibration> autoCalibration;
        auto pipeline = makePipeline(device, autoCalibration);
        REQUIRE(autoCalibration);
        autoCalibration->initialConfig = std::make_shared<dai::AutoCalibrationConfig>(config);

        // Queues
        auto output = autoCalibration->output.createOutputQueue();

        pipeline.start();

        std::this_thread::sleep_for(10s);

        pipeline.stop();
        pipeline.wait();

        REQUIRE_FALSE(sawUnsupportedResolutionWarning);
    }
}
