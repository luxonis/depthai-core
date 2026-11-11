#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "buffer_test_utils.hpp"
#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

using buffer_test::requireNoMessage;
using camera_test::nextFrame;
using camera_test::send;

namespace {

constexpr float WANTED_FPS = 20.0f;

struct ControlledCamera {
    dai::CameraBoardSocket socket;
    float fps;
    std::shared_ptr<dai::MessageQueue> frames;
    std::shared_ptr<dai::InputQueue> control;
};

// Starts every connected camera paused, with auto exposure locked, at 20 FPS or the lowest rate of its first config
std::vector<ControlledCamera> pausedCameras(dai::Pipeline& p) {
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    std::vector<ControlledCamera> cameras;
    for(const auto& feature : features) {
        REQUIRE_FALSE(feature.configs.empty());
        const bool supported =
            std::any_of(feature.configs.begin(), feature.configs.end(), [](const auto& config) { return camera_test::supportsFps(config, WANTED_FPS); });
        const float fps = supported ? WANTED_FPS : feature.configs.front().minFps;
        auto camera = p.create<dai::node::Camera>()->build(feature.socket, std::nullopt, fps);
        camera->initialControl.setStopStreaming().setAutoExposureLock(true);
        auto* output = camera->requestOutput({320, 240}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps);
        REQUIRE(output != nullptr);
        cameras.push_back({feature.socket, fps, output->createOutputQueue(8, false), camera->inputControl.createInputQueue()});
    }
    return cameras;
}

// Starts the pipeline and then every camera. A paused camera can stall the other camera of a synchronized pair, so every camera starts before any is tested
std::vector<ControlledCamera> startedCameras(dai::Pipeline& p) {
    auto cameras = pausedCameras(p);
    p.start();
    for(auto& camera : cameras) {
        CAPTURE(camera.socket);
        requireNoMessage(camera.frames);
    }
    for(auto& camera : cameras) {
        send(camera.control, dai::CameraControl().setAutoExposureEnable());
        send(camera.control, dai::CameraControl().setAutoExposureLock(true));
        send(camera.control, dai::CameraControl().setStartStreaming());
    }
    for(auto& camera : cameras) {
        CAPTURE(camera.socket);
        nextFrame(camera.frames);
    }
    return cameras;
}

// Frame rate over 16 intervals of device timestamps
double measuredFps(const std::shared_ptr<dai::MessageQueue>& frames) {
    constexpr int INTERVALS = 16;
    const auto first = nextFrame(frames)->getTimestampDevice();
    auto last = first;
    for(int i = 0; i < INTERVALS; ++i) last = nextFrame(frames)->getTimestampDevice();
    return INTERVALS / std::chrono::duration<double>(last - first).count();
}

}  // namespace

TEST_CASE("Auto exposure changes keep the configured frame rate", "[camera-hil]") {
    dai::Pipeline p;
    auto cameras = startedCameras(p);
    for(auto& camera : cameras) {
        CAPTURE(camera.socket, camera.fps);
        const std::vector<std::pair<std::string, dai::CameraControl>> requests{
            {"auto exposure", dai::CameraControl().setAutoExposureEnable()},
            {"auto exposure lock", dai::CameraControl().setAutoExposureLock(true)},
            {"auto exposure unlock", dai::CameraControl().setAutoExposureLock(false)},
        };
        for(const auto& [name, request] : requests) {
            CAPTURE(name);
            camera_test::drain(camera.frames);
            send(camera.control, request);
            for(int i = 0; i < 6; ++i) nextFrame(camera.frames);
            REQUIRE(measuredFps(camera.frames) == Catch::Approx(camera.fps).margin(camera.fps * 0.15));
        }
    }
}

TEST_CASE("Rejected controls keep frames coming", "[camera-hil]") {
    dai::Pipeline p;
    auto cameras = startedCameras(p);
    for(auto& camera : cameras) {
        CAPTURE(camera.socket);
        // Each request is outside what the camera accepts: autofocus without a lens range, an exposure longer than half the frame period,
        // a brightness above the public range of -10 to 10, and a lens position that is not a number
        const std::vector<std::pair<std::string, dai::CameraControl>> requests{
            {"autofocus", dai::CameraControl().setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO)},
            {"manual exposure", dai::CameraControl().setManualExposure(49000, 100)},
            {"brightness", dai::CameraControl().setBrightness(11)},
            {"manual focus", dai::CameraControl().setManualFocusRaw(std::numeric_limits<float>::quiet_NaN())},
        };
        for(const auto& [name, request] : requests) {
            CAPTURE(name);
            camera_test::drain(camera.frames);
            auto previous = nextFrame(camera.frames);
            send(camera.control, request);
            send(camera.control, dai::CameraControl().setAutoExposureEnable());
            for(int i = 0; i < 10; ++i) {
                auto frame = nextFrame(camera.frames);
                REQUIRE(frame->getSequenceNum() > previous->getSequenceNum());
                REQUIRE(frame->getTimestampDevice() > previous->getTimestampDevice());
                previous = frame;
            }
        }
    }
}

TEST_CASE("Exposure controls sent while paused do not restart the camera", "[camera-hil]") {
    using namespace std::chrono_literals;
    dai::Pipeline p;
    auto cameras = startedCameras(p);
    for(auto& camera : cameras) {
        CAPTURE(camera.socket);
        send(camera.control, dai::CameraControl().setStopStreaming());
        std::this_thread::sleep_for(1s);
        camera_test::drain(camera.frames);
        requireNoMessage(camera.frames);

        send(camera.control, dai::CameraControl().setAutoExposureLock(true));
        send(camera.control, dai::CameraControl().setAutoExposureEnable());
        requireNoMessage(camera.frames);

        send(camera.control, dai::CameraControl().setStartStreaming());
        nextFrame(camera.frames);
    }
}
