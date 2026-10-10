#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("Test reconnecting to the pipeline multiple times") {
    constexpr auto TIMES_TO_RUN = 10;
    for(int i = 0; i < TIMES_TO_RUN; i++) {
        dai::Pipeline p;
        auto camera = p.create<dai::node::Camera>()->build();
        auto* cameraOutput = camera->requestOutput(std::make_pair(640, 400));
        REQUIRE(cameraOutput != nullptr);
        auto outputQueue = cameraOutput->createOutputQueue();
        p.start();
        // Wait for the first image
        bool hasTimedOut = false;
        auto img = outputQueue->get<dai::ImgFrame>(std::chrono::duration<double>(15), hasTimedOut);
        REQUIRE(!hasTimedOut);
        REQUIRE(img != nullptr);
    }
}

TEST_CASE("Initially stopped synchronized cameras feeding StereoDepth do not crash", "[paired-camera-stop]") {
    using namespace std::chrono_literals;
    const bool rightFirst = GENERATE(false, true);
    const bool staggeredStart = GENERATE(false, true);
    CAPTURE(rightFirst, staggeredStart);

    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    if(device->getPlatform() != dai::Platform::RVC2) {
        SKIP("This regression requires RVC2 synchronized OV9282 cameras");
    }
    const auto features = device->getConnectedCameraFeatures();
    for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        if(std::none_of(features.begin(), features.end(), [socket](const auto& camera) { return camera.socket == socket && camera.sensorName == "OV9282"; })) {
            SKIP("This regression requires OV9282 cameras on CAM_B and CAM_C");
        }
    }
    device->setMaxReconnectionAttempts(0);

    std::array<dai::Node::Output*, 2> outputs;
    std::vector<std::shared_ptr<dai::InputQueue>> controls;
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    size_t index = 0;
    for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        auto camera = p.create<dai::node::Camera>()->build(socket);
        camera->initialControl.setStopStreaming();
        outputs[index] = camera->requestOutput({640, 400}, std::nullopt, dai::ImgResizeMode::CROP, 30.0f);
        REQUIRE(outputs[index] != nullptr);
        controls.push_back(camera->inputControl.createInputQueue());
        ++index;
    }
    for(auto* output : outputs) {
        queues.push_back(output->createOutputQueue());
    }
    auto stereo = p.create<dai::node::StereoDepth>()->build(*outputs[0], *outputs[1]);
    queues.push_back(stereo->depth.createOutputQueue());

    p.start();
    std::this_thread::sleep_for(5s);
    REQUIRE_FALSE(device->hasCrashed());
    REQUIRE_FALSE(device->isClosed());
    REQUIRE(device->isPipelineRunning());
    // Initialization can leave frames queued before initialControl takes effect.
    for(auto& queue : queues) {
        while(queue->tryGet<dai::ImgFrame>() != nullptr) {
        }
    }
    auto requireNoFrames = [&](const char* phase) {
        INFO(phase);
        for(size_t i = 0; i < queues.size(); ++i) {
            CAPTURE(i);
            bool timedOut = false;
            auto frame = queues[i]->get<dai::ImgFrame>(100ms, timedOut);
            REQUIRE(timedOut);
            REQUIRE(frame == nullptr);
        }
    };
    requireNoFrames("Before either START command");

    for(size_t i = 0; i < controls.size(); ++i) {
        auto& control = controls[rightFirst ? controls.size() - 1 - i : i];
        auto start = std::make_shared<dai::CameraControl>();
        start->setStartStreaming();
        control->send(start);
        if(i == 0 && staggeredStart) {
            // One ready receiver must not start the synchronized sensor pair.
            std::this_thread::sleep_for(1s);
            requireNoFrames("Before the second START command");
        }
    }
    for(auto& queue : queues) {
        bool timedOut = false;
        auto frame = queue->get<dai::ImgFrame>(5s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
    }
    REQUIRE_FALSE(device->hasCrashed());
}
