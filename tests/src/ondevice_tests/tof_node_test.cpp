#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>

#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/ToFPreset.hpp"
#include "depthai/common/ToFSensorMode.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ToF.hpp"

namespace {

bool deviceHasToF(const std::shared_ptr<dai::Device>& device) {
    for(const auto& feature : device->getConnectedCameraFeatures()) {
        if(std::find(feature.supportedTypes.begin(), feature.supportedTypes.end(), dai::CameraSensorType::TOF)
           != feature.supportedTypes.end()) {
            return true;
        }
    }
    return false;
}

dai::CameraBoardSocket findToFSocket(const std::shared_ptr<dai::Device>& device) {
    for(const auto& feature : device->getConnectedCameraFeatures()) {
        if(std::find(feature.supportedTypes.begin(), feature.supportedTypes.end(), dai::CameraSensorType::TOF)
           != feature.supportedTypes.end()) {
            return feature.socket;
        }
    }
    return dai::CameraBoardSocket::AUTO;
}

}  // namespace

TEST_CASE("ToF output reference aliases by platform", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);

    auto tof = pipeline.create<dai::node::ToF>();

    if(device->getPlatform() == dai::Platform::RVC4) {
        REQUIRE(&tof->depth == &tof->rawDepth);
    } else {
        REQUIRE(&tof->depth != &tof->rawDepth);
        REQUIRE(&tof->confidence == &tof->amplitude);
    }
}

TEST_CASE("ToF build rejects sensorMode on RVC2", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() == dai::Platform::RVC4) {
        WARN("Skipping RVC2-only build validation on RVC4 device");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    dai::ToFBuildOptions options;
    options.sensorMode = dai::ToFSensorMode::F3_FULL;
    REQUIRE_THROWS_AS(tof->build(options), std::runtime_error);
}

TEST_CASE("ToF build rejects double build", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        dai::ToFBuildOptions options;
        options.fps = 10.f;
        options.preset = dai::ToFPreset::MID_RANGE;
        tof->build(options);
    } else {
        tof->build(dai::CameraBoardSocket::AUTO, dai::ImageFiltersPresetMode::TOF_MID_RANGE);
    }

    REQUIRE_THROWS_AS(tof->build(dai::CameraBoardSocket::AUTO, dai::ImageFiltersPresetMode::TOF_MID_RANGE), std::runtime_error);
}

TEST_CASE("ToF build preserves initialConfig when preset omitted on RVC4", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping RVC4-only preset behavior test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    tof->tofBaseNode.initialConfig->phaseUnwrapErrorThreshold = 123;
    tof->tofBaseNode.initialConfig->enableRadialToPerp = false;

    dai::ToFBuildOptions options;
    options.fps = 10.f;
    options.sensorMode = dai::ToFSensorMode::F3_FULL;
    tof->build(options);

    REQUIRE(tof->tofBaseNode.initialConfig->phaseUnwrapErrorThreshold == 123);
    REQUIRE(tof->tofBaseNode.initialConfig->enableRadialToPerp.has_value());
    REQUIRE(*tof->tofBaseNode.initialConfig->enableRadialToPerp == false);
}

TEST_CASE("ToF build applies preset when explicitly set on RVC4", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping RVC4-only preset behavior test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    tof->tofBaseNode.initialConfig->phaseUnwrapErrorThreshold = 123;

    dai::ToFBuildOptions options;
    options.fps = 10.f;
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    REQUIRE(tof->tofBaseNode.initialConfig->phaseUnwrapErrorThreshold == 75);
}

TEST_CASE("ToF resolution helpers after build on RVC4", "[ToF][Device][API]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping RVC4 resolution helper test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    dai::ToFBuildOptions options;
    options.sensorMode = dai::ToFSensorMode::F3_BINNING_2X2;
    options.fps = 10.f;
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    REQUIRE(tof->getOutputResolution() == dai::getToFSensorModeOutputResolution(dai::ToFSensorMode::F3_BINNING_2X2));
    REQUIRE(tof->getRawResolution() == dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F3_BINNING_2X2));
    REQUIRE(tof->getSensorResolution() == tof->getOutputResolution());
}

TEST_CASE("ToF auto-camera streams IPP depth on RVC4", "[ToF][Device][Stream]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping RVC4 ToF streaming test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>();
    dai::ToFBuildOptions options;
    options.sensorMode = dai::ToFSensorMode::F3_FULL;
    options.fps = 10.f;
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    const auto depthQueue = tof->depth.createOutputQueue(4, false);
    pipeline.start();

    REQUIRE(tof->getCamera() != nullptr);
    REQUIRE(tof->getBoardSocket() != dai::CameraBoardSocket::AUTO);

    const auto [expectedW, expectedH] = tof->getOutputResolution();
    bool timedOut = false;
    const auto depth = depthQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    REQUIRE(depth->getWidth() == expectedW);
    REQUIRE(depth->getHeight() == expectedH);
}

TEST_CASE("ToF manual rawInput skips auto-camera on RVC4", "[ToF][Device][Stream]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping RVC4 manual rawInput test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    const auto tofSocket = findToFSocket(device);
    auto tof = pipeline.create<dai::node::ToF>();
    dai::ToFBuildOptions options;
    options.boardSocket = tofSocket;
    options.sensorMode = dai::ToFSensorMode::F3_FULL;
    options.fps = 10.f;
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    auto camera = pipeline.create<dai::node::Camera>();
    camera->setSensorType(dai::CameraSensorType::TOF);
    const auto [rawW, rawH] = tof->getRawResolution();
    camera->build(tof->getBoardSocket(), std::make_pair(rawW, rawH), 10.f);
    camera->raw.link(tof->rawInput);

    const auto depthQueue = tof->depth.createOutputQueue(4, false);
    pipeline.start();

    REQUIRE(tof->getCamera() == nullptr);

    const auto [expectedW, expectedH] = tof->getOutputResolution();
    bool timedOut = false;
    const auto depth = depthQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    REQUIRE(depth->getWidth() == expectedW);
    REQUIRE(depth->getHeight() == expectedH);
}

TEST_CASE("ToF confidence queue receives amplitude data on RVC2", "[ToF][Device][Stream]") {
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC2) {
        WARN("Skipping RVC2 confidence alias streaming test");
        return;
    }
    if(!deviceHasToF(device)) {
        WARN("Skipping: device has no ToF sensor");
        return;
    }

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ImageFiltersPresetMode::TOF_MID_RANGE);
    REQUIRE(&tof->confidence == &tof->amplitude);

    const auto confQueue = tof->confidence.createOutputQueue(4, false);
    pipeline.start();

    bool timedOut = false;
    const auto frame = confQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    REQUIRE(frame->getWidth() > 0);
    REQUIRE(frame->getHeight() > 0);
}
