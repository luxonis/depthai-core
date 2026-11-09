#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstddef>
#include <memory>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/ToF.hpp"

using namespace dai;

namespace {

bool deviceHasTofSensor(const std::shared_ptr<Device>& device) {
    for(const auto& features : device->getConnectedCameraFeatures()) {
        for(const auto type : features.supportedTypes) {
            if(type == CameraSensorType::TOF) return true;
        }
    }
    return false;
}

/** Stops the pipeline on scope exit if it is still running. */
struct PipelineStopGuard {
    Pipeline& pipeline;
    explicit PipelineStopGuard(Pipeline& p) : pipeline(p) {}
    ~PipelineStopGuard() {
        if(pipeline.isRunning()) pipeline.stop();
    }
    PipelineStopGuard(const PipelineStopGuard&) = delete;
    PipelineStopGuard& operator=(const PipelineStopGuard&) = delete;
};

constexpr auto FRAME_TIMEOUT = std::chrono::seconds(15);
constexpr size_t PHASE_BYTES_PER_PIXEL = 6;  // int8 (cos, sin) for each of the 3 modulation frequencies

}  // namespace

TEST_CASE("ToF FUSION depth mode sends phase frames matching the depth frame", "[ToF][fusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) {
        SKIP("Requires an RVC4 device with a ToF sensor.");
    }
    if(!device->hasGPU()) {
        SKIP("FUSION depth mode needs the GPU; without it the node falls back to the vendor unwrap and sends no phase.");
    }

    Pipeline pipeline(device);
    auto tof = pipeline.create<node::ToF>()->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, 30.0f);
    tof->tofBase->initialConfig->setDepthMode(ToFConfig::DepthMode::FUSION);

    auto depthQueue = tof->depth.createOutputQueue();
    auto confidenceQueue = tof->confidence.createOutputQueue();
    auto phaseQueue = tof->phase.createOutputQueue();

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    bool timedOut = false;
    const auto depth = depthQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    REQUIRE(depth->getType() == ImgFrame::Type::RAW16);

    timedOut = false;
    const auto confidence = confidenceQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(confidence != nullptr);
    REQUIRE(confidence->getWidth() == depth->getWidth());
    REQUIRE(confidence->getHeight() == depth->getHeight());

    // The first frame is the TNR start-up frame (vendor unwrap, no phase); the next ones come from the spatial unwrap.
    timedOut = false;
    const auto phase = phaseQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(phase != nullptr);
    REQUIRE(phase->getType() == ImgFrame::Type::RAW8);
    REQUIRE(phase->getWidth() == depth->getWidth() * PHASE_BYTES_PER_PIXEL);
    REQUIRE(phase->getHeight() == depth->getHeight());
    REQUIRE(phase->getStride() == phase->getWidth());
    REQUIRE(phase->getData().size() == static_cast<size_t>(depth->getWidth()) * depth->getHeight() * PHASE_BYTES_PER_PIXEL);
}

TEST_CASE("ToF STANDARD depth mode accepts a phase link on RVC4", "[ToF][fusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) {
        SKIP("Requires an RVC4 device with a ToF sensor.");
    }

    Pipeline pipeline(device);
    auto tof = pipeline.create<node::ToF>()->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, 30.0f);
    auto depthQueue = tof->depth.createOutputQueue();
    auto phaseQueue = tof->phase.createOutputQueue();

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    bool timedOut = false;
    const auto depth = depthQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    // STANDARD mode never sends on the phase output.
    REQUIRE(phaseQueue->tryGet<ImgFrame>() == nullptr);
}
