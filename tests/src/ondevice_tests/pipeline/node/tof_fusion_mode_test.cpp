#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
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

TEST_CASE("ToF with a linked phase output sends phase frames matching the depth frame", "[ToF][fusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) {
        SKIP("Requires an RVC4 device with a ToF sensor.");
    }
    if(!device->hasGPU()) {
        SKIP("The fusion depth mode needs the GPU; without it the node falls back to the vendor unwrap and sends empty phase frames.");
    }

    Pipeline pipeline(device);
    auto tof = pipeline.create<node::ToF>()->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, 30.0f);

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

    // The first frame is the TNR start-up frame (vendor unwrap: an empty phase frame); the next ones come from the spatial
    // unwrap.
    std::shared_ptr<ImgFrame> phase;
    const auto deadline = std::chrono::steady_clock::now() + FRAME_TIMEOUT;
    while(std::chrono::steady_clock::now() < deadline) {
        timedOut = false;
        phase = phaseQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(phase != nullptr);
        if(!phase->getData().empty()) break;
        REQUIRE(phase->getWidth() == 0);
    }
    REQUIRE(phase != nullptr);
    REQUIRE(phase->getType() == ImgFrame::Type::RAW8);
    REQUIRE(phase->getWidth() == depth->getWidth() * PHASE_BYTES_PER_PIXEL);
    REQUIRE(phase->getHeight() == depth->getHeight());
    REQUIRE(phase->getStride() == phase->getWidth());
    REQUIRE(phase->getData().size() == static_cast<size_t>(depth->getWidth()) * depth->getHeight() * PHASE_BYTES_PER_PIXEL);
}

TEST_CASE("ToF without the phase output keeps the standard depth range on RVC4", "[ToF][fusion]") {
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) {
        SKIP("Requires an RVC4 device with a ToF sensor.");
    }

    Pipeline pipeline(device);
    auto tof = pipeline.create<node::ToF>()->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, 30.0f);
    auto depthQueue = tof->depth.createOutputQueue();

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    // The standard mode caps the depth below 6.5 m (the fusion mode keeps up to 15 m).
    constexpr uint16_t STANDARD_MAX_DEPTH_MM = 6500;
    for(int i = 0; i < 5; ++i) {
        bool timedOut = false;
        const auto depth = depthQueue->get<ImgFrame>(FRAME_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(depth != nullptr);
        REQUIRE(depth->getType() == ImgFrame::Type::RAW16);
        const auto* data = reinterpret_cast<const uint16_t*>(depth->getData().data());
        const size_t count = static_cast<size_t>(depth->getWidth()) * depth->getHeight();
        REQUIRE(*std::max_element(data, data + count) < STANDARD_MAX_DEPTH_MM);
    }
}
