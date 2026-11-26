#pragma once

#include <spdlog/async_logger.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "camera/CameraSensor.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

// Thermal node on rvc4, for the Seek Eagle thermal core: CameraSensor captures its raw wire frames and the Seek SDK
// (SeekEagle.cpp) turns them into the temperature (GRAYF16, degrees Celsius), color (YUV422i) and raw (RAW16) outputs.
class Thermal : public node::Thermal {
   public:
    constexpr static const char* NAME = node::Thermal::NAME;

    Thermal() = delete;
    Thermal(std::unique_ptr<Properties> props) : node::Thermal(std::move(props)) {}

    bool runOnHost() const override {
        return true;
    }

    void buildStage2() override;
    void buildStage3() override;
    void run() override;
    void stop() override;

   private:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    std::unique_ptr<CameraSensor> cameraSensor;
    CameraBoardSocket socket{CameraBoardSocket::AUTO};
    std::string deviceId;
    // transformation of the output frames, made for the size of the first frame
    ImgTransformation transformation;
    std::pair<int, int> transformationSize{0, 0};
    // timestamps of the last SDK frame, the same for both of its outputs
    struct Timestamps {
        uint64_t seq{UINT64_MAX};
        std::chrono::nanoseconds device{}, host{};
        std::optional<std::chrono::nanoseconds> system;
    } timestamps;

    void sendFrame(CameraSensor::FrameCallbackArgs args);
};

}  // namespace gate
}  // namespace dai
