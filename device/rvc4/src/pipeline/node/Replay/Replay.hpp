#pragma once

#include <filesystem>
#include <memory>

#include "depthai/pipeline/node/host/Replay.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

/**
 * Device-side ReplayVideo. The host uploads the source and sends resolved properties.
 * H.264 and H.265 are decoded to linear NV12. FFV1, MJPEG, JPEG, and PNG use the software GStreamer elements.
 * This class needs the ReplayVideo DeviceNode from the record-replay depthai-core, not the older host-only node.
 */
class ReplayVideo : public ::dai::node::ReplayVideo {
   public:
    constexpr static const char* NAME = ::dai::node::ReplayVideo::NAME;

    ReplayVideo();
    explicit ReplayVideo(std::unique_ptr<Properties> props);
    ~ReplayVideo() override;

    bool runOnHost() const override {
        return true;
    }
    void buildStage1() override;
    void run() override;
    void stop() override;

   private:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    struct State;
    friend void readMetadata(const std::filesystem::path& path, State& state, int frameCount);
    friend void requireFrameCount(const State& state, int frameCount);
    std::unique_ptr<State> state;
};

class ReplayMetadataOnly : public ::dai::node::ReplayMetadataOnly {
   public:
    constexpr static const char* NAME = ::dai::node::ReplayMetadataOnly::NAME;

    ReplayMetadataOnly();
    explicit ReplayMetadataOnly(std::unique_ptr<Properties> props);
    ~ReplayMetadataOnly() override;

    bool runOnHost() const override {
        return true;
    }
    void buildStage1() override;
    void run() override;

   private:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    struct State;
    std::unique_ptr<State> state;
};

}  // namespace gate
}  // namespace dai
