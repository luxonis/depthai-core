#pragma once

#include <cstdint>
#include <depthai/pipeline/ThreadedNode.hpp>
#include <memory>

// shared
#include <depthai/properties/internal/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

using XLinkOutProperties = ::dai::internal::XLinkOutProperties;

/**
 * @brief Replay node, used to replay a file to a source node
 */
class ReplayVideo : public NodeCRTP<ThreadedHostNode, ReplayVideo> {
   private:
    std::optional<std::tuple<int, int>> size;
    std::optional<float> fps;
    std::filesystem::path replayVideo;
    std::filesystem::path replayFile;
    ImgFrame::Type outFrameType = ImgFrame::Type::YUV420p;

    bool loop = true;

   public:
    constexpr static const char* NAME = "ReplayVideo";

    /**
     * Output for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void run() override;

    /**
     * Get metadata input file path.
     */
    std::filesystem::path getReplayMetadataFile() const;
    /**
     * Get video input file path.
     */
    std::filesystem::path getReplayVideoFile() const;
    /**
     * Get output frame type.
     */
    ImgFrame::Type getOutFrameType() const;
    /**
     * Get output frame size.
     */
    std::tuple<int, int> getSize() const;
    /**
     * Get replay FPS.
     */
    float getFps() const;
    /**
     * Return whether replay loops.
     */
    bool getLoop() const;

    /**
     * Set metadata input file path.
     */
    ReplayVideo& setReplayMetadataFile(const std::filesystem::path& replayFile);
    /**
     * Set video input file path.
     */
    ReplayVideo& setReplayVideoFile(const std::filesystem::path& replayVideo);
    /**
     * Set output frame type.
     */
    ReplayVideo& setOutFrameType(ImgFrame::Type outFrameType);
    /**
     * Set output frame size.
     */
    ReplayVideo& setSize(std::tuple<int, int> size);
    /**
     * Set output frame size.
     */
    ReplayVideo& setSize(int width, int height);
    /**
     * Set replay FPS.
     */
    ReplayVideo& setFps(float fps);
    /**
     * Enable or disable looping.
     */
    ReplayVideo& setLoop(bool loop);
};

/**
 * @brief Replay node, used to replay a file to a source node
 */
class ReplayMetadataOnly : public NodeCRTP<ThreadedHostNode, ReplayMetadataOnly> {
   private:
    std::filesystem::path replayFile;

    std::optional<float> fps;
    bool loop = true;

   public:
    constexpr static const char* NAME = "ReplayMetadataOnly";

    /**
     * Output for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override;

    /**
     * Get replay file path.
     */
    std::filesystem::path getReplayFile() const;
    /**
     * Get replay FPS.
     */
    float getFps() const;
    /**
     * Return whether replay loops.
     */
    bool getLoop() const;

    /**
     * Set replay file path.
     */
    ReplayMetadataOnly& setReplayFile(const std::filesystem::path& replayFile);
    /**
     * Set replay FPS.
     */
    ReplayMetadataOnly& setFps(float fps);
    /**
     * Enable or disable looping.
     */
    ReplayMetadataOnly& setLoop(bool loop);
};

}  // namespace node
}  // namespace dai
