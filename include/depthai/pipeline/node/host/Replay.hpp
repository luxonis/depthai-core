#pragma once

#include <cstdint>
#include <depthai/pipeline/ThreadedNode.hpp>
#include <memory>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

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
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override;

    std::filesystem::path getReplayMetadataFile() const;
    std::filesystem::path getReplayVideoFile() const;
    ImgFrame::Type getOutFrameType() const;
    std::tuple<int, int> getSize() const;
    float getFps() const;
    bool getLoop() const;

    ReplayVideo& setReplayMetadataFile(const std::filesystem::path& replayFile);
    ReplayVideo& setReplayVideoFile(const std::filesystem::path& replayVideo);
    ReplayVideo& setOutFrameType(ImgFrame::Type outFrameType);
    ReplayVideo& setSize(std::tuple<int, int> size);
    ReplayVideo& setSize(int width, int height);
    ReplayVideo& setFps(float fps);
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

    std::filesystem::path getReplayFile() const;
    float getFps() const;
    bool getLoop() const;

    ReplayMetadataOnly& setReplayFile(const std::filesystem::path& replayFile);
    ReplayMetadataOnly& setFps(float fps);
    ReplayMetadataOnly& setLoop(bool loop);
};

}  // namespace node
}  // namespace dai
