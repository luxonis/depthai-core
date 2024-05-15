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
    std::string replayVideo;
    std::string replayFile;
    ImgFrame::Type outFrameType = ImgFrame::Type::YUV420p;

    bool loop = true;

   public:
    constexpr static const char* NAME = "ReplayVideo";

    /**
     * Output for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    std::string getReplayMetadataFile() const;
    std::string getReplayVideo() const;
    ImgFrame::Type getOutFrameType() const;
    std::tuple<int, int> getSize() const;
    float getFps() const;
    bool getLoop() const;

    ReplayVideo& setReplayMetadataFile(const std::string& replayFile);
    ReplayVideo& setReplayVideo(const std::string& replayVideo);
    ReplayVideo& setOutFrameType(ImgFrame::Type outFrameType);
    ReplayVideo& setSize(std::tuple<int, int> size);
    ReplayVideo& setSize(int width, int height);
    ReplayVideo& setFps(float fps);
    ReplayVideo& setLoop(bool loop);
};

/**
 * @brief Replay node, used to replay a file to a source node
 */
class ReplayMessage : public NodeCRTP<ThreadedHostNode, ReplayMessage> {
   private:
    std::string replayFile;

    bool loop = true;

   public:
    constexpr static const char* NAME = "ReplayMessage";

    /**
     * Output for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    std::string getReplayFile() const;
    bool getLoop() const;

    ReplayMessage& setReplayFile(const std::string& replayFile);
    ReplayMessage& setLoop(bool loop);
};

}  // namespace node
}  // namespace dai
