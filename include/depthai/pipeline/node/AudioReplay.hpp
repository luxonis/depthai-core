#pragma once

// depthai
#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {
namespace node {

class AudioReplay : public NodeCRTP<ThreadedHostNode, AudioReplay> {
   public:
    constexpr static const char* NAME = "AudioReplay";

    void run() override;

    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    AudioReplay& setSourceFile(const std::filesystem::path& sourceFile);
    AudioReplay& setLoop(bool loop);
    AudioReplay& setFps(int fps);
    std::filesystem::path getSourceFile() const;
    bool getLoop() const;
    int getFps() const;
   private:
    int fps = 10;
    std::filesystem::path sourceFile;
    bool loop = false;
};

}  // namespace node
}  // namespace dai
