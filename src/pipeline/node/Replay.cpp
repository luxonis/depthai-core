#include "depthai/pipeline/node/Replay.hpp"

#include <memory>

#include "depthai/utility/RecordReplay.hpp"
#include "pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/datatype/IMUData.hpp"

namespace dai {
namespace node {

void Replay::build() {
    hostNode = true;
}

Buffer getMessage(std::vector<uint8_t>& frame) {
    // TODO
    return {};
}

void Replay::start() {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoPlayer = std::make_shared<VideoPlayer>();
#else
    throw std::runtime_error("Replay node requires OpenCV support");
#endif
}
void Replay::run() {
    if(replayFile.empty()) {
        throw std::runtime_error("Replay replayFile must be set");
    }
    videoPlayer->init(replayFile);
    while(isRunning()) {
        // auto frame = videoPlayer->next();
        // if(frame.empty()) {
        //     // TODO: error
        //     break;
        // }
        // auto msg = std::make_shared<Buffer>(getMessage(frame));
        // out.send(msg);
    }
}
void Replay::stop() {
    videoPlayer->close();
}

Replay& Replay::setReplayFile(const std::string& replayFile) {
    this->replayFile = replayFile;
    return *this;
}

}  // namespace node
}  // namespace dai
