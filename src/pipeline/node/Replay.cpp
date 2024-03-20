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

void Replay::start() {
#if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    videoPlayer = std::make_unique<utility::VideoPlayerOpenCV>();
#else
    videoPlayer = std::make_unique<utility::VideoPlayerMp4v2>();
#endif
}
void Replay::run() {
    if(replayFile.empty()) {
        throw std::runtime_error("Replay replayFile must be set");
    }
//     // TODO(asahtik): EncodedFrame and byte writer (Buffer, IMUData)
//     while(isRunning()) {
//         auto msg = input.queue.get<dai::Buffer>();
//         if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
//             auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
//             // TODO(asahtik): Somehow seet framerate 
//             videoPlayer->init(recordFile, imgFrame->getWidth(), imgFrame->getHeight(), 30);
// #if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
//             auto frame = imgFrame->getCvFrame();
//             span data(frame.data, frame.total() * frame.elemSize());
//             videoPlayer->write(imgFrame->getCvFrame().data);
// #else
//             auto data = imgFrame->getReplayData();
//             videoPlayer->write(data);
// #endif
//         } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
//             throw std::runtime_error("TODO: EncodedFrame recording is not supported yet");
//         } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
//             throw std::runtime_error("TODO: IMUData recording is not supported yet");
//         } else {
//             throw std::runtime_error("TODO: General Buffer recording is not supported yet");
//         }
//     }
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
