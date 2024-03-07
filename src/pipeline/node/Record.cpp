#include "depthai/pipeline/node/Record.hpp"

#include <memory>

#include "depthai/utility/RecordReplay.hpp"
#include "pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/datatype/IMUData.hpp"

namespace dai {
namespace node {

void Record::build() {
    hostNode = true;
}

void Record::start() {
#if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    videoRecorder = std::make_unique<utility::VideoRecorderOpenCV>();
#else
    videoRecorder = std::make_unique<utility::VideoRecorderMp4v2>();
#endif
}
void Record::run() {
    if(recordFile.empty()) {
        throw std::runtime_error("Record recordPath must be set");
    }
    // TODO(asahtik): EncodedFrame and byte writer (Buffer, IMUData)
    while(isRunning()) {
        auto msg = input.queue.get<dai::Buffer>();
        if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
            auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
            // TODO(asahtik): Somehow seet framerate 
            videoRecorder->init(recordFile, imgFrame->getWidth(), imgFrame->getHeight(), 30);
#if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            auto frame = imgFrame->getCvFrame();
            span data(frame.data, frame.total() * frame.elemSize());
            videoRecorder->write(imgFrame->getCvFrame().data);
#else
            auto data = imgFrame->getRecordData();
            videoRecorder->write(data);
#endif
        } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
            throw std::runtime_error("TODO: EncodedFrame recording is not supported yet");
        } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
            throw std::runtime_error("TODO: IMUData recording is not supported yet");
        } else {
            throw std::runtime_error("TODO: General Buffer recording is not supported yet");
        }
    }
}
void Record::stop() {
    videoRecorder->close();
}

Record& Record::setRecordFile(const std::string& recordFile) {
    this->recordFile = recordFile;
    return *this;
}

}  // namespace node
}  // namespace dai
