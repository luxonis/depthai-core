#include "depthai/pipeline/node/Record.hpp"

#include <cstdint>
#include <memory>

#include "depthai/utility/RecordReplay.hpp"
#include "pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/datatype/IMUData.hpp"
#include "utility/span.hpp"

namespace dai {
namespace node {

void Record::build() {
    hostNode = true;
}

void Record::start() {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoRecorder = std::make_shared<VideoRecorder>();
#else
    throw std::runtime_error("Record node requires OpenCV support");
#endif
}
void Record::run() {
    if(recordFile.empty()) {
        throw std::runtime_error("Record recordFile must be set");
    }
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int fps = 0;
    // TODO(asahtik): EncodedFrame and byte writer (Buffer, IMUData)
    while(isRunning()) {
        auto msg = in.queue.get<dai::Buffer>();
        span<uint8_t> data = msg->getData();
        if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
            auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
            videoRecorder->init(recordFile, width, height, fps, utility::VideoRecorder::VideoCodec::RAW);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            auto frame = imgFrame->getCvFrame();
            span data(frame.data, frame.total() * frame.elemSize());
            videoRecorder->write(imgFrame->getCvFrame().data);
#else
            throw std::runtime_error("Record node requires OpenCV support");
#endif
        } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
            auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
            videoRecorder->init(recordFile, width, height, fps, utility::VideoRecorder::VideoCodec::RAW);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            auto frame = imgFrame->getCvFrame();
            span data(frame.data, frame.total() * frame.elemSize());
            videoRecorder->write(imgFrame->getCvFrame().data);
#else
            throw std::runtime_error("Record node requires OpenCV support");
#endif
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
