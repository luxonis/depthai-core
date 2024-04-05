#include "depthai/pipeline/node/Record.hpp"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <memory>

#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/span.hpp"

namespace dai {
namespace node {

enum class StreamType { EncodedVideo, RawVideo, Byte, Unknown };

using VideoCodec = dai::utility::VideoRecorder::VideoCodec;

void Record::build() {
    hostNode = true;
}

void Record::run() {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoRecorder = std::make_shared<dai::utility::VideoRecorder>();
#else
    throw std::runtime_error("Record node requires OpenCV support");
#endif

    if(recordFile.empty()) {
        throw std::runtime_error("Record recordFile must be set");
    }

    StreamType streamType = StreamType::Unknown;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int fps = 0;
    unsigned int i = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    // TODO(asahtik): Byte writer (Buffer, IMUData), metadata
    while(isRunning()) {
        auto msg = in.queue.get<dai::Buffer>();
        if(msg == nullptr) continue;
        if(streamType == StreamType::Unknown) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                if(imgFrame->getType() == dai::ImgFrame::Type::BITSTREAM)
                    throw std::runtime_error(
                        "Record node does not support encoded ImgFrame messages. Use the `out` output of VideoEncoder to record encoded frames.");
                streamType = StreamType::RawVideo;
                width = imgFrame->getWidth();
                height = imgFrame->getHeight();
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                if(encFrame->getProfile() == EncodedFrame::Profile::HEVC) {
                    throw std::runtime_error("Record node does not support H265 encoding");
                }
                streamType = StreamType::EncodedVideo;
                width = encFrame->getWidth();
                height = encFrame->getHeight();
                spdlog::trace("Record node detected {}x{} resolution", width, height);
            } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
                streamType = StreamType::Byte;
            } else {
                throw std::runtime_error("Record node does not support this type of message");
            }
            spdlog::trace("Record node detected stream type {}",
                          streamType == StreamType::RawVideo       ? "RawVideo"
                          : streamType == StreamType::EncodedVideo ? "EncodedVideo"
                                                                   : "Byte");
        }
        if(streamType == StreamType::RawVideo || streamType == StreamType::EncodedVideo) {
            if(i == 0)
                start = msg->getTimestampDevice();
            else if(i == fpsInitLength - 1) {
                end = msg->getTimestampDevice();
                fps = roundf((fpsInitLength * 1e6f) / (float)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
                spdlog::trace("Record node detected {} fps", fps);
                if(streamType == StreamType::EncodedVideo) {
                    auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                    videoRecorder->init(
                        recordFile, width, height, fps, encFrame->getProfile() == EncodedFrame::Profile::JPEG ? VideoCodec::MJPEG : VideoCodec::H264);
                } else {
                    videoRecorder->init(recordFile, width, height, fps, VideoCodec::RAW);
                }
            }
            if(i >= fpsInitLength - 1) {
                auto data = msg->getData();
                if(streamType == StreamType::RawVideo) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
                    auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                    auto frame = imgFrame->getCvFrame();
                    bool isGrayscale = imgFrame->getType() == ImgFrame::Type::GRAY8 || imgFrame->getType() == ImgFrame::Type::GRAYF16
                                       || (ImgFrame::Type::RAW16 <= imgFrame->getType() && imgFrame->getType() <= ImgFrame::Type::RAW8);
                    if(isGrayscale) {
                        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                    }
                    span cvData(frame.data, frame.total() * frame.elemSize());
                    videoRecorder->write(cvData);
#else
                    throw std::runtime_error("Record node requires OpenCV support");
#endif
                } else videoRecorder->write(data);
            }
            if(i < fpsInitLength) ++i;
        } else {
            throw std::runtime_error("TODO: Implement byte writer");
        }
    }

    videoRecorder->close();
}

Record& Record::setRecordFile(const std::string& recordFile) {
    this->recordFile = recordFile;
    return *this;
}

}  // namespace node
}  // namespace dai
