#include "depthai/pipeline/node/host/Record.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/span.hpp"
#include "utility/RecordReplayImpl.hpp"
#include "depthai/schemas/ADatatype.pb.h"

namespace dai {
namespace node {

enum class StreamType { EncodedVideo, RawVideo, Imu, Byte, Unknown };

using VideoCodec = dai::utility::VideoRecorder::VideoCodec;

void RecordVideo::run() {
    std::unique_ptr<utility::VideoRecorder> videoRecorder;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoRecorder = std::make_unique<dai::utility::VideoRecorder>();
#else
    throw std::runtime_error("RecordVideo node requires OpenCV support");
#endif

    utility::ByteRecorder byteRecorder;

    if(recordVideoFile.empty()) {
        throw std::runtime_error("RecordVideo recordVideoFile must be set");
    }
    bool recordMetadata = !recordMetadataFile.empty();

    StreamType streamType = StreamType::Unknown;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int fps = 0;
    unsigned int i = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    while(isRunning()) {
        auto msg = input.get<dai::Buffer>();
        if(msg == nullptr) continue;
        if(streamType == StreamType::Unknown) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                if(imgFrame->getType() == dai::ImgFrame::Type::BITSTREAM)
                    throw std::runtime_error(
                        "RecordVideo node does not support encoded ImgFrame messages. Use the `out` output of VideoEncoder to record encoded frames.");
                streamType = StreamType::RawVideo;
                width = imgFrame->getWidth();
                height = imgFrame->getHeight();
                if(recordMetadata) byteRecorder.init(recordMetadataFile.string(), compressionLevel, "video");
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                if(encFrame->getProfile() == EncodedFrame::Profile::HEVC) {
                    throw std::runtime_error("RecordVideo node does not support H265 encoding");
                }
                streamType = StreamType::EncodedVideo;
                width = encFrame->getWidth();
                height = encFrame->getHeight();
                if(logger) logger->trace("RecordVideo node detected {}x{} resolution", width, height);
                if(recordMetadata) byteRecorder.init(recordMetadataFile.string(), compressionLevel, "video");
            } else {
                throw std::runtime_error("RecordVideo can only record video streams.");
            }
            if(logger)
                logger->trace("RecordVideo node detected stream type {}",
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
                if(logger) logger->trace("RecordVideo node detected {} fps", fps);
                if(streamType == StreamType::EncodedVideo) {
                    auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                    videoRecorder->init(recordVideoFile.string(),
                                        width,
                                        height,
                                        fps,
                                        encFrame->getProfile() == EncodedFrame::Profile::JPEG ? VideoCodec::MJPEG : VideoCodec::H264);
                } else {
                    videoRecorder->init(recordVideoFile.string(), width, height, fps, VideoCodec::RAW);
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
                    assert(frame.isContinuous());
                    span cvData(frame.data, frame.total() * frame.elemSize());
                    videoRecorder->write(cvData, frame.step);
                    if(recordMetadata) {
                        dai::proto::adatatype::ADatatype adatatype;
                        adatatype.mutable_imgframe()->CopyFrom((*imgFrame->getProtoMessage(true)));
                        byteRecorder.write(adatatype);
                    }
#else
                    throw std::runtime_error("RecordVideo node requires OpenCV support");
#endif
                } else {
                    videoRecorder->write(data);
                    if(recordMetadata) {
                        auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                        auto imgFrame = encFrame->getImgFrameMeta();
                        dai::proto::adatatype::ADatatype adatatype;
                        adatatype.mutable_imgframe()->CopyFrom((*imgFrame.getProtoMessage(true)));
                        byteRecorder.write(adatatype);
                    }
                }
            }
            if(i < fpsInitLength) ++i;
        } else {
            throw std::runtime_error("RecordVideo can only record video streams.");
        }
    }

    videoRecorder->close();
}

void RecordMetadataOnly::run() {
    utility::ByteRecorder byteRecorder;

    StreamType streamType = StreamType::Unknown;
    while(isRunning()) {
        auto msg = input.get<dai::Buffer>();
        if(msg == nullptr) continue;
        if(streamType == StreamType::Unknown) {
            if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
                streamType = StreamType::Imu;
                byteRecorder.init(recordFile.string(), compressionLevel, "imu");
            } else {
                throw std::runtime_error("RecordMetadataOnly node does not support this type of message");
            }
            if(logger)
                logger->trace("RecordMetadataOnly node detected stream type {}",
                              streamType == StreamType::RawVideo       ? "RawVideo"
                              : streamType == StreamType::EncodedVideo ? "EncodedVideo"
                                                                       : "Byte");
        }
        if(streamType == StreamType::Imu) {
            auto imuData = std::dynamic_pointer_cast<IMUData>(msg);
            dai::proto::adatatype::ADatatype adatatype;
            adatatype.mutable_imgframe()->CopyFrom((*imuData->getProtoMessage()));
            byteRecorder.write(adatatype);
        } else {
            throw std::runtime_error("RecordMetadataOnly unsupported message type");
        }
    }
}

std::filesystem::path RecordVideo::getRecordMetadataFile() const {
    return recordMetadataFile;
}
std::filesystem::path RecordVideo::getRecordVideoFile() const {
    return recordVideoFile;
}
RecordVideo::CompressionLevel RecordVideo::getCompressionLevel() const {
    return compressionLevel;
}

RecordVideo& RecordVideo::setRecordMetadataFile(const std::filesystem::path& recordFile) {
    this->recordMetadataFile = recordFile;
    return *this;
}
RecordVideo& RecordVideo::setRecordVideoFile(const std::filesystem::path& recordFile) {
    this->recordVideoFile = recordFile;
    return *this;
}
RecordVideo& RecordVideo::setCompressionLevel(CompressionLevel compressionLevel) {
    this->compressionLevel = compressionLevel;
    return *this;
}

std::filesystem::path RecordMetadataOnly::getRecordFile() const {
    return recordFile;
}
RecordMetadataOnly::CompressionLevel RecordMetadataOnly::getCompressionLevel() const {
    return compressionLevel;
}

RecordMetadataOnly& RecordMetadataOnly::setRecordFile(const std::filesystem::path& recordFile) {
    this->recordFile = recordFile;
    return *this;
}
RecordMetadataOnly& RecordMetadataOnly::setCompressionLevel(CompressionLevel compressionLevel) {
    this->compressionLevel = compressionLevel;
    return *this;
}

}  // namespace node
}  // namespace dai
