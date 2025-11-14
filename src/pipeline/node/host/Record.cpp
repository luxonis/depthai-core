#include "depthai/pipeline/node/host/Record.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/span.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/RecordReplayImpl.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "depthai/schemas/IMUData.pb.h"
    #include "depthai/schemas/ImgFrame.pb.h"
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "utility/ProtoSerializable.hpp"
#endif

namespace dai {
namespace node {

using VideoCodec = dai::utility::VideoRecorder::VideoCodec;

void RecordVideo::run() {
#ifdef DEPTHAI_ENABLE_PROTOBUF
    auto& logger = pimpl->logger;
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

    DatatypeEnum streamType = DatatypeEnum::ADatatype;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int i = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    while(mainLoop()) {
        std::shared_ptr<dai::Buffer> msg = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            msg = input.get<dai::Buffer>();
        }
        if(msg == nullptr) continue;
        if(streamType == DatatypeEnum::ADatatype) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                if(imgFrame->getType() == dai::ImgFrame::Type::BITSTREAM)
                    throw std::runtime_error(
                        "RecordVideo node does not support encoded ImgFrame messages. Use the `out` output of VideoEncoder to record encoded frames.");
                streamType = DatatypeEnum::ImgFrame;
                width = imgFrame->getWidth();
                height = imgFrame->getHeight();
                if(recordMetadata) byteRecorder.init<dai::proto::img_frame::ImgFrame>(recordMetadataFile.string(), compressionLevel, "video");
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                if(encFrame->getProfile() == EncodedFrame::Profile::HEVC) {
                    throw std::runtime_error("RecordVideo node does not support H265 encoding");
                }
                streamType = DatatypeEnum::EncodedFrame;
                width = encFrame->getWidth();
                height = encFrame->getHeight();
                if(logger) logger->trace("RecordVideo node detected {}x{} resolution", width, height);
                if(recordMetadata) byteRecorder.init<dai::proto::img_frame::ImgFrame>(recordMetadataFile.string(), compressionLevel, "video");
            } else {
                throw std::runtime_error("RecordVideo can only record video streams.");
            }
            if(logger)
                logger->trace("RecordVideo node detected stream type {}",
                              streamType == DatatypeEnum::ImgFrame       ? "RawVideo"
                              : streamType == DatatypeEnum::EncodedFrame ? "EncodedVideo"
                                                                         : "Byte");
        }
        if(streamType == DatatypeEnum::ImgFrame || streamType == DatatypeEnum::EncodedFrame) {
            if(!videoRecorder->isInitialized()) {
                if(i == 0) {
                    start = msg->getTimestampDevice();
                }

                if(fps.has_value() || i == (fpsInitLength - 1)) {
                    end = msg->getTimestampDevice();
                    unsigned int calculatedFps =
                        roundf((fpsInitLength * 1e6f) / (float)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
                    if(logger) logger->trace("RecordVideo node detected {} fps", fps.value_or(calculatedFps));
                    if(streamType == DatatypeEnum::EncodedFrame) {
                        auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                        videoRecorder->init(recordVideoFile.string(),
                                            width,
                                            height,
                                            fps.value_or(calculatedFps),
                                            encFrame->getProfile() == EncodedFrame::Profile::JPEG ? VideoCodec::MJPEG : VideoCodec::H264);
                    } else {
                        videoRecorder->init(recordVideoFile.string(), width, height, fps.value_or(calculatedFps), VideoCodec::RAW);
                    }
                } else if(i < fpsInitLength) {
                    i++;
                    continue;
                }
            }

            // Record the data
            auto data = msg->getData();
            if(streamType == DatatypeEnum::ImgFrame) {
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
                    byteRecorder.write(imgFrame->serializeProto(true));
                }
    #else
                throw std::runtime_error("RecordVideo node requires OpenCV support");
    #endif
            } else {
                videoRecorder->write(data);
                if(recordMetadata) {
                    auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                    auto imgFrame = encFrame->getImgFrameMeta();
                    byteRecorder.write(imgFrame.serializeProto(true));
                }
            }
        } else {
            throw std::runtime_error("RecordVideo can only record video streams.");
        }
    }

    videoRecorder->close();
#else
    throw std::runtime_error("RecordVideo node requires protobuf support");
#endif
}

void RecordMetadataOnly::run() {
#ifdef DEPTHAI_ENABLE_PROTOBUF
    auto& logger = pimpl->logger;
    utility::ByteRecorder byteRecorder;

    DatatypeEnum streamType = DatatypeEnum::ADatatype;
    while(mainLoop()) {
        std::shared_ptr<dai::Buffer> msg = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            msg = input.get<dai::Buffer>();
        }
        if(msg == nullptr) continue;
        if(streamType == DatatypeEnum::ADatatype) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                streamType = DatatypeEnum::ImgFrame;
                byteRecorder.init<dai::proto::img_frame::ImgFrame>(recordFile.string(), compressionLevel, "img_frame");
            } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
                streamType = DatatypeEnum::IMUData;
                byteRecorder.init<dai::proto::imu_data::IMUData>(recordFile.string(), compressionLevel, "imu");
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                streamType = DatatypeEnum::EncodedFrame;
                byteRecorder.init<dai::proto::encoded_frame::EncodedFrame>(recordFile.string(), compressionLevel, "encoded_frame");
            } else if(std::dynamic_pointer_cast<PointCloudData>(msg) != nullptr) {
                streamType = DatatypeEnum::PointCloudData;
                byteRecorder.init<dai::proto::point_cloud_data::PointCloudData>(recordFile.string(), compressionLevel, "point_cloud");
            } else {
                throw std::runtime_error("RecordMetadataOnly node does not support this type of message");
            }
            if(logger) logger->trace("RecordMetadataOnly node detected stream type {}", (int)streamType);
        }
        auto serializable = std::dynamic_pointer_cast<ProtoSerializable>(msg);
        if(serializable == nullptr) {
            throw std::runtime_error("RecordMetadataOnly unsupported message type");
        }
        byteRecorder.write(serializable->serializeProto());
    }
#else
    throw std::runtime_error("RecordMetadataOnly node requires protobuf support");
#endif
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
RecordVideo& RecordVideo::setFps(unsigned int fps) {
    this->fps = fps;
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
