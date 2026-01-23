#include <stdexcept>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/RecordReplayImpl.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <google/protobuf/message.h>

    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "depthai/schemas/IMUData.pb.h"
    #include "depthai/schemas/ImgFrame.pb.h"
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {
namespace node {

#ifdef DEPTHAI_ENABLE_PROTOBUF
// Video Message
inline std::shared_ptr<Buffer> getVideoMessage(const proto::img_frame::ImgFrame& metadata, ImgFrame::Type outFrameType, std::vector<uint8_t>& frame) {
    auto imgFrame = std::make_shared<ImgFrame>();
    utility::setProtoMessage(*imgFrame, &metadata, true);

    assert(frame.size() == imgFrame->getWidth() * imgFrame->getHeight() * 3);
    cv::Mat img(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, frame.data());
    imgFrame->setCvFrame(img, outFrameType);
    return std::dynamic_pointer_cast<Buffer>(imgFrame);
}

inline std::shared_ptr<Buffer> getMessage(const std::shared_ptr<google::protobuf::Message> metadata, DatatypeEnum datatype) {
    switch(datatype) {
        case DatatypeEnum::ImgFrame: {
            auto imgFrame = std::make_shared<ImgFrame>();
            utility::setProtoMessage(*imgFrame, metadata.get(), false);
            return imgFrame;
        }
        case DatatypeEnum::EncodedFrame: {
            auto encFrame = std::make_shared<EncodedFrame>();
            utility::setProtoMessage(*encFrame, metadata.get(), false);
            return encFrame;
        }
        case DatatypeEnum::IMUData: {
            auto imuData = std::make_shared<IMUData>();
            utility::setProtoMessage(*imuData, metadata.get(), false);
            return imuData;
        }
        case DatatypeEnum::PointCloudData: {
            auto pclData = std::make_shared<PointCloudData>();
            utility::setProtoMessage(*pclData, metadata.get(), false);
            return pclData;
        }
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::NNData:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::ImgDetections:
        case DatatypeEnum::SpatialImgDetections:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationRVC4:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::ImgAnnotations:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::RGBDData:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CoverageData:
        case DatatypeEnum::PipelineEvent:
        case DatatypeEnum::PipelineState:
        case DatatypeEnum::PipelineEventAggregationConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::VppConfig:
            break;
    }
    throw std::runtime_error("Cannot replay message type: " + std::to_string((int)datatype));
}

inline std::shared_ptr<google::protobuf::Message> getProtoMessage(utility::BytePlayer& bytePlayer, DatatypeEnum datatype) {
    switch(datatype) {
        case DatatypeEnum::ImgFrame: {
            auto msg = bytePlayer.next<proto::img_frame::ImgFrame>();
            if(msg.has_value()) {
                return std::make_shared<proto::img_frame::ImgFrame>(msg.value());
            }
            break;
        }
        case DatatypeEnum::IMUData: {
            auto msg = bytePlayer.next<proto::imu_data::IMUData>();
            if(msg.has_value()) {
                return std::make_shared<proto::imu_data::IMUData>(msg.value());
            }
            break;
        }
        case DatatypeEnum::EncodedFrame: {
            auto msg = bytePlayer.next<proto::encoded_frame::EncodedFrame>();
            if(msg.has_value()) {
                return std::make_shared<proto::encoded_frame::EncodedFrame>(msg.value());
            }
            break;
        }
        case DatatypeEnum::PointCloudData: {
            auto msg = bytePlayer.next<proto::point_cloud_data::PointCloudData>();
            if(msg.has_value()) {
                return std::make_shared<proto::point_cloud_data::PointCloudData>(msg.value());
            }
            break;
        }
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::NNData:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::ImgDetections:
        case DatatypeEnum::SpatialImgDetections:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationRVC4:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::ImgAnnotations:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::RGBDData:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CoverageData:
        case DatatypeEnum::PipelineEvent:
        case DatatypeEnum::PipelineState:
        case DatatypeEnum::PipelineEventAggregationConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::VppConfig:
            throw std::runtime_error("Cannot replay message type: " + std::to_string((int)datatype));
    }
    return {};
}
#endif

void ReplayVideo::run() {
#ifdef DEPTHAI_ENABLE_PROTOBUF
    auto& logger = pimpl->logger;
    if(replayVideo.empty() && replayFile.empty()) {
        throw std::runtime_error("ReplayVideo node requires replayVideo or replayFile to be set");
    }
    utility::VideoPlayer videoPlayer;
    utility::BytePlayer bytePlayer;
    DatatypeEnum datatype = DatatypeEnum::ImgFrame;
    bool hasVideo = !replayVideo.empty();
    bool hasMetadata = !replayFile.empty();
    if(!replayVideo.empty()) try {
            videoPlayer.init(replayVideo.string());
            if(size.has_value()) {
                const auto& [width, height] = size.value();
                videoPlayer.setSize(width, height);
            }
        } catch(const std::exception& e) {
            hasVideo = false;
            if(logger) logger->warn("Video not replaying: {}", e.what());
        }
    if(!replayFile.empty()) try {
            auto schemaName = bytePlayer.init(replayFile.string());
            datatype = utility::schemaNameToDatatype(schemaName);
        } catch(const std::exception& e) {
            hasMetadata = false;
            if(logger) logger->warn("Metadata not replaying: {}", e.what());
        }
    if(!hasVideo) {
        throw std::runtime_error("Video file not found or could not be opened");
    }
    bool first = true;
    auto start = std::chrono::steady_clock::now();
    uint64_t index = 0;
    auto loopStart = std::chrono::steady_clock::now();
    auto prevMsgTs = loopStart;
    while(mainLoop()) {
        std::shared_ptr<proto::img_frame::ImgFrame> metadata;
        std::vector<uint8_t> frame;
        if(hasMetadata) {
            if(datatype != DatatypeEnum::ImgFrame) {
                throw std::runtime_error("Invalid message type, expected ImgFrame");
            }
            auto msg = getProtoMessage(bytePlayer, datatype);
            if(msg != nullptr) {
                metadata = std::dynamic_pointer_cast<proto::img_frame::ImgFrame>(msg);
            } else if(!first) {
                // End of file
                if(loop) {
                    bytePlayer.restart();
                    if(hasVideo) {
                        videoPlayer.restart();
                    }
                    continue;
                }
                break;
            } else {
                hasMetadata = false;
            }
        }
        if(hasVideo) {
            auto msg = videoPlayer.next();
            if(msg.has_value()) {
                frame = msg.value();
            } else if(!first) {
                // End of file
                if(loop) {
                    if(hasMetadata) {
                        bytePlayer.restart();
                    }
                    videoPlayer.restart();
                    continue;
                }
                break;
            } else {
                hasVideo = false;
            }
        }

        if(!hasMetadata && !hasVideo) {
            break;
        }

        if(!hasVideo) {
            throw std::runtime_error("Video file not found");
        }

        if(!hasMetadata) {
            ImgFrame frame;
            auto time = std::chrono::steady_clock::now() - start;
            const auto& [width, height] = videoPlayer.size();
            frame.setWidth(width);
            frame.setHeight(height);
            frame.setType(outFrameType);
            frame.setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock>(time));
            frame.setSequenceNum(index++);
            frame.sourceFb = frame.fb;
            frame.transformation = ImgTransformation(width, height);
            auto protoMsg = utility::getProtoMessage(&frame, true);
            std::shared_ptr<google::protobuf::Message> sharedProtoMsg = std::move(protoMsg);
            metadata = std::dynamic_pointer_cast<proto::img_frame::ImgFrame>(sharedProtoMsg);
        } else if(size.has_value()) {
            metadata->mutable_fb()->set_width(std::get<0>(size.value()));
            metadata->mutable_fb()->set_height(std::get<1>(size.value()));
        }

        auto buffer = getVideoMessage(*metadata, outFrameType, frame);

        if(first) prevMsgTs = buffer->getTimestampDevice();

        if(hasMetadata && !(fps.has_value() && fps.value() > 0.1f)) {
            std::this_thread::sleep_until(loopStart + (buffer->getTimestampDevice() - prevMsgTs));
        }

        {
            auto blockEvent = this->outputBlockEvent();
            if(buffer) out.send(buffer);
        }

        if(fps.has_value() && fps.value() > 0.1f) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds((uint32_t)roundf(1000.f / fps.value())));
        } else if(!hasMetadata) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds(1000 / 30));
        }

        loopStart = std::chrono::steady_clock::now();
        prevMsgTs = buffer->getTimestampDevice();

        first = false;
    }
    logger->info("Replay finished - stopping the pipeline!");
    try {
        stopPipeline();
    } catch(const std::exception& e) {
        // FIXME: This is a workaround for a bug in the pipeline
        if(e.what() != std::string("Pipeline is null")) {
            throw;
        }
    }
#else
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#endif
}

void ReplayMetadataOnly::run() {
#ifdef DEPTHAI_ENABLE_PROTOBUF
    auto& logger = pimpl->logger;
    if(replayFile.empty()) {
        throw std::runtime_error("ReplayMetadataOnly node requires replayFile to be set");
    }
    utility::BytePlayer bytePlayer;
    DatatypeEnum datatype = DatatypeEnum::Buffer;
    bool hasMetadata = !replayFile.empty();
    if(!replayFile.empty()) try {
            auto schemaName = bytePlayer.init(replayFile.string());
            datatype = utility::schemaNameToDatatype(schemaName);
        } catch(const std::exception& e) {
            hasMetadata = false;
            logger->warn("Metadata not replaying: {}", e.what());
        }
    if(!hasMetadata) {
        throw std::runtime_error("Metadata file not found");
    }
    bool first = true;
    auto loopStart = std::chrono::steady_clock::now();
    auto prevMsgTs = loopStart;
    while(mainLoop()) {
        std::shared_ptr<google::protobuf::Message> metadata;
        std::vector<uint8_t> frame;
        if(!utility::deserializationSupported(datatype)) {
            throw std::runtime_error("Invalid message type. Cannot replay");
        }
        auto msg = getProtoMessage(bytePlayer, datatype);
        if(msg != nullptr) {
            metadata = msg;
        } else if(!first) {
            // End of file
            if(loop) {
                bytePlayer.restart();
                continue;
            }
            break;
        } else {
            throw std::runtime_error("Metadata file contains no messages");
        }
        auto buffer = getMessage(metadata, datatype);

        if(first) prevMsgTs = buffer->getTimestampDevice();

        if(!(fps.has_value() && fps.value() > 0.1f)) {
            std::this_thread::sleep_until(loopStart + (buffer->getTimestampDevice() - prevMsgTs));
        }

        {
            auto blockEvent = this->outputBlockEvent();
            if(buffer) out.send(buffer);
        }

        if(fps.has_value() && fps.value() > 0.1f) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds((uint32_t)roundf(1000.f / fps.value())));
        }

        loopStart = std::chrono::steady_clock::now();
        prevMsgTs = buffer->getTimestampDevice();

        first = false;
    }
    try {
        stopPipeline();
    } catch(const std::exception& e) {
        // FIXME: This is a workaround for a bug in the pipeline
        if(e.what() != std::string("Pipeline is null")) {
            throw;
        }
    }
#else
    throw std::runtime_error("ReplayMetadataOnly node requires protobuf support");
#endif
}

std::filesystem::path ReplayVideo::getReplayMetadataFile() const {
    return replayFile;
}

std::filesystem::path ReplayVideo::getReplayVideoFile() const {
    return replayVideo;
}

ImgFrame::Type ReplayVideo::getOutFrameType() const {
    return outFrameType;
}

std::tuple<int, int> ReplayVideo::getSize() const {
    return size.value_or(std::make_tuple(0, 0));
}

float ReplayVideo::getFps() const {
    return fps.value_or(0.0f);
}

bool ReplayVideo::getLoop() const {
    return loop;
}

ReplayVideo& ReplayVideo::setReplayMetadataFile(const std::filesystem::path& replayFile) {
    this->replayFile = replayFile;
    return *this;
}

ReplayVideo& ReplayVideo::setReplayVideoFile(const std::filesystem::path& replayVideo) {
    this->replayVideo = replayVideo;
    return *this;
}

ReplayVideo& ReplayVideo::setOutFrameType(ImgFrame::Type outFrameType) {
    this->outFrameType = outFrameType;
    return *this;
}

ReplayVideo& ReplayVideo::setSize(std::tuple<int, int> size) {
    this->size = size;
    return *this;
}
ReplayVideo& ReplayVideo::setSize(int width, int height) {
    return setSize(std::make_tuple(width, height));
}
ReplayVideo& ReplayVideo::setFps(float fps) {
    this->fps = fps;
    return *this;
}
ReplayVideo& ReplayVideo::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

std::filesystem::path ReplayMetadataOnly::getReplayFile() const {
    return replayFile;
}
float ReplayMetadataOnly::getFps() const {
    return fps.value_or(0.0f);
}
bool ReplayMetadataOnly::getLoop() const {
    return loop;
}

ReplayMetadataOnly& ReplayMetadataOnly::setReplayFile(const std::filesystem::path& replayFile) {
    this->replayFile = replayFile;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setFps(float fps) {
    this->fps = fps;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

}  // namespace node
}  // namespace dai
