#include "depthai/pipeline/node/host/Replay.hpp"

#include <archive.h>
#include <archive_entry.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/RGBDData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/utility/ReplayPlayback.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "opencv/ReplayInternal.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/Environment.hpp"
#include "utility/Platform.hpp"
#include "utility/RecordReplayImpl.hpp"
#include "utility/ReplayPacing.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <google/protobuf/message.h>

    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "depthai/schemas/IMUData.pb.h"
    #include "depthai/schemas/ImageAnnotations.pb.h"
    #include "depthai/schemas/ImgDetections.pb.h"
    #include "depthai/schemas/ImgFrame.pb.h"
    #include "depthai/schemas/NNData.pb.h"
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "depthai/schemas/RGBDData.pb.h"
    #include "depthai/schemas/SegmentationMask.pb.h"
    #include "depthai/schemas/SpatialImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {
namespace node {

namespace {

using Steady = std::chrono::steady_clock;
using SteadyTime = Steady::time_point;

struct TokenBook {
    std::mutex mutex;
    std::uint64_t next = 1;
    const void* key = nullptr;
    std::uint64_t token = 0;
    int holders = 0;
};

TokenBook gTokens;

std::uint64_t acquireToken(const void* key) {
    std::lock_guard<std::mutex> lock(gTokens.mutex);
    if(gTokens.key == key && gTokens.holders > 0) {
        gTokens.holders++;
        return gTokens.token;
    }
    gTokens.key = key;
    gTokens.token = gTokens.next++;
    gTokens.holders = 1;
    return gTokens.token;
}

void releaseToken(const void* key) {
    std::lock_guard<std::mutex> lock(gTokens.mutex);
    if(gTokens.key == key && gTokens.holders > 0) gTokens.holders--;
}

bool deviceIsRvc4(const DeviceNode& node) {
    const auto device = node.getDevice();
    if(!device) return false;
    try {
        return device->getPlatform() == Platform::RVC4;
    } catch(const std::exception&) {
        return false;
    }
}

std::uint64_t outputBytes(ImgFrame::Type type, int width, int height) {
    const auto pixels = static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    if(type == ImgFrame::Type::GRAY8) return pixels;
    if(type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p) return pixels * 3 / 2;
    if(type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i) return pixels * 3;
    return pixels;
}

bool fpsIsSentinel(bool fpsSet, float fps) {
    // 1000 paces at 1 ms, but pass-boundary timestamps keep the recorded mean. Only exactly 1000 is that sentinel.
    return !fpsSet || !(fps > 0.f) || std::abs(fps - 1000.f) < 0.1f;
}

std::uint64_t archiveContentBytes(const std::filesystem::path& archive) {
    struct archive* reader = archive_read_new();
    if(reader == nullptr) throw std::runtime_error("ReplayVideo archive cannot be opened: " + archive.string());
    archive_read_support_filter_all(reader);
    archive_read_support_format_all(reader);
    if(archive_read_open_filename(reader, archive.string().c_str(), 10240) != ARCHIVE_OK) {
        const std::string message = archive_error_string(reader) == nullptr ? archive.string() : archive_error_string(reader);
        archive_read_free(reader);
        throw std::runtime_error("ReplayVideo archive cannot be opened: " + message);
    }
    std::uint64_t sum = 0;
    struct archive_entry* entry = nullptr;
    while(archive_read_next_header(reader, &entry) == ARCHIVE_OK) {
        if(archive_entry_filetype(entry) == AE_IFREG && archive_entry_size(entry) > 0) {
            sum += static_cast<std::uint64_t>(archive_entry_size(entry));
        }
        archive_read_data_skip(reader);
    }
    archive_read_free(reader);
    return sum;
}

struct FileSnap {
    std::filesystem::path path;
    std::uintmax_t size = 0;
    std::filesystem::file_time_type mtime{};
};

FileSnap snapOf(const std::filesystem::path& path) {
    FileSnap snap;
    snap.path = path;
    snap.size = std::filesystem::file_size(path);
    snap.mtime = std::filesystem::last_write_time(path);
    return snap;
}

void checkSnaps(const std::vector<FileSnap>& snaps) {
    for(const auto& snap : snaps) {
        std::error_code error;
        const auto size = std::filesystem::file_size(snap.path, error);
        const auto mtime = std::filesystem::last_write_time(snap.path, error);
        if(error || size != snap.size || mtime != snap.mtime) {
            throw std::runtime_error("ReplayVideo frame changed during replay: " + snap.path.string());
        }
    }
}

struct RecordedTiming {
    bool present = false;
    std::int64_t sequence = 0;
    SteadyTime timestamp{};
    SteadyTime timestampDevice{};
    std::optional<std::chrono::system_clock::time_point> timestampSystem;
};

void logLines(const std::shared_ptr<spdlog::async_logger>& logger, const std::vector<std::string>& lines) {
    if(!logger) return;
    for(const auto& line : lines) logger->info("{}", line);
}

void logProgress(
    const std::shared_ptr<spdlog::async_logger>& logger, const std::string& kind, const std::string& hash, std::uint64_t done, std::uint64_t total) {
    if(!logger || total == 0) return;
    logger->info("Replay {} {} {}/{} {}", kind, hash, done, total, kind == "upload" ? "bytes" : "frames");
}

}  // namespace

struct ReplayRuntime {
    std::uint64_t nodeId = 0;
    std::uint64_t token = 0;
    const void* pipelineKey = nullptr;
    bool persistent = false;
    bool onHost = true;
    bool loop = true;
    bool stopAtEnd = true;
    bool fpsSet = false;
    float fps = 0.f;
    bool released = false;
    std::vector<std::string> cacheKeys;
    std::shared_ptr<Device> device;

    bool video = false;
    replay::ListedSource listed;
    replay::FrameReader reader;
    bool usePredecoded = false;
    std::filesystem::path predecodedDir;
    ImgFrame::Type outType = ImgFrame::Type::YUV420p;
    int outW = 0;
    int outH = 0;
    bool passthrough = false;
    bool recorded = false;
    int count = 0;
    std::int64_t seqFirst = 0;
    std::int64_t seqLast = 0;
    SteadyTime deviceFirst{};
    SteadyTime deviceLast{};
    Steady::duration nominal{};
    std::vector<std::shared_ptr<ImgFrame>> metadata;
    std::string metadataWarning;
    std::vector<FileSnap> snaps;
    std::filesystem::path mcapPath;
};

namespace {

#ifdef DEPTHAI_ENABLE_PROTOBUF
std::shared_ptr<Buffer> getMessage(const std::shared_ptr<google::protobuf::Message> metadata, DatatypeEnum datatype) {
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
        case DatatypeEnum::NNData: {
            auto nnData = std::make_shared<NNData>();
            utility::setProtoMessage(*nnData, metadata.get(), false);
            return nnData;
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
        case DatatypeEnum::ImgAnnotations: {
            auto annotations = std::make_shared<ImgAnnotations>();
            utility::setProtoMessage(*annotations, metadata.get(), false);
            return annotations;
        }
        case DatatypeEnum::ImgDetections: {
            auto detections = std::make_shared<ImgDetections>();
            utility::setProtoMessage(*detections, metadata.get(), false);
            return detections;
        }
        case DatatypeEnum::SpatialImgDetections: {
            auto detections = std::make_shared<SpatialImgDetections>();
            utility::setProtoMessage(*detections, metadata.get(), false);
            return detections;
        }
        case DatatypeEnum::SegmentationMask: {
            auto mask = std::make_shared<SegmentationMask>();
            utility::setProtoMessage(*mask, metadata.get(), false);
            return mask;
        }
        case DatatypeEnum::RGBDData: {
            auto rgbdData = std::make_shared<RGBDData>();
            utility::setProtoMessage(*rgbdData, metadata.get(), false);
            return rgbdData;
        }
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::Transformable:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationRVC4:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::DetectionParserConfig:
        case DatatypeEnum::SegmentationParserConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::GPUStereoConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::GateControl:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::MapData:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::Odometry:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::AlignConfig:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::MultiDeviceCalibrationControl:
        case DatatypeEnum::MultiDeviceCalibrationResult:
        case DatatypeEnum::AutoCalibrationConfig:
        case DatatypeEnum::AutoCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CalibrationMetrics:
        case DatatypeEnum::CoverageData:
        case DatatypeEnum::PipelineEvent:
        case DatatypeEnum::PipelineState:
        case DatatypeEnum::PipelineEventAggregationConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::VppConfig:
        case DatatypeEnum::PacketizedData:
        case DatatypeEnum::ImgDetectionsFilterConfig:
        case DatatypeEnum::Classifications:
        case DatatypeEnum::Keypoints:
        case DatatypeEnum::Clusters:
        case DatatypeEnum::Map2D:
        case DatatypeEnum::Lines:
        case DatatypeEnum::Predictions:
        case DatatypeEnum::FastSAMParserConfig:
        case DatatypeEnum::HRNetParserConfig:
        case DatatypeEnum::MLSDParserConfig:
        case DatatypeEnum::MPPalmDetectionParserConfig:
        case DatatypeEnum::PPTextDetectionParserConfig:
        case DatatypeEnum::RFDETRParserConfig:
        case DatatypeEnum::SCRFDParserConfig:
        case DatatypeEnum::SuperAnimalParserConfig:
        case DatatypeEnum::YuNetParserConfig:
        case DatatypeEnum::ClassificationSequenceParserConfig:
        case DatatypeEnum::MapOutputParserConfig:
        case DatatypeEnum::XFeatMonoParserConfig:
        case DatatypeEnum::XFeatStereoParserConfig:
        case DatatypeEnum::RecordControl:
        case DatatypeEnum::RecordStatus:
        case DatatypeEnum::COUNT:
            break;
    }
    throw std::runtime_error("Cannot replay message type: " + std::to_string(static_cast<int>(datatype)));
}

std::shared_ptr<google::protobuf::Message> getProtoMessage(utility::BytePlayer& bytePlayer, DatatypeEnum datatype) {
    switch(datatype) {
        case DatatypeEnum::ImgFrame: {
            auto msg = bytePlayer.next<proto::img_frame::ImgFrame>();
            if(msg.has_value()) return std::make_shared<proto::img_frame::ImgFrame>(msg.value());
            break;
        }
        case DatatypeEnum::IMUData: {
            auto msg = bytePlayer.next<proto::imu_data::IMUData>();
            if(msg.has_value()) return std::make_shared<proto::imu_data::IMUData>(msg.value());
            break;
        }
        case DatatypeEnum::EncodedFrame: {
            auto msg = bytePlayer.next<proto::encoded_frame::EncodedFrame>();
            if(msg.has_value()) return std::make_shared<proto::encoded_frame::EncodedFrame>(msg.value());
            break;
        }
        case DatatypeEnum::NNData: {
            auto msg = bytePlayer.next<proto::nn_data::NNData>();
            if(msg.has_value()) return std::make_shared<proto::nn_data::NNData>(msg.value());
            break;
        }
        case DatatypeEnum::PointCloudData: {
            auto msg = bytePlayer.next<proto::point_cloud_data::PointCloudData>();
            if(msg.has_value()) return std::make_shared<proto::point_cloud_data::PointCloudData>(msg.value());
            break;
        }
        case DatatypeEnum::ImgAnnotations: {
            auto msg = bytePlayer.next<proto::image_annotations::ImageAnnotations>();
            if(msg.has_value()) return std::make_shared<proto::image_annotations::ImageAnnotations>(msg.value());
            break;
        }
        case DatatypeEnum::ImgDetections: {
            auto msg = bytePlayer.next<proto::img_detections::ImgDetections>();
            if(msg.has_value()) return std::make_shared<proto::img_detections::ImgDetections>(msg.value());
            break;
        }
        case DatatypeEnum::SpatialImgDetections: {
            auto msg = bytePlayer.next<proto::spatial_img_detections::SpatialImgDetections>();
            if(msg.has_value()) return std::make_shared<proto::spatial_img_detections::SpatialImgDetections>(msg.value());
            break;
        }
        case DatatypeEnum::SegmentationMask: {
            auto msg = bytePlayer.next<proto::segmentation_mask::SegmentationMask>();
            if(msg.has_value()) return std::make_shared<proto::segmentation_mask::SegmentationMask>(msg.value());
            break;
        }
        case DatatypeEnum::RGBDData: {
            auto msg = bytePlayer.next<proto::rgbd_data::RGBDData>();
            if(msg.has_value()) return std::make_shared<proto::rgbd_data::RGBDData>(msg.value());
            break;
        }
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::Transformable:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationRVC4:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::DetectionParserConfig:
        case DatatypeEnum::SegmentationParserConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::GateControl:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::GPUStereoConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::MapData:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::Odometry:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::AlignConfig:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::MultiDeviceCalibrationControl:
        case DatatypeEnum::MultiDeviceCalibrationResult:
        case DatatypeEnum::AutoCalibrationConfig:
        case DatatypeEnum::AutoCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CalibrationMetrics:
        case DatatypeEnum::CoverageData:
        case DatatypeEnum::PipelineEvent:
        case DatatypeEnum::PipelineState:
        case DatatypeEnum::PipelineEventAggregationConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::VppConfig:
        case DatatypeEnum::PacketizedData:
        case DatatypeEnum::ImgDetectionsFilterConfig:
        case DatatypeEnum::Classifications:
        case DatatypeEnum::Keypoints:
        case DatatypeEnum::Clusters:
        case DatatypeEnum::Map2D:
        case DatatypeEnum::Lines:
        case DatatypeEnum::Predictions:
        case DatatypeEnum::FastSAMParserConfig:
        case DatatypeEnum::HRNetParserConfig:
        case DatatypeEnum::MLSDParserConfig:
        case DatatypeEnum::MPPalmDetectionParserConfig:
        case DatatypeEnum::PPTextDetectionParserConfig:
        case DatatypeEnum::RFDETRParserConfig:
        case DatatypeEnum::SCRFDParserConfig:
        case DatatypeEnum::SuperAnimalParserConfig:
        case DatatypeEnum::YuNetParserConfig:
        case DatatypeEnum::ClassificationSequenceParserConfig:
        case DatatypeEnum::MapOutputParserConfig:
        case DatatypeEnum::XFeatMonoParserConfig:
        case DatatypeEnum::XFeatStereoParserConfig:
        case DatatypeEnum::RecordControl:
        case DatatypeEnum::RecordStatus:
        case DatatypeEnum::COUNT:
            throw std::runtime_error("Cannot replay message type: " + std::to_string(static_cast<int>(datatype)));
    }
    return {};
}
#endif

void offsetRGBDChildFrameMetadata(const std::shared_ptr<Buffer>& buffer,
                                  std::int64_t sequenceNumOffset,
                                  Steady::duration timestampOffset,
                                  Steady::duration deviceTimestampOffset) {
    auto rgbdData = std::dynamic_pointer_cast<RGBDData>(buffer);
    if(rgbdData == nullptr) return;
    const auto offsetFrameMetadata = [&](const std::optional<RGBDData::FrameVariant>& frame) {
        if(!frame.has_value()) return;
        std::visit(
            [&](const auto& typedFrame) {
                if(typedFrame == nullptr) return;
                typedFrame->setSequenceNum(typedFrame->getSequenceNum() + sequenceNumOffset);
                typedFrame->setTimestamp(typedFrame->getTimestamp() + timestampOffset);
                typedFrame->setTimestampDevice(typedFrame->getTimestampDevice() + deviceTimestampOffset);
                if(typedFrame->getTimestampSystem().has_value()) {
                    const auto shifted =
                        *typedFrame->getTimestampSystem() + std::chrono::duration_cast<std::chrono::system_clock::duration>(deviceTimestampOffset);
                    typedFrame->setTimestampSystem(shifted);
                }
            },
            frame.value());
    };
    offsetFrameMetadata(rgbdData->getRGBFrame());
    offsetFrameMetadata(rgbdData->getDepthFrame());
}

std::string uploadFile(Device& device, const std::shared_ptr<spdlog::async_logger>& logger, const std::filesystem::path& file, const std::string& hash) {
    const auto found = device.replayCacheQuery(hash);
    if(std::get<0>(found)) {
        if(logger) logger->info("Replay cache hit {}", hash);
        return std::get<1>(found);
    }
    const auto total = static_cast<std::uint64_t>(std::filesystem::file_size(file));
    device.replayCacheBegin(hash, total);
    try {
        std::ifstream input(file, std::ios::binary);
        if(!input) throw std::runtime_error("ReplayVideo source cannot be read: " + file.string());
        std::uint64_t offset = 0;
        int lastBucket = 0;
        std::vector<std::uint8_t> chunk(static_cast<std::size_t>(replay::kUploadChunk));
        while(input) {
            input.read(reinterpret_cast<char*>(chunk.data()), static_cast<std::streamsize>(chunk.size()));
            const auto count = input.gcount();
            if(count <= 0) break;
            std::vector<std::uint8_t> bytes(chunk.begin(), chunk.begin() + count);
            device.replayCacheWrite(hash, offset, bytes);
            offset += static_cast<std::uint64_t>(count);
            if(total > 0) {
                const int bucket = static_cast<int>((offset * 100) / total) / 10 * 10;
                if(bucket >= lastBucket + 10) {
                    lastBucket = bucket;
                    logProgress(logger, "upload", hash, offset, total);
                }
            }
        }
        device.replayCacheCommit(hash);
        if(total > 0 && lastBucket < 100) logProgress(logger, "upload", hash, total, total);
    } catch(...) {
        try {
            device.replayCacheAbort(hash);
        } catch(const std::exception&) {
        }
        throw;
    }
    if(logger) logger->info("Replay cache upload {} {} bytes", hash, total);
    const auto committed = device.replayCacheQuery(hash);
    if(std::get<0>(committed) && !std::get<1>(committed).empty()) return std::get<1>(committed);
    return "/data/depthai/replay_cache/" + hash;
}

replay::CacheSlot acquireSlot(ReplayRuntime& runtime,
                              const std::shared_ptr<spdlog::async_logger>& logger,
                              const replay::CacheSlotRequest& request,
                              bool copyBytes) {
    auto slot = replay::beginCacheEntry(runtime.nodeId, runtime.token, runtime.persistent, request);
    if(slot.hit) {
        logLines(logger, slot.logs);
        runtime.cacheKeys.push_back(request.key);
        return slot;
    }
    try {
        replay::commitCacheEntry(runtime.nodeId, runtime.token, runtime.persistent, request);
    } catch(...) {
        replay::abortCacheEntry(request.key);
        throw;
    }
    runtime.cacheKeys.push_back(request.key);
    if(copyBytes && logger) logger->info("Replay cache upload {} {} bytes", request.sourceHash, request.sizeBytes);
    return slot;
}

void applyEnv(bool& persistent, bool persistentSet, std::int32_t& decodeMode, bool decodeModeSet, const char* nodeName) {
    if(!persistentSet) {
        const auto flag = utility::getEnvAs<std::string>("DEPTHAI_REPLAY_PERSISTENT_UPLOAD", "", false);
        if(flag == "1" || flag == "true" || flag == "TRUE") persistent = true;
    }
    if(!decodeModeSet) {
        const auto mode = utility::getEnvAs<std::string>("DEPTHAI_REPLAY_DECODE_MODE", "", false);
        if(mode.empty() || mode == "STREAM")
            decodeMode = 0;
        else if(mode == "PRE_DECODE")
            decodeMode = 1;
        else
            throw std::runtime_error(std::string(nodeName) + " decode mode is not supported: " + mode);
    }
}

void checkFps(float fps, bool fpsSet, const char* nodeName) {
    if(fpsSet && !std::isfinite(fps)) throw std::runtime_error(std::string(nodeName) + " fps is not a finite number");
    if(fpsSet && fps < 0.f) throw std::runtime_error(std::string(nodeName) + " fps must be 0 or more");
}

Steady::duration nominalOf(bool fpsSet, float fps, SteadyTime first, SteadyTime last, int count) {
    if(!fpsIsSentinel(fpsSet, fps)) return replay::playbackPeriod(fps);
    if(count >= 2) {
        const auto mean = (last - first) / (count - 1);
        if(mean.count() > 0) return mean;
    }
    return replay::playbackPeriod(30.0);
}

void applyRecorded(const std::shared_ptr<Buffer>& buffer, const ReplayRuntime& runtime, const RecordedTiming& recorded, int pass) {
    std::int64_t seqOffset = runtime.seqFirst;
    SteadyTime tsOffset = runtime.deviceFirst;
    for(int index = 0; index < pass; index++) {
        seqOffset += runtime.seqLast - runtime.seqFirst + 1;
        tsOffset += (runtime.deviceLast - runtime.deviceFirst) + runtime.nominal;
    }
    const auto sequence = recorded.sequence - runtime.seqFirst + seqOffset;
    const auto deviceOut = recorded.timestampDevice - runtime.deviceFirst + tsOffset;
    const auto amount = deviceOut - recorded.timestampDevice;
    const auto originalSequence = buffer->getSequenceNum();
    const auto originalTimestamp = buffer->getTimestamp();
    const auto originalDevice = buffer->getTimestampDevice();
    buffer->setSequenceNum(sequence);
    buffer->setTimestamp(recorded.timestamp + amount);
    buffer->setTimestampDevice(deviceOut);
    if(recorded.timestampSystem.has_value()) {
        buffer->setTimestampSystem(*recorded.timestampSystem + std::chrono::duration_cast<std::chrono::system_clock::duration>(amount));
    } else {
        buffer->setTimestampSystem(std::nullopt);
    }
    offsetRGBDChildFrameMetadata(
        buffer, buffer->getSequenceNum() - originalSequence, buffer->getTimestamp() - originalTimestamp, buffer->getTimestampDevice() - originalDevice);
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
struct McapScan {
    int count = 0;
    std::int64_t seqFirst = 0;
    std::int64_t seqLast = 0;
    SteadyTime deviceFirst{};
    SteadyTime deviceLast{};
    std::vector<std::shared_ptr<ImgFrame>> frames;
    DatatypeEnum datatype = DatatypeEnum::Buffer;
};

McapScan scanMcap(const std::filesystem::path& path, bool imgFramesOnly, const char* nodeName) {
    if(!std::filesystem::exists(path)) throw std::runtime_error(std::string(nodeName) + " metadata file is missing: " + path.string());
    utility::BytePlayer player;
    const auto schema = player.init(path.string());
    const auto datatype = utility::schemaNameToDatatype(schema);
    if(!utility::deserializationSupported(datatype)) throw std::runtime_error(std::string(nodeName) + " metadata type is not supported: " + schema);
    if(imgFramesOnly && datatype != DatatypeEnum::ImgFrame) {
        throw std::runtime_error(std::string(nodeName) + " metadata type is not ImgFrame: " + schema);
    }
    McapScan scan;
    scan.datatype = datatype;
    bool first = true;
    std::int64_t previousSeq = 0;
    SteadyTime previousDevice{};
    while(true) {
        auto proto = getProtoMessage(player, datatype);
        if(proto == nullptr) break;
        auto buffer = getMessage(proto, datatype);
        if(first) {
            scan.seqFirst = buffer->getSequenceNum();
            scan.deviceFirst = buffer->getTimestampDevice();
            first = false;
        } else {
            if(buffer->getSequenceNum() <= previousSeq) {
                throw std::runtime_error(std::string(nodeName) + " sequence numbers do not increase: " + std::to_string(previousSeq) + " then "
                                         + std::to_string(buffer->getSequenceNum()));
            }
            if(buffer->getTimestampDevice() < previousDevice) throw std::runtime_error(std::string(nodeName) + " timestamps decrease");
        }
        previousSeq = buffer->getSequenceNum();
        previousDevice = buffer->getTimestampDevice();
        scan.seqLast = previousSeq;
        scan.deviceLast = previousDevice;
        scan.count++;
        if(imgFramesOnly) scan.frames.push_back(std::dynamic_pointer_cast<ImgFrame>(buffer));
    }
    if(scan.count == 0) throw std::runtime_error(std::string(nodeName) + " metadata file is empty: " + path.string());
    return scan;
}
#endif

void noteDaiTiming(ReplayRuntime& runtime, const replay::DecodedPixels& pixels, int index) {
    if(!pixels.hasTiming) return;
    if(index == 0) {
        runtime.seqFirst = pixels.sequenceNum;
        runtime.deviceFirst = pixels.timestampDevice;
        runtime.recorded = true;
    } else {
        if(pixels.sequenceNum <= runtime.seqLast) throw std::runtime_error("ReplayVideo sequence numbers do not increase");
        if(pixels.timestampDevice < runtime.deviceLast) throw std::runtime_error("ReplayVideo timestamps decrease");
    }
    runtime.seqLast = pixels.sequenceNum;
    runtime.deviceLast = pixels.timestampDevice;
}

std::shared_ptr<ImgFrame> frameFromPixels(const replay::DecodedPixels& raw, const ReplayRuntime& runtime, const std::shared_ptr<ImgFrame>& metadata) {
    auto converted = raw.passthrough || runtime.passthrough ? raw : replay::convertPixels(raw, runtime.outType, runtime.outW, runtime.outH);
    if(!raw.passthrough && !runtime.passthrough) {
        converted.transformation = raw.transformation;
        converted.sourceFb = raw.sourceFb;
        converted.cam = raw.cam;
        converted.instanceNum = raw.instanceNum;
        converted.category = raw.category;
    }
    auto frame = replay::pixelsToFrame(converted);
    if(metadata != nullptr) {
        frame->sourceFb = metadata->sourceFb;
        frame->cam = metadata->cam;
        frame->instanceNum = metadata->instanceNum;
        frame->category = metadata->category;
        frame->transformation = metadata->transformation;
    } else if(raw.sourceFb.width != 0) {
        frame->sourceFb = raw.sourceFb;
        frame->cam = raw.cam;
        frame->instanceNum = raw.instanceNum;
        frame->category = raw.category;
        if(raw.transformation.getSize().first != 0) frame->transformation = raw.transformation;
    }
    if(!runtime.passthrough && (runtime.outW != runtime.listed.width || runtime.outH != runtime.listed.height) && runtime.listed.width > 0
       && runtime.listed.height > 0) {
        frame->transformation.addScale(static_cast<float>(runtime.outW) / static_cast<float>(runtime.listed.width),
                                       static_cast<float>(runtime.outH) / static_cast<float>(runtime.listed.height));
    }
    return frame;
}

RecordedTiming timingFrom(const replay::DecodedPixels& pixels, const std::shared_ptr<ImgFrame>& metadata) {
    RecordedTiming timing;
    if(metadata != nullptr) {
        timing.present = true;
        timing.sequence = metadata->getSequenceNum();
        timing.timestamp = metadata->getTimestamp();
        timing.timestampDevice = metadata->getTimestampDevice();
        timing.timestampSystem = metadata->getTimestampSystem();
        return timing;
    }
    if(pixels.hasTiming) {
        timing.present = true;
        timing.sequence = pixels.sequenceNum;
        timing.timestamp = pixels.timestamp;
        timing.timestampDevice = pixels.timestampDevice;
        timing.timestampSystem = pixels.timestampSystem;
    }
    return timing;
}

}  // namespace

#ifdef DEPTHAI_ENABLE_PROTOBUF
static std::int32_t bridgeDatatypeFromSchema(const std::string& schema) {
    const auto datatype = utility::schemaNameToDatatype(schema);
    if(!utility::deserializationSupported(datatype)) {
        throw std::runtime_error("Cannot replay message type: " + schema);
    }
    return static_cast<std::int32_t>(datatype);
}

static std::shared_ptr<Buffer> bridgeNextRecordedBuffer(utility::BytePlayer& player, std::int32_t datatypeValue) {
    const auto datatype = static_cast<DatatypeEnum>(datatypeValue);
    auto proto = getProtoMessage(player, datatype);
    if(proto == nullptr) return nullptr;
    return getMessage(proto, datatype);
}
#else
static std::int32_t bridgeDatatypeFromSchema(const std::string&) {
    throw std::runtime_error("ReplayVideo node requires protobuf support");
}

static std::shared_ptr<Buffer> bridgeNextRecordedBuffer(utility::BytePlayer&, std::int32_t) {
    throw std::runtime_error("ReplayVideo node requires protobuf support");
}
#endif

}  // namespace node

namespace replay {

std::int32_t datatypeFromSchema(const std::string& schema) {
    return node::bridgeDatatypeFromSchema(schema);
}

std::shared_ptr<Buffer> nextRecordedBuffer(utility::BytePlayer& player, std::int32_t datatypeValue) {
    return node::bridgeNextRecordedBuffer(player, datatypeValue);
}

}  // namespace replay

namespace node {

void releaseEveryReplayPrepare(Pipeline& pipeline) {
    for(const auto& node : pipeline.getAllNodes()) {
        if(auto video = std::dynamic_pointer_cast<ReplayVideo>(node)) video->releaseAfterStop();
        if(auto metadata = std::dynamic_pointer_cast<ReplayMetadataOnly>(node)) metadata->releaseAfterStop();
    }
}

ReplayVideo::ReplayVideo() = default;

ReplayVideo::ReplayVideo(const std::shared_ptr<Device>& device) : DeviceNodeCRTP<DeviceNode, ReplayVideo, ReplayVideoProperties>(device) {}

ReplayVideo::ReplayVideo(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, ReplayVideo, ReplayVideoProperties>(std::move(props)) {}

ReplayVideo::~ReplayVideo() {
    releaseAfterStop();
}

void ReplayVideo::releaseAfterStop() {
    if(runtime == nullptr || runtime->released) return;
    runtime->released = true;
    if(runtime->onHost)
        replay::releaseReplayNode(runtime->nodeId, runtime->token);
    else if(runtime->device) {
        try {
            runtime->device->replayCacheRelease(runtime->cacheKeys);
        } catch(const std::exception& ex) {
            if(pimpl->logger) pimpl->logger->warn("Replay cache release failed: {}", ex.what());
        }
    }
    releaseToken(runtime->pipelineKey);
    prepared = false;
    runtime.reset();
}

void ReplayVideo::invalidatePrepare() {
    prepared = false;
    runtime.reset();
}

bool ReplayVideo::isSupportedOutFrameType(ImgFrame::Type type) {
    return replay::isReplayOutType(type);
}

void ReplayVideo::setReplayCacheLimit(std::uint64_t bytes) {
    replay::writeHostCacheLimit(bytes);
}

ReplayCacheInfo ReplayVideo::getReplayCacheInfo() {
    return replay::readHostCacheInfo();
}

void ReplayVideo::clearReplayCache() {
    replay::clearHostCache();
}

std::filesystem::path ReplayVideo::getReplayMetadataFile() const {
    return properties.metadataPath;
}

std::filesystem::path ReplayVideo::getReplayVideoFile() const {
    return properties.sourcePath;
}

ImgFrame::Type ReplayVideo::getOutFrameType() const {
    return static_cast<ImgFrame::Type>(properties.outFrameType);
}

bool ReplayVideo::getOutFrameTypeSet() const {
    return properties.outFrameTypeSet;
}

std::tuple<int, int> ReplayVideo::getSize() const {
    if(!properties.sizeSet) return {0, 0};
    return {properties.width, properties.height};
}

float ReplayVideo::getFps() const {
    if(!properties.fpsSet) return 0.f;
    return properties.fps;
}

bool ReplayVideo::getLoop() const {
    return properties.loop;
}

ReplaySourceLocation ReplayVideo::getSourceLocation() const {
    return static_cast<ReplaySourceLocation>(properties.sourceLocation);
}

ReplayDecodeMode ReplayVideo::getDecodeMode() const {
    return static_cast<ReplayDecodeMode>(properties.decodeMode);
}

bool ReplayVideo::getPersistentUpload() const {
    return properties.persistentUpload;
}

std::string ReplayVideo::getArchiveEntry() const {
    return properties.archiveEntry;
}

bool ReplayVideo::getStopPipelineAtEnd() const {
    return properties.stopPipelineAtEnd;
}

ReplayVideo& ReplayVideo::setReplayVideoFile(const std::filesystem::path& replayVideo) {
    return setSource(replayVideo, ReplaySourceLocation::HOST);
}

ReplayVideo& ReplayVideo::setReplayMetadataFile(const std::filesystem::path& replayFile) {
    return setMetadataFile(replayFile);
}

ReplayVideo& ReplayVideo::setOutFrameType(ImgFrame::Type outFrameType) {
    properties.outFrameType = static_cast<std::int32_t>(outFrameType);
    properties.outFrameTypeSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setSize(std::tuple<int, int> size) {
    properties.width = std::get<0>(size);
    properties.height = std::get<1>(size);
    properties.sizeSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setSize(int width, int height) {
    return setSize(std::make_tuple(width, height));
}

ReplayVideo& ReplayVideo::setFps(float fps) {
    properties.fps = fps;
    properties.fpsSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setLoop(bool loop) {
    properties.loop = loop;
    properties.loopSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setSource(const std::filesystem::path& path, ReplaySourceLocation location) {
    properties.sourcePath = path.string();
    properties.sourcePathSet = true;
    properties.sourceLocation = static_cast<std::int32_t>(location);
    properties.sourceLocationSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setMetadataFile(const std::filesystem::path& path) {
    properties.metadataPath = path.string();
    properties.metadataPathSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setDecodeMode(ReplayDecodeMode mode) {
    properties.decodeMode = static_cast<std::int32_t>(mode);
    properties.decodeModeSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setRunOnHost(bool runOnHost) {
    properties.runOnHostMode = runOnHost ? 1 : 2;
    return *this;
}

ReplayVideo& ReplayVideo::setPersistentUpload(bool persistent) {
    properties.persistentUpload = properties.persistentUpload || persistent;
    properties.persistentUploadSet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setArchiveEntry(const std::string& entry) {
    properties.archiveEntry = entry;
    properties.archiveEntrySet = true;
    return *this;
}

ReplayVideo& ReplayVideo::setStopPipelineAtEnd(bool stopPipeline) {
    properties.stopPipelineAtEnd = stopPipeline;
    properties.stopPipelineAtEndSet = true;
    return *this;
}

bool ReplayVideo::runOnHost() const {
    if(properties.runOnHostMode == 1) return true;
    if(properties.runOnHostMode == 2) return false;
    return !deviceIsRvc4(*this);
}

void ReplayVideo::buildStage1() {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
    auto& logger = pimpl->logger;
    try {
        if(prepared) return;
        checkFps(properties.fps, properties.fpsSet, "ReplayVideo");
        if(properties.sizeSet && (properties.width <= 0 || properties.height <= 0 || (properties.width % 2) != 0 || (properties.height % 2) != 0)) {
            throw std::runtime_error("ReplayVideo size " + std::to_string(properties.width) + "x" + std::to_string(properties.height) + " is not supported");
        }
        if(properties.outFrameTypeSet && !replay::isReplayOutType(static_cast<ImgFrame::Type>(properties.outFrameType))) {
            throw std::runtime_error(std::string("ReplayVideo output type is not supported: ")
                                     + replay::frameTypeName(static_cast<ImgFrame::Type>(properties.outFrameType)));
        }
        const bool onHost = runOnHost();
        if(properties.runOnHostMode == 2 && !deviceIsRvc4(*this)) {
            throw std::runtime_error("ReplayVideo setRunOnHost(false) is not supported without an RVC4 device");
        }
        if(!properties.sourcePathSet || properties.sourcePath.empty()) throw std::runtime_error("ReplayVideo source is not set");
        const auto location = static_cast<ReplaySourceLocation>(properties.sourceLocation);
        const std::filesystem::path sourcePath(properties.sourcePath);
        if(location == ReplaySourceLocation::DEVICE) {
            if(onHost) throw std::runtime_error("ReplayVideo DEVICE source cannot run on the host");
            if(!sourcePath.is_absolute()) throw std::runtime_error("ReplayVideo device path must be absolute: " + sourcePath.string());
        }
        applyEnv(properties.persistentUpload, properties.persistentUploadSet, properties.decodeMode, properties.decodeModeSet, "ReplayVideo");
        const bool predecode = properties.decodeMode == static_cast<std::int32_t>(ReplayDecodeMode::PRE_DECODE);

        runtime = std::make_unique<ReplayRuntime>();
        runtime->nodeId = static_cast<std::uint64_t>(id < 0 ? 0 : id);
        runtime->pipelineKey = parent.lock().get();
        runtime->token = acquireToken(runtime->pipelineKey);
        runtime->persistent = properties.persistentUpload;
        runtime->onHost = onHost;
        runtime->loop = properties.loop;
        runtime->stopAtEnd = properties.stopPipelineAtEnd;
        runtime->fpsSet = properties.fpsSet && properties.fps > 0.f && std::isfinite(properties.fps);
        runtime->fps = properties.fps;
        runtime->device = getDevice();

        if(!onHost) {
            if(location == ReplaySourceLocation::DEVICE) {
                properties.resolvedPath = sourcePath.string();
                properties.resolvedOnHost = false;
                properties.sourceHash.clear();
                if(!properties.sizeSet) {
                    const auto size = getDevice()->replayCacheSourceSize(sourcePath.string());
                    properties.resolvedWidth = std::get<0>(size);
                    properties.resolvedHeight = std::get<1>(size);
                    properties.resolvedFps = static_cast<float>(std::get<2>(size));
                }
                prepared = true;
                return;
            }
            if(std::filesystem::is_directory(sourcePath)) {
                throw std::runtime_error("ReplayVideo directory source must be archived before device replay: " + sourcePath.string());
            }
        }

        if(onHost && getDevice() && getDevice()->getProtocol() == X_LINK_LOCAL_SHDMEM) replay::setReplayCacheStandalone(true);

        std::filesystem::path local = sourcePath;
        std::string hash;
        const bool archive = !std::filesystem::is_directory(sourcePath) && ([&]() {
            auto ext = sourcePath.filename().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            return ext.size() >= 4 && (ext.rfind(".tar") != std::string::npos || ext.rfind(".tgz") != std::string::npos);
        })();
        if(!std::filesystem::exists(sourcePath)) throw std::runtime_error("ReplayVideo source is missing: " + sourcePath.string());

        if(onHost && archive) {
            hash = replay::sha256OfFile(sourcePath);
            const auto peak = static_cast<std::uint64_t>(std::filesystem::file_size(sourcePath)) + archiveContentBytes(sourcePath);
            replay::CacheSlotRequest request;
            request.key = hash;
            request.kind = ReplayCacheKind::SOURCE;
            request.sourceHash = hash;
            request.sizeBytes = peak;
            request.diskBytes = peak;
            request.ownsFiles = true;
            auto slot = replay::beginCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
            try {
                if(!slot.hit) {
                    const auto listing = replay::extractReplayArchive(sourcePath, slot.path);
                    request.sizeBytes = listing.entryBytes;
                    request.diskBytes = listing.entryBytes;
                    replay::commitCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
                    if(logger) logger->info("Replay cache upload {} {} bytes", hash, listing.entryBytes);
                } else {
                    logLines(logger, slot.logs);
                }
            } catch(...) {
                if(!slot.hit) replay::abortCacheEntry(request.key);
                throw;
            }
            runtime->cacheKeys.push_back(hash);
            local = properties.archiveEntrySet ? replay::resolveArchiveEntry(slot.path, properties.archiveEntry, false) : replay::selectReplayArchiveSource(slot.path);
        } else if(onHost && std::filesystem::is_directory(sourcePath)) {
            hash = replay::sha256OfDirectory(sourcePath);
            replay::CacheSlotRequest request;
            request.key = hash;
            request.kind = ReplayCacheKind::SOURCE;
            request.sourceHash = hash;
            request.ownsFiles = false;
            request.externalPath = sourcePath;
            auto listedPreview = replay::inspectReplaySource(sourcePath);
            request.sizeBytes = listedPreview.bytes;
            request.diskBytes = 0;
            auto slot = acquireSlot(*runtime, logger, request, false);
            local = properties.archiveEntrySet ? replay::resolveArchiveEntry(sourcePath, properties.archiveEntry, false) : sourcePath;
            (void)slot;
        } else if(onHost) {
            std::filesystem::path metadata = properties.metadataPathSet ? std::filesystem::path(properties.metadataPath) : std::filesystem::path{};
            hash = (!metadata.empty() && std::filesystem::exists(metadata)) ? replay::sha256OfFileAndMetadata(sourcePath, metadata)
                                                                            : replay::sha256OfFile(sourcePath);
            replay::CacheSlotRequest request;
            request.key = hash;
            request.kind = ReplayCacheKind::SOURCE;
            request.sourceHash = hash;
            request.sizeBytes = static_cast<std::uint64_t>(std::filesystem::file_size(sourcePath));
            if(!metadata.empty() && std::filesystem::exists(metadata)) request.sizeBytes += static_cast<std::uint64_t>(std::filesystem::file_size(metadata));
            request.diskBytes = 0;
            request.ownsFiles = false;
            request.externalPath = sourcePath;
            acquireSlot(*runtime, logger, request, false);
            local = sourcePath;
        } else {
            hash = replay::sha256OfFile(sourcePath);
            properties.resolvedPath = uploadFile(*getDevice(), logger, sourcePath, hash);
            properties.sourceHash = hash;
            properties.resolvedOnHost = false;
            runtime->cacheKeys.push_back(hash);
            // The device opens resolvedMetadataPath. The host path in metadataPath is not
            // on the device, so the sidecar has to be uploaded before the early return.
            if(properties.metadataPathSet) {
                const std::filesystem::path metadataPath(properties.metadataPath);
                if(!std::filesystem::exists(metadataPath)) {
                    throw std::runtime_error("ReplayVideo metadata file is missing: " + metadataPath.string());
                }
                const auto metadataHash = replay::sha256OfFile(metadataPath);
                properties.resolvedMetadataPath = uploadFile(*getDevice(), logger, metadataPath, metadataHash);
                runtime->cacheKeys.push_back(metadataHash);
            }
        }

        if(!onHost) {
            prepared = true;
            properties.sourceHash = hash;
            return;
        }

        runtime->listed = replay::inspectReplaySource(local);
        if(runtime->listed.kind == replay::ListedSource::Kind::Frames) {
            const auto marker = runtime->listed.path / (".dai_closed_" + runtime->listed.stream);
            if(std::filesystem::exists(marker)) {
                std::ifstream input(marker);
                std::string text;
                std::getline(input, text);
                while(!text.empty() && (text.back() == '\r' || text.back() == ' ')) text.pop_back();
                if(text == "open") throw std::runtime_error("ReplayVideo recording is still open: " + marker.string());
            }
        }
        if(!properties.sizeSet && ((runtime->listed.width % 2) != 0 || (runtime->listed.height % 2) != 0)) {
            throw std::runtime_error("ReplayVideo size " + std::to_string(runtime->listed.width) + "x" + std::to_string(runtime->listed.height)
                                     + " is not supported");
        }
        const bool keepDaiType = runtime->listed.dai && !runtime->listed.mjpegDai && !properties.outFrameTypeSet && !properties.sizeSet;
        if(keepDaiType) {
            runtime->outType = runtime->listed.sourceType;
            runtime->passthrough = runtime->listed.passthrough;
            runtime->outW = runtime->listed.width;
            runtime->outH = runtime->listed.height;
        } else {
            if(runtime->listed.passthrough) {
                throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replay::frameTypeName(runtime->listed.sourceType));
            }
            runtime->outType = properties.outFrameTypeSet ? static_cast<ImgFrame::Type>(properties.outFrameType)
                               : (runtime->listed.dai && !runtime->listed.mjpegDai && replay::isReplayOutType(runtime->listed.sourceType))
                                   ? runtime->listed.sourceType
                                   : ImgFrame::Type::YUV420p;
            if(!replay::isReplayOutType(runtime->outType)) {
                throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replay::frameTypeName(runtime->outType));
            }
            runtime->outW = properties.sizeSet ? properties.width : runtime->listed.width;
            runtime->outH = properties.sizeSet ? properties.height : runtime->listed.height;
        }
        runtime->count = runtime->listed.frameCount;
        if(runtime->listed.kind == replay::ListedSource::Kind::Mcap) {
            throw std::runtime_error("ReplayVideo source is metadata only: " + local.string());
        }

        std::filesystem::path metadataPath;
        if(runtime->listed.dai) {
            if(properties.metadataPathSet || !runtime->listed.autoMetadata.empty()) {
                runtime->metadataWarning = "ReplayVideo ignored metadata file for a .dai stream: "
                                           + (properties.metadataPathSet ? properties.metadataPath : runtime->listed.autoMetadata.string());
            }
        } else if(properties.metadataPathSet) {
            metadataPath = properties.metadataPath;
            if(!std::filesystem::exists(metadataPath)) throw std::runtime_error("ReplayVideo metadata file is missing: " + metadataPath.string());
        } else if(!runtime->listed.autoMetadata.empty()) {
            metadataPath = runtime->listed.autoMetadata;
        }
        if(!metadataPath.empty()) {
            auto scan = scanMcap(metadataPath, true, "ReplayVideo");
            if(scan.count != runtime->listed.frameCount) {
                throw std::runtime_error("ReplayVideo metadata count " + std::to_string(scan.count) + " does not match frame count "
                                         + std::to_string(runtime->listed.frameCount));
            }
            const auto message = scan.frames.front();
            if(message != nullptr
               && (static_cast<int>(message->getWidth()) != runtime->listed.width || static_cast<int>(message->getHeight()) != runtime->listed.height)) {
                throw std::runtime_error("ReplayVideo metadata frame size does not match: message 0 is " + std::to_string(message->getWidth()) + "x"
                                         + std::to_string(message->getHeight()));
            }
            runtime->metadata = std::move(scan.frames);
            runtime->recorded = true;
            runtime->seqFirst = scan.seqFirst;
            runtime->seqLast = scan.seqLast;
            runtime->deviceFirst = scan.deviceFirst;
            runtime->deviceLast = scan.deviceLast;
        }

        if(runtime->listed.kind == replay::ListedSource::Kind::Video) runtime->snaps.push_back(snapOf(runtime->listed.path));
        for(const auto& frame : runtime->listed.frames) runtime->snaps.push_back(snapOf(frame.path));

        runtime->reader.open(runtime->listed);
        const bool decodeAll = predecode || runtime->listed.dai;
        if(decodeAll) {
            std::filesystem::path decodedDir;
            std::string decodedKey;
            bool decodingToCache = false;
            if(predecode) {
                decodedKey = hash + "_" + runtime->listed.stream + "_" + replay::frameTypeName(runtime->outType) + "_" + std::to_string(runtime->outW) + "x"
                             + std::to_string(runtime->outH);
                replay::CacheSlotRequest request;
                request.key = decodedKey;
                request.kind = ReplayCacheKind::DECODED;
                request.sourceHash = hash;
                request.sizeBytes = static_cast<std::uint64_t>(runtime->count) * outputBytes(runtime->outType, runtime->outW, runtime->outH);
                request.diskBytes = request.sizeBytes;
                request.ownsFiles = true;
                auto slot = replay::beginCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
                decodedDir = slot.path;
                if(slot.hit) {
                    logLines(logger, slot.logs);
                    runtime->cacheKeys.push_back(decodedKey);
                    runtime->usePredecoded = true;
                    runtime->predecodedDir = decodedDir;
                    decodingToCache = false;
                } else {
                    decodingToCache = true;
                }
            }
            if(!runtime->usePredecoded) {
                try {
                    for(int index = 0; index < runtime->count; index++) {
                        checkSnaps(runtime->snaps);
                        auto pixels = runtime->reader.read(index);
                        if(runtime->listed.dai && metadataPath.empty()) noteDaiTiming(*runtime, pixels, index);
                        if(decodingToCache) {
                            auto frame = frameFromPixels(
                                pixels,
                                *runtime,
                                index < static_cast<int>(runtime->metadata.size()) ? runtime->metadata[static_cast<std::size_t>(index)] : nullptr);
                            const auto timing = timingFrom(
                                pixels, index < static_cast<int>(runtime->metadata.size()) ? runtime->metadata[static_cast<std::size_t>(index)] : nullptr);
                            if(timing.present) {
                                frame->setSequenceNum(timing.sequence);
                                frame->setTimestamp(timing.timestamp);
                                frame->setTimestampDevice(timing.timestampDevice);
                                frame->setTimestampSystem(timing.timestampSystem);
                            }
                            char name[32];
                            std::snprintf(name, sizeof(name), "frame_%08d.dai", index);
                            frame->save(decodedDir / name);
                            const auto done = static_cast<std::uint64_t>(index + 1);
                            if(runtime->count < 10 || done == static_cast<std::uint64_t>(runtime->count)
                               || (index + 1) % std::max(runtime->count / 10, 1) == 0) {
                                logProgress(logger, "decode", hash, done, static_cast<std::uint64_t>(runtime->count));
                            }
                        }
                    }
                    if(decodingToCache) {
                        replay::CacheSlotRequest request;
                        request.key = decodedKey;
                        request.kind = ReplayCacheKind::DECODED;
                        request.sourceHash = hash;
                        request.sizeBytes = static_cast<std::uint64_t>(runtime->count) * outputBytes(runtime->outType, runtime->outW, runtime->outH);
                        request.diskBytes = request.sizeBytes;
                        request.ownsFiles = true;
                        replay::commitCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
                        runtime->cacheKeys.push_back(decodedKey);
                        runtime->usePredecoded = true;
                        runtime->predecodedDir = decodedDir;
                    }
                } catch(...) {
                    if(decodingToCache) replay::abortCacheEntry(decodedKey);
                    throw;
                }
            }
        } else {
            checkSnaps(runtime->snaps);
            runtime->reader.read(0);
        }
        if(runtime->recorded) runtime->nominal = nominalOf(runtime->fpsSet, runtime->fps, runtime->deviceFirst, runtime->deviceLast, runtime->count);
        properties.resolvedPath = local.string();
        properties.resolvedMetadataPath = metadataPath.string();
        properties.sourceHash = hash;
        properties.resolvedWidth = runtime->outW;
        properties.resolvedHeight = runtime->outH;
        properties.resolvedType = static_cast<std::int32_t>(runtime->outType);
        properties.resolvedFps = runtime->fpsSet ? runtime->fps : static_cast<float>(runtime->listed.fps);
        properties.resolvedOnHost = true;
        prepared = true;
    } catch(...) {
        releaseAfterStop();
        throw;
    }
#endif
}

void ReplayVideo::run() {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
    auto& logger = pimpl->logger;
    if(!prepared || runtime == nullptr) throw std::runtime_error("ReplayVideo source is not prepared");
    if(!runtime->onHost) throw std::runtime_error("ReplayVideo device execution is running on the device");
    if(!runtime->metadataWarning.empty() && logger) logger->warn("{}", runtime->metadataWarning);
    if(!runtime->usePredecoded) runtime->reader.open(runtime->listed);
    replay::PlaybackPacer pacing(runtime->fpsSet ? runtime->fps : 0.f);
    std::optional<SteadyTime> previousDevice;
    bool naturalEnd = false;
    int sent = 0;
    while(mainLoop()) {
        const int index = runtime->count == 0 ? 0 : sent % runtime->count;
        const int pass = runtime->count == 0 ? 0 : sent / runtime->count;
        if(sent > 0 && index == 0) {
            if(!runtime->loop) {
                naturalEnd = true;
                break;
            }
        }
        checkSnaps(runtime->snaps);
        std::shared_ptr<ImgFrame> frame;
        RecordedTiming timing;
        if(runtime->usePredecoded) {
            char name[32];
            std::snprintf(name, sizeof(name), "frame_%08d.dai", index);
            frame = std::make_shared<ImgFrame>();
            frame->load(runtime->predecodedDir / name);
            timing.present = runtime->recorded;
            timing.sequence = frame->getSequenceNum();
            timing.timestamp = frame->getTimestamp();
            timing.timestampDevice = frame->getTimestampDevice();
            timing.timestampSystem = frame->getTimestampSystem();
        } else {
            auto pixels = runtime->reader.read(index);
            const auto metadata = index < static_cast<int>(runtime->metadata.size()) ? runtime->metadata[static_cast<std::size_t>(index)] : nullptr;
            frame = frameFromPixels(pixels, *runtime, metadata);
            timing = timingFrom(pixels, metadata);
        }
        if(runtime->recorded && timing.present)
            applyRecorded(frame, *runtime, timing, pass);
        else {
            frame->setSequenceNum(sent);
            frame->setTimestampSystem(std::nullopt);
            frame->sourceFb = frame->fb;
            if(frame->transformation.getSize().first == 0)
                frame->transformation = ImgTransformation(static_cast<size_t>(runtime->outW), static_cast<size_t>(runtime->outH));
        }
        std::optional<replay::PlaybackClock::duration> recordedInterval;
        if(runtime->recorded && timing.present) {
            if(index == 0 && sent > 0) recordedInterval = runtime->nominal;
            else if(previousDevice) recordedInterval = timing.timestampDevice - *previousDevice;
            previousDevice = timing.timestampDevice;
        }
        const auto sendTime = pacing.waitToSend(recordedInterval, [this] { return mainLoop(); });
        if(!sendTime) break;
        if(!runtime->recorded || !timing.present) {
            frame->setTimestamp(*sendTime);
            frame->setTimestampDevice(*sendTime);
        }
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(frame);
        }
        pacing.sent();
        sent++;
        if(!runtime->loop && sent >= runtime->count) {
            naturalEnd = true;
            break;
        }
    }
    if(naturalEnd && mainLoop()) {
        if(logger) logger->info("Replay finished");
        if(runtime->stopAtEnd && !runtime->loop) stopPipeline();
    }
#endif
}

ReplayMetadataOnly::ReplayMetadataOnly() = default;

ReplayMetadataOnly::ReplayMetadataOnly(const std::shared_ptr<Device>& device)
    : DeviceNodeCRTP<DeviceNode, ReplayMetadataOnly, ReplayMetadataOnlyProperties>(device) {}

ReplayMetadataOnly::ReplayMetadataOnly(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ReplayMetadataOnly, ReplayMetadataOnlyProperties>(std::move(props)) {}

ReplayMetadataOnly::~ReplayMetadataOnly() {
    releaseAfterStop();
}

void ReplayMetadataOnly::releaseAfterStop() {
    if(runtime == nullptr || runtime->released) return;
    runtime->released = true;
    if(runtime->onHost)
        replay::releaseReplayNode(runtime->nodeId, runtime->token);
    else if(runtime->device) {
        try {
            runtime->device->replayCacheRelease(runtime->cacheKeys);
        } catch(const std::exception& ex) {
            if(pimpl->logger) pimpl->logger->warn("Replay cache release failed: {}", ex.what());
        }
    }
    releaseToken(runtime->pipelineKey);
    prepared = false;
    runtime.reset();
}

void ReplayMetadataOnly::invalidatePrepare() {
    prepared = false;
    runtime.reset();
}

std::filesystem::path ReplayMetadataOnly::getReplayFile() const {
    return properties.sourcePath;
}
float ReplayMetadataOnly::getFps() const {
    return properties.fpsSet ? properties.fps : 0.f;
}
bool ReplayMetadataOnly::getLoop() const {
    return properties.loop;
}
ReplaySourceLocation ReplayMetadataOnly::getSourceLocation() const {
    return static_cast<ReplaySourceLocation>(properties.sourceLocation);
}
bool ReplayMetadataOnly::getPersistentUpload() const {
    return properties.persistentUpload;
}
std::string ReplayMetadataOnly::getArchiveEntry() const {
    return properties.archiveEntry;
}
bool ReplayMetadataOnly::getStopPipelineAtEnd() const {
    return properties.stopPipelineAtEnd;
}

ReplayMetadataOnly& ReplayMetadataOnly::setReplayFile(const std::filesystem::path& replayFile) {
    return setSource(replayFile, ReplaySourceLocation::HOST);
}
ReplayMetadataOnly& ReplayMetadataOnly::setFps(float fps) {
    properties.fps = fps;
    properties.fpsSet = true;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setLoop(bool loop) {
    properties.loop = loop;
    properties.loopSet = true;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setSource(const std::filesystem::path& path, ReplaySourceLocation location) {
    properties.sourcePath = path.string();
    properties.sourcePathSet = true;
    properties.sourceLocation = static_cast<std::int32_t>(location);
    properties.sourceLocationSet = true;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setRunOnHost(bool runOnHost) {
    properties.runOnHostMode = runOnHost ? 1 : 2;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setPersistentUpload(bool persistent) {
    properties.persistentUpload = properties.persistentUpload || persistent;
    properties.persistentUploadSet = true;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setArchiveEntry(const std::string& entry) {
    properties.archiveEntry = entry;
    properties.archiveEntrySet = true;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setStopPipelineAtEnd(bool stopPipeline) {
    properties.stopPipelineAtEnd = stopPipeline;
    properties.stopPipelineAtEndSet = true;
    return *this;
}

bool ReplayMetadataOnly::runOnHost() const {
    if(properties.runOnHostMode == 1) return true;
    if(properties.runOnHostMode == 2) return false;
    return !deviceIsRvc4(*this);
}

void ReplayMetadataOnly::buildStage1() {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    throw std::runtime_error("ReplayMetadataOnly node requires protobuf support");
#else
    auto& logger = pimpl->logger;
    try {
        if(prepared) return;
        checkFps(properties.fps, properties.fpsSet, "ReplayMetadataOnly");
        const bool onHost = runOnHost();
        if(properties.runOnHostMode == 2 && !deviceIsRvc4(*this)) {
            throw std::runtime_error("ReplayMetadataOnly setRunOnHost(false) is not supported without an RVC4 device");
        }
        if(!properties.sourcePathSet || properties.sourcePath.empty()) throw std::runtime_error("ReplayMetadataOnly source is not set");
        const auto location = static_cast<ReplaySourceLocation>(properties.sourceLocation);
        const std::filesystem::path sourcePath(properties.sourcePath);
        if(location == ReplaySourceLocation::DEVICE) {
            if(onHost) throw std::runtime_error("ReplayMetadataOnly DEVICE source cannot run on the host");
            if(!sourcePath.is_absolute()) throw std::runtime_error("ReplayMetadataOnly device path must be absolute: " + sourcePath.string());
        }
        std::int32_t ignoredDecode = 0;
        applyEnv(properties.persistentUpload, properties.persistentUploadSet, ignoredDecode, true, "ReplayMetadataOnly");
        runtime = std::make_unique<ReplayRuntime>();
        runtime->nodeId = static_cast<std::uint64_t>(id < 0 ? 0 : id);
        runtime->pipelineKey = parent.lock().get();
        runtime->token = acquireToken(runtime->pipelineKey);
        runtime->persistent = properties.persistentUpload;
        runtime->onHost = onHost;
        runtime->loop = properties.loop;
        runtime->stopAtEnd = properties.stopPipelineAtEnd;
        runtime->fpsSet = properties.fpsSet && properties.fps > 0.f && std::isfinite(properties.fps);
        runtime->fps = properties.fps;
        runtime->device = getDevice();
        if(!onHost && location == ReplaySourceLocation::DEVICE) {
            properties.resolvedPath = sourcePath.string();
            properties.resolvedOnHost = false;
            prepared = true;
            return;
        }
        if(!std::filesystem::exists(sourcePath)) throw std::runtime_error("ReplayMetadataOnly source is missing: " + sourcePath.string());
        if(onHost && getDevice() && getDevice()->getProtocol() == X_LINK_LOCAL_SHDMEM) replay::setReplayCacheStandalone(true);
        std::filesystem::path local = sourcePath;
        if(onHost && std::filesystem::is_directory(sourcePath)) {
            const auto hash = replay::sha256OfDirectory(sourcePath);
            replay::CacheSlotRequest request;
            request.key = hash;
            request.sourceHash = hash;
            request.kind = ReplayCacheKind::SOURCE;
            request.ownsFiles = false;
            request.externalPath = sourcePath;
            request.sizeBytes = 0;
            for(const auto& entry : std::filesystem::directory_iterator(sourcePath)) {
                if(entry.is_regular_file()) request.sizeBytes += static_cast<std::uint64_t>(entry.file_size());
            }
            acquireSlot(*runtime, logger, request, false);
            local = properties.archiveEntrySet ? replay::resolveArchiveEntry(sourcePath, properties.archiveEntry, true) : sourcePath;
        } else if(onHost) {
            auto ext = sourcePath.filename().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            const bool archive = ext.rfind(".tar") != std::string::npos || ext.rfind(".tgz") != std::string::npos;
            if(archive) {
                const auto hash = replay::sha256OfFile(sourcePath);
                const auto peak = static_cast<std::uint64_t>(std::filesystem::file_size(sourcePath)) + archiveContentBytes(sourcePath);
                replay::CacheSlotRequest request;
                request.key = hash;
                request.sourceHash = hash;
                request.kind = ReplayCacheKind::SOURCE;
                request.ownsFiles = true;
                request.sizeBytes = peak;
                request.diskBytes = peak;
                auto slot = replay::beginCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
                try {
                    if(!slot.hit) {
                        const auto listing = replay::extractReplayArchive(sourcePath, slot.path);
                        request.sizeBytes = listing.entryBytes;
                        request.diskBytes = listing.entryBytes;
                        replay::commitCacheEntry(runtime->nodeId, runtime->token, runtime->persistent, request);
                        if(logger) logger->info("Replay cache upload {} {} bytes", hash, listing.entryBytes);
                    } else
                        logLines(logger, slot.logs);
                } catch(...) {
                    if(!slot.hit) replay::abortCacheEntry(request.key);
                    throw;
                }
                runtime->cacheKeys.push_back(hash);
                local = properties.archiveEntrySet ? replay::resolveArchiveEntry(slot.path, properties.archiveEntry, true) : replay::selectReplayArchiveSource(slot.path);
            } else {
                const auto hash = replay::sha256OfFile(sourcePath);
                replay::CacheSlotRequest request;
                request.key = hash;
                request.sourceHash = hash;
                request.kind = ReplayCacheKind::SOURCE;
                request.ownsFiles = false;
                request.externalPath = sourcePath;
                request.sizeBytes = static_cast<std::uint64_t>(std::filesystem::file_size(sourcePath));
                acquireSlot(*runtime, logger, request, false);
            }
        } else {
            if(std::filesystem::is_directory(sourcePath)) {
                throw std::runtime_error("ReplayMetadataOnly directory source must be archived before device replay: " + sourcePath.string());
            }
            const auto hash = replay::sha256OfFile(sourcePath);
            properties.resolvedPath = uploadFile(*getDevice(), logger, sourcePath, hash);
            properties.sourceHash = hash;
            properties.resolvedOnHost = false;
            runtime->cacheKeys.push_back(hash);
            prepared = true;
            return;
        }
        if(std::filesystem::is_directory(local)) {
            if(!properties.archiveEntry.empty())
                local = replay::resolveArchiveEntry(local, properties.archiveEntry, true);
            else
                throw std::runtime_error("ReplayMetadataOnly source is not an mcap file: " + local.string());
        }
        auto scan = scanMcap(local, false, "ReplayMetadataOnly");
        runtime->mcapPath = local;
        runtime->count = scan.count;
        runtime->recorded = true;
        runtime->seqFirst = scan.seqFirst;
        runtime->seqLast = scan.seqLast;
        runtime->deviceFirst = scan.deviceFirst;
        runtime->deviceLast = scan.deviceLast;
        runtime->nominal = nominalOf(runtime->fpsSet, runtime->fps, runtime->deviceFirst, runtime->deviceLast, runtime->count);
        runtime->snaps.push_back(snapOf(local));
        properties.resolvedPath = local.string();
        properties.sourceHash = replay::sha256OfFile(std::filesystem::is_directory(sourcePath) ? local : sourcePath);
        properties.resolvedOnHost = true;
        prepared = true;
    } catch(...) {
        releaseAfterStop();
        throw;
    }
#endif
}

void ReplayMetadataOnly::run() {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    throw std::runtime_error("ReplayMetadataOnly node requires protobuf support");
#else
    auto& logger = pimpl->logger;
    if(!prepared || runtime == nullptr || runtime->mcapPath.empty()) throw std::runtime_error("ReplayMetadataOnly source is not prepared");
    if(!runtime->onHost) throw std::runtime_error("ReplayMetadataOnly device execution is running on the device");
    utility::BytePlayer player;
    const auto schema = player.init(runtime->mcapPath.string());
    const auto datatype = utility::schemaNameToDatatype(schema);
    replay::PlaybackPacer pacing(runtime->fpsSet ? runtime->fps : 0.f);
    bool naturalEnd = false;
    int sent = 0;
    SteadyTime previousDevice{};
    bool havePrevious = false;
    while(mainLoop()) {
        const int index = sent % std::max(runtime->count, 1);
        if(sent > 0 && index == 0) {
            if(!runtime->loop) {
                naturalEnd = true;
                break;
            }
            player.restart();
            havePrevious = false;
        }
        auto proto = getProtoMessage(player, datatype);
        if(proto == nullptr) {
            if(!runtime->loop) {
                naturalEnd = true;
                break;
            }
            player.restart();
            havePrevious = false;
            continue;
        }
        auto buffer = getMessage(proto, datatype);
        RecordedTiming timing;
        timing.present = true;
        timing.sequence = buffer->getSequenceNum();
        timing.timestamp = buffer->getTimestamp();
        timing.timestampDevice = buffer->getTimestampDevice();
        timing.timestampSystem = buffer->getTimestampSystem();
        std::optional<replay::PlaybackClock::duration> recordedInterval;
        if(sent > 0) {
            if(index == 0) recordedInterval = runtime->nominal;
            else if(havePrevious) recordedInterval = timing.timestampDevice - previousDevice;
        }
        checkSnaps(runtime->snaps);
        const int pass = runtime->count == 0 ? 0 : sent / runtime->count;
        applyRecorded(buffer, *runtime, timing, pass);
        previousDevice = timing.timestampDevice;
        havePrevious = true;
        if(!pacing.waitToSend(recordedInterval, [this] { return mainLoop(); })) break;
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(buffer);
        }
        pacing.sent();
        sent++;
        if(!runtime->loop && sent >= runtime->count) {
            naturalEnd = true;
            break;
        }
    }
    if(naturalEnd && mainLoop()) {
        if(logger) logger->info("Replay finished");
        if(runtime->stopAtEnd && !runtime->loop) stopPipeline();
    }
#endif
}

}  // namespace node
}  // namespace dai
