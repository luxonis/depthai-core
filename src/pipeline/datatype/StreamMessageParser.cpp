#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

// standard
#include <memory>
#include <sstream>

// libraries
#include <XLink/XLinkPublicDefines.h>
#include <spdlog/spdlog.h>

#include "utility/Logging.hpp"

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/AprilTagConfig.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"

// shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawAprilTagConfig.hpp"
#include "depthai-shared/datatype/RawAprilTags.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai-shared/datatype/RawEdgeDetectorConfig.hpp"
#include "depthai-shared/datatype/RawEncodedFrame.hpp"
#include "depthai-shared/datatype/RawFeatureTrackerConfig.hpp"
#include "depthai-shared/datatype/RawIMUData.hpp"
#include "depthai-shared/datatype/RawImageAlignConfig.hpp"
#include "depthai-shared/datatype/RawImageManipConfig.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/datatype/RawMessageGroup.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai-shared/datatype/RawPointCloudConfig.hpp"
#include "depthai-shared/datatype/RawPointCloudData.hpp"
#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"
#include "depthai-shared/datatype/RawSpatialLocations.hpp"
#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "depthai-shared/datatype/RawSystemInformation.hpp"
#include "depthai-shared/datatype/RawToFConfig.hpp"
#include "depthai-shared/datatype/RawTracklets.hpp"
#include "depthai-shared/utility/Serialization.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {

static constexpr std::array<uint8_t, 16> endOfPacketMarker = {0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};

// Reads int from little endian format
inline int readIntLE(uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

template <class T>
inline std::shared_ptr<T> parseDatatype(std::uint8_t* metadata, size_t size, std::vector<uint8_t>& data) {
    auto tmp = std::make_shared<T>();

    // deserialize
    utility::deserialize(metadata, size, *tmp);
    // Move data
    tmp->data = std::move(data);

    return tmp;
}

static std::tuple<DatatypeEnum, size_t, size_t> parseHeader(streamPacketDesc_t* const packet) {
    if(packet->length < 24) {
        throw std::runtime_error(fmt::format("Bad packet, couldn't parse (not enough data), total size {}", packet->length));
    }
    const std::uint32_t packetLength = packet->length - endOfPacketMarker.size();
    const int serializedObjectSize = readIntLE(packet->data + packetLength - 4);
    const auto objectType = static_cast<DatatypeEnum>(readIntLE(packet->data + packetLength - 8));

    uint8_t* marker = packet->data + packetLength;
    if(memcmp(marker, endOfPacketMarker.data(), endOfPacketMarker.size()) != 0) {
        std::string hex;
        for(std::uint32_t i = 0; i < endOfPacketMarker.size(); i++) {
            hex += fmt::format("{:02X}", marker[i]);
        }
        logger::warn("StreamMessageParser end-of-packet marker mismatch, got: " + hex);
    }

    const auto info = fmt::format(", total size {}, type {}, metadata size {}", packet->length, objectType, serializedObjectSize);

    if(serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata size negative)" + info);
    } else if(serializedObjectSize > static_cast<int>(packetLength)) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata size larger than packet length)" + info);
    }
    if(static_cast<int>(packetLength) - 8 - serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse (data too small)" + info);
    }
    const std::uint32_t bufferLength = packetLength - 8 - serializedObjectSize;
    if(bufferLength > packetLength) {
        throw std::runtime_error("Bad packet, couldn't parse (data too large)" + info);
    }
    auto* const metadataStart = packet->data + bufferLength;

    if(metadataStart < packet->data || metadataStart >= packet->data + packetLength) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata out of bounds)" + info);
    }

    return {objectType, serializedObjectSize, bufferLength};
}

std::shared_ptr<RawBuffer> StreamMessageParser::parseMessage(streamPacketDesc_t* const packet) {
    DatatypeEnum objectType;
    size_t serializedObjectSize;
    size_t bufferLength;
    std::tie(objectType, serializedObjectSize, bufferLength) = parseHeader(packet);
    auto* const metadataStart = packet->data + bufferLength;

    // copy data part
    std::vector<uint8_t> data(packet->data, packet->data + bufferLength);

    // Create corresponding object
    switch(objectType) {
        case DatatypeEnum::Buffer:
            return parseDatatype<RawBuffer>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::ImgFrame:
            return parseDatatype<RawImgFrame>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::EncodedFrame:
            return parseDatatype<RawEncodedFrame>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::NNData:
            return parseDatatype<RawNNData>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::ImageManipConfig:
            return parseDatatype<RawImageManipConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::CameraControl:
            return parseDatatype<RawCameraControl>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::ImgDetections:
            return parseDatatype<RawImgDetections>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::SpatialImgDetections:
            return parseDatatype<RawSpatialImgDetections>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::SystemInformation:
            return parseDatatype<RawSystemInformation>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::SpatialLocationCalculatorData:
            return parseDatatype<RawSpatialLocations>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return parseDatatype<RawSpatialLocationCalculatorConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::AprilTags:
            return parseDatatype<RawAprilTags>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::AprilTagConfig:
            return parseDatatype<RawAprilTagConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::Tracklets:
            return parseDatatype<RawTracklets>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::IMUData:
            return parseDatatype<RawIMUData>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::StereoDepthConfig:
            return parseDatatype<RawStereoDepthConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::EdgeDetectorConfig:
            return parseDatatype<RawEdgeDetectorConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::TrackedFeatures:
            return parseDatatype<RawTrackedFeatures>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::FeatureTrackerConfig:
            return parseDatatype<RawFeatureTrackerConfig>(metadataStart, serializedObjectSize, data);
            break;

        case DatatypeEnum::ToFConfig:
            return parseDatatype<RawToFConfig>(metadataStart, serializedObjectSize, data);
            break;
        case DatatypeEnum::PointCloudConfig:
            return parseDatatype<RawPointCloudConfig>(metadataStart, serializedObjectSize, data);
            break;
        case DatatypeEnum::PointCloudData:
            return parseDatatype<RawPointCloudData>(metadataStart, serializedObjectSize, data);
            break;
        case DatatypeEnum::MessageGroup:
            return parseDatatype<RawMessageGroup>(metadataStart, serializedObjectSize, data);
            break;
        case DatatypeEnum::ImageAlignConfig:
            return parseDatatype<RawImageAlignConfig>(metadataStart, serializedObjectSize, data);
            break;
    }

    throw std::runtime_error(
        fmt::format("Bad packet, couldn't parse, total size {}, type {}, metadata size {}", packet->length, objectType, serializedObjectSize));
}

std::shared_ptr<ADatatype> StreamMessageParser::parseMessageToADatatype(streamPacketDesc_t* const packet, DatatypeEnum& objectType) {
    size_t serializedObjectSize;
    size_t bufferLength;
    std::tie(objectType, serializedObjectSize, bufferLength) = parseHeader(packet);
    auto* const metadataStart = packet->data + bufferLength;

    // copy data part
    std::vector<uint8_t> data(packet->data, packet->data + bufferLength);

    switch(objectType) {
        case DatatypeEnum::Buffer: {
            return std::make_shared<Buffer>(parseDatatype<RawBuffer>(metadataStart, serializedObjectSize, data));
        } break;

        case DatatypeEnum::ImgFrame:
            return std::make_shared<ImgFrame>(parseDatatype<RawImgFrame>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::EncodedFrame:
            return std::make_shared<EncodedFrame>(parseDatatype<RawEncodedFrame>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::NNData:
            return std::make_shared<NNData>(parseDatatype<RawNNData>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::ImageManipConfig:
            return std::make_shared<ImageManipConfig>(parseDatatype<RawImageManipConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::CameraControl:
            return std::make_shared<CameraControl>(parseDatatype<RawCameraControl>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::ImgDetections:
            return std::make_shared<ImgDetections>(parseDatatype<RawImgDetections>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::SpatialImgDetections:
            return std::make_shared<SpatialImgDetections>(parseDatatype<RawSpatialImgDetections>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::SystemInformation:
            return std::make_shared<SystemInformation>(parseDatatype<RawSystemInformation>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::SpatialLocationCalculatorData:
            return std::make_shared<SpatialLocationCalculatorData>(parseDatatype<RawSpatialLocations>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return std::make_shared<SpatialLocationCalculatorConfig>(
                parseDatatype<RawSpatialLocationCalculatorConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::AprilTags:
            return std::make_shared<AprilTags>(parseDatatype<RawAprilTags>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::AprilTagConfig:
            return std::make_shared<AprilTagConfig>(parseDatatype<RawAprilTagConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::Tracklets:
            return std::make_shared<Tracklets>(parseDatatype<RawTracklets>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::IMUData:
            return std::make_shared<IMUData>(parseDatatype<RawIMUData>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::StereoDepthConfig:
            return std::make_shared<StereoDepthConfig>(parseDatatype<RawStereoDepthConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::EdgeDetectorConfig:
            return std::make_shared<EdgeDetectorConfig>(parseDatatype<RawEdgeDetectorConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::TrackedFeatures:
            return std::make_shared<TrackedFeatures>(parseDatatype<RawTrackedFeatures>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::FeatureTrackerConfig:
            return std::make_shared<FeatureTrackerConfig>(parseDatatype<RawFeatureTrackerConfig>(metadataStart, serializedObjectSize, data));
            break;

        case DatatypeEnum::ToFConfig:
            return std::make_shared<ToFConfig>(parseDatatype<RawToFConfig>(metadataStart, serializedObjectSize, data));
            break;
        case DatatypeEnum::PointCloudConfig:
            return std::make_shared<PointCloudConfig>(parseDatatype<RawPointCloudConfig>(metadataStart, serializedObjectSize, data));
            break;
        case DatatypeEnum::PointCloudData:
            return std::make_shared<PointCloudData>(parseDatatype<RawPointCloudData>(metadataStart, serializedObjectSize, data));
            break;
        case DatatypeEnum::MessageGroup:
            return std::make_shared<MessageGroup>(parseDatatype<RawMessageGroup>(metadataStart, serializedObjectSize, data));
            break;
        case DatatypeEnum::ImageAlignConfig:
            return std::make_shared<ImageAlignConfig>(parseDatatype<RawImageAlignConfig>(metadataStart, serializedObjectSize, data));
            break;
    }

    throw std::runtime_error(fmt::format(
        "Bad packet, couldn't parse (invalid message type), total size {}, type {}, metadata size {}", packet->length, objectType, serializedObjectSize));
}

std::shared_ptr<ADatatype> StreamMessageParser::parseMessageToADatatype(streamPacketDesc_t* const packet) {
    DatatypeEnum objectType;
    return parseMessageToADatatype(packet, objectType);
}

std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const RawBuffer& data) {
    // Serialization:
    // 1. fill vector with bytes from data.data
    // 2. serialize and append metadata
    // 3. append datatype enum (4B LE)
    // 4. append size (4B LE) of serialized metadata
    // 5. append 16-byte marker/canary

    DatatypeEnum datatype;
    std::vector<std::uint8_t> metadata;
    data.serialize(metadata, datatype);
    uint32_t metadataSize = static_cast<uint32_t>(metadata.size());

    // 4B datatype & 4B metadata size
    std::array<std::uint8_t, 4> leDatatype;
    std::array<std::uint8_t, 4> leMetadataSize;
    for(int i = 0; i < 4; i++) leDatatype[i] = (static_cast<std::int32_t>(datatype) >> (i * 8)) & 0xFF;
    for(int i = 0; i < 4; i++) leMetadataSize[i] = (metadataSize >> i * 8) & 0xFF;

    std::vector<std::uint8_t> ser;
    ser.reserve(data.data.size() + metadata.size() + leDatatype.size() + leMetadataSize.size() + endOfPacketMarker.size());
    ser.insert(ser.end(), data.data.begin(), data.data.end());
    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype.begin(), leDatatype.end());
    ser.insert(ser.end(), leMetadataSize.begin(), leMetadataSize.end());
    ser.insert(ser.end(), endOfPacketMarker.begin(), endOfPacketMarker.end());

    return ser;
}

std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const std::shared_ptr<const RawBuffer>& data) {
    if(!data) return {};
    return serializeMessage(*data);
}

std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const ADatatype& data) {
    return serializeMessage(data.serialize());
}

std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const std::shared_ptr<const ADatatype>& data) {
    if(!data) return {};
    return serializeMessage(*data);
}

}  // namespace dai
