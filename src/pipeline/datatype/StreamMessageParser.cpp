#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

// standard
#include <memory>
#include <sstream>

// libraries
#include <XLink/XLinkPublicDefines.h>
#include <spdlog/spdlog.h>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/AprilTagConfig.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
    #include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
    #include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#endif  // DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MapData.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/RGBDData.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/SystemInformationS3.hpp"
#include "depthai/pipeline/datatype/ThermalConfig.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
// shared
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "pipeline/datatype/ObjectTrackerConfig.hpp"
#include "utility/SharedMemory.hpp"
#include "utility/VectorMemory.hpp"
#include "xlink/XLinkStream.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {

static constexpr std::array<uint8_t, 16> endOfPacketMarker = {0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};

// Reads int from little endian format
inline int readIntLE(uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

template <class T>
inline std::shared_ptr<T> parseDatatype(std::uint8_t* metadata, size_t size, std::vector<uint8_t>& data, long fd) {
    auto tmp = std::make_shared<T>();

    // deserialize
    utility::deserialize(metadata, size, *tmp);
    if(fd < 0) {
        tmp->data = std::make_shared<dai::VectorMemory>(std::move(data));
    } else {
        tmp->data = std::make_shared<dai::SharedMemory>(fd);
    }

    return tmp;
}

static std::tuple<DatatypeEnum, size_t, size_t> parseHeader(streamPacketDesc_t* const packet) {
    if(packet == nullptr || packet->data == nullptr) {
        throw std::runtime_error("Bad packet, couldn't parse (null packet or data)");
    }
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
        // logger::warn("StreamMessageParser end-of-packet marker mismatch, got: " + hex);
    }

    const auto info = fmt::format(", total size {}, type {}, metadata size {}", packet->length, (int32_t)objectType, serializedObjectSize);

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

std::shared_ptr<ADatatype> StreamMessageParser::parseMessage(streamPacketDesc_t* const packet) {
    DatatypeEnum objectType;
    size_t serializedObjectSize;
    size_t bufferLength;
    long fd;
    std::tie(objectType, serializedObjectSize, bufferLength) = parseHeader(packet);
    auto* const metadataStart = packet->data + bufferLength;

    // copy data part
    if(packet->data == nullptr && bufferLength > 0) {
        throw std::runtime_error("Bad packet, couldn't parse (null data buffer)");
    }
    std::vector<uint8_t> data;
    if(bufferLength > 0) {
        data.assign(packet->data, packet->data + bufferLength);
    }

    fd = packet->fd;

    // Create corresponding object
    switch(objectType) {
        // ADatatype is a special case, since no metadata is actually serialized
        case DatatypeEnum::ADatatype: {
            auto pBuf = std::make_shared<ADatatype>();
            return pBuf;
        }
        case DatatypeEnum::Buffer:
            return parseDatatype<Buffer>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::ImgFrame:
            return parseDatatype<ImgFrame>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::EncodedFrame:
            return parseDatatype<EncodedFrame>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::NNData:
            return parseDatatype<NNData>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::ImageManipConfig:
            return parseDatatype<ImageManipConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::ImageAlignConfig:
            return parseDatatype<ImageAlignConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::CameraControl:
            return parseDatatype<CameraControl>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::ImgDetections:
            return parseDatatype<ImgDetections>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::SpatialImgDetections:
            return parseDatatype<SpatialImgDetections>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::SystemInformation:
            return parseDatatype<SystemInformation>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::SystemInformationS3:
            return parseDatatype<SystemInformationS3>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::SpatialLocationCalculatorData:
            return parseDatatype<SpatialLocationCalculatorData>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return parseDatatype<SpatialLocationCalculatorConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::AprilTags:
            return parseDatatype<AprilTags>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::AprilTagConfig:
            return parseDatatype<AprilTagConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::Tracklets:
            return parseDatatype<Tracklets>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::IMUData:
            return parseDatatype<IMUData>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::StereoDepthConfig:
            return parseDatatype<StereoDepthConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::NeuralDepthConfig:
            return parseDatatype<NeuralDepthConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::EdgeDetectorConfig:
            return parseDatatype<EdgeDetectorConfig>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::TrackedFeatures:
            return parseDatatype<TrackedFeatures>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::FeatureTrackerConfig:
            return parseDatatype<FeatureTrackerConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::BenchmarkReport:
            return parseDatatype<BenchmarkReport>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ThermalConfig:
            return parseDatatype<ThermalConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ToFConfig:
            return parseDatatype<ToFConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::PointCloudConfig:
            return parseDatatype<PointCloudConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::PointCloudData:
            return parseDatatype<PointCloudData>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::MapData:
            return parseDatatype<MapData>(metadataStart, serializedObjectSize, data, fd);
        case DatatypeEnum::MessageGroup:
            return parseDatatype<MessageGroup>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::TransformData:
            return parseDatatype<TransformData>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ImgAnnotations:
            return parseDatatype<ImgAnnotations>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ImageFiltersConfig:
            return parseDatatype<ImageFiltersConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
            return parseDatatype<ToFDepthConfidenceFilterConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::RGBDData:
            return parseDatatype<RGBDData>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::ObjectTrackerConfig:
            return parseDatatype<ObjectTrackerConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
        case DatatypeEnum::DynamicCalibrationControl:
            return parseDatatype<DynamicCalibrationControl>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::DynamicCalibrationResult:
            return parseDatatype<DynamicCalibrationResult>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::CalibrationQuality:
            return parseDatatype<CalibrationQuality>(metadataStart, serializedObjectSize, data, fd);
            break;

        case DatatypeEnum::CoverageData:
            return parseDatatype<CoverageData>(metadataStart, serializedObjectSize, data, fd);
            break;
#else
        // Explicitly enum these in this switch state:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CoverageData:
            break;
#endif  // DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
        default:
            break;
    }
    throw std::runtime_error("Bad packet, couldn't parse");
}

std::shared_ptr<ADatatype> StreamMessageParser::parseMessage(StreamPacketDesc packet) {
    return parseMessage(&packet);
}

std::vector<std::uint8_t> StreamMessageParser::serializeMetadata(const ADatatype& message) {
    // Serialization:
    // 1. fill vector with bytes from message.data
    // 2. serialize and append metadata
    // 3. append datatype enum (4B LE)
    // 4. append size (4B LE) of serialized metadata
    // 5. append 16-byte marker/canary

    DatatypeEnum datatype;
    std::vector<std::uint8_t> metadata;
    message.serialize(metadata, datatype);
    uint32_t metadataSize = static_cast<uint32_t>(metadata.size());

    // 4B datatype & 4B metadata size
    std::array<std::uint8_t, 4> leDatatype;
    std::array<std::uint8_t, 4> leMetadataSize;
    for(int i = 0; i < 4; i++) leDatatype[i] = (static_cast<std::int32_t>(datatype) >> (i * 8)) & 0xFF;
    for(int i = 0; i < 4; i++) leMetadataSize[i] = (metadataSize >> i * 8) & 0xFF;

    std::vector<std::uint8_t> ser;
    ser.reserve(metadata.size() + leDatatype.size() + leMetadataSize.size() + endOfPacketMarker.size());
    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype.begin(), leDatatype.end());
    ser.insert(ser.end(), leMetadataSize.begin(), leMetadataSize.end());
    ser.insert(ser.end(), endOfPacketMarker.begin(), endOfPacketMarker.end());

    return ser;
}

std::vector<std::uint8_t> StreamMessageParser::serializeMetadata(const std::shared_ptr<const ADatatype>& data) {
    if(!data) return {};
    return serializeMetadata(*data);
}

// std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const ADatatype& message) {
//     // Serialization:
//     // 1. fill vector with bytes from data.data
//     // 2. serialize and append metadata
//     // 3. append datatype enum (4B LE)
//     // 4. append size (4B LE) of serialized metadata

//     throw std::invalid_argument("TODO");

//     std::vector<std::uint8_t> metadata = serializeMetadata(message);
//     return metadata;

//     // std::vector<std::uint8_t> ser;
//     // ser.reserve(data.data.size() + metadata.size() + leDatatype.size() + leMetadataSize.size());
//     // ser.insert(ser.end(), data.data.begin(), data.data.end());
//     // ser.insert(ser.end(), metadata.begin(), metadata.end());
//     // ser.insert(ser.end(), leDatatype.begin(), leDatatype.end());
//     // ser.insert(ser.end(), leMetadataSize.begin(), leMetadataSize.end());
//     // return ser;
// }

// std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const std::shared_ptr<const ADatatype>& data) {
//     if(!data) return {};
//     return serializeMessage(*data);
// }

// std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const ADatatype& data) {
//     return serializeMessage(data.serialize());
// }

// std::vector<std::uint8_t> StreamMessageParser::serializeMessage(const std::shared_ptr<const ADatatype>& data) {
//     if(!data) return {};
//     return serializeMessage(*data);
// }

}  // namespace dai
