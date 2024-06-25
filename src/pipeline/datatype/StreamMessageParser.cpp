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
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
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
#include "depthai/pipeline/datatype/SystemInformationS3.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"

// shared
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/utility/Serialization.hpp"
#include "utility/VectorMemory.hpp"
#include "xlink/XLinkStream.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {

// Reads int from little endian format
inline int readIntLE(uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

template <class T>
inline std::shared_ptr<T> parseDatatype(std::uint8_t* metadata, size_t size, std::vector<uint8_t>& data, long fd) {
    auto tmp = std::make_shared<T>();

    // deserialize
    utility::deserialize(metadata, size, *tmp);
    // Move data - TODO(Morato) change this back to zero copy
    if (fd < 0) {
	tmp->data = std::make_shared<dai::VectorMemory>(std::move(data));
    } else {
	tmp->data = std::make_shared<dai::SharedMemory>(fd);
    }

    return tmp;
}

static std::tuple<DatatypeEnum, size_t, size_t> parseHeader(streamPacketDesc_t* const packet) {
    if(packet->length < 8) {
        throw std::runtime_error("Bad packet, couldn't parse (not enough data)");
    }
    const int serializedObjectSize = readIntLE(packet->data + packet->length - 4);
    const auto objectType = static_cast<DatatypeEnum>(readIntLE(packet->data + packet->length - 8));

    if(serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata size negative)");
    } else if(serializedObjectSize > static_cast<int>(packet->length)) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata size larger than packet length)");
    }
    if(static_cast<int>(packet->length) - 8 - serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse (data too small)");
    }
    const std::uint32_t bufferLength = packet->length - 8 - serializedObjectSize;
    if(bufferLength > packet->length) {
        throw std::runtime_error("Bad packet, couldn't parse (data too large)");
    }
    auto* const metadataStart = packet->data + bufferLength;

    if(metadataStart < packet->data || metadataStart >= packet->data + packet->length) {
        throw std::runtime_error("Bad packet, couldn't parse (metadata out of bounds)");
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
    std::vector<uint8_t> data(packet->data, packet->data + bufferLength);

    fd = packet->fd;

    // Create corresponding object
    switch(objectType) {
        // ADatatype is a special case, since no metadata is actually serialized
        case DatatypeEnum::ADatatype: {
            auto pBuf = std::make_shared<ADatatype>();
            return pBuf;
        }
        case DatatypeEnum::Buffer: {
            return parseDatatype<Buffer>(metadataStart, serializedObjectSize, data, fd);
            break;
        }

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
        case DatatypeEnum::ToFConfig:
            return parseDatatype<ToFConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::PointCloudConfig:
            return parseDatatype<PointCloudConfig>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::PointCloudData:
            return parseDatatype<PointCloudData>(metadataStart, serializedObjectSize, data, fd);
            break;
        case DatatypeEnum::MessageGroup:
            return parseDatatype<MessageGroup>(metadataStart, serializedObjectSize, data, fd);
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
    ser.reserve(metadata.size() + leDatatype.size() + leMetadataSize.size());
    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype.begin(), leDatatype.end());
    ser.insert(ser.end(), leMetadataSize.begin(), leMetadataSize.end());

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
