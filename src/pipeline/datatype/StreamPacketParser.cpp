#include "StreamPacketParser.hpp"

// standard
#include <memory>
#include <sstream>

// libraries
#include <XLink/XLinkPublicDefines.h>
#include <spdlog/spdlog.h>

#include <nlohmann/json.hpp>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"

// shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai-shared/datatype/RawEdgeDetectorConfig.hpp"
#include "depthai-shared/datatype/RawIMUData.hpp"
#include "depthai-shared/datatype/RawImageManipConfig.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"
#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"
#include "depthai-shared/datatype/RawSpatialLocations.hpp"
#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "depthai-shared/datatype/RawSystemInformation.hpp"
#include "depthai-shared/datatype/RawTracklets.hpp"

// StreamPacket structure ->  || imgframepixels... , serialized_object, object_type, serialized_object_size ||
// object_type -> DataType(int), serialized_object_size -> int

namespace dai {

// Reads int from little endian format
inline int readIntLE(uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

template <class T>
inline std::shared_ptr<T> parseDatatype(nlohmann::json& ser, std::vector<uint8_t>& data) {
    auto tmp = std::make_shared<T>();
    nlohmann::from_json(ser, *tmp);
    tmp->data = std::move(data);
    return tmp;
}

std::shared_ptr<RawBuffer> parsePacket(streamPacketDesc_t* packet) {
    int serializedObjectSize = readIntLE(packet->data + packet->length - 4);
    auto objectType = static_cast<DatatypeEnum>(readIntLE(packet->data + packet->length - 8));

    if(serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse");
    }
    std::uint32_t bufferLength = packet->length - 8 - serializedObjectSize;
    auto* msgpackStart = packet->data + bufferLength;

    nlohmann::json jser = nlohmann::json::from_msgpack(msgpackStart, msgpackStart + serializedObjectSize);

    // copy data part
    std::vector<uint8_t> data(packet->data, packet->data + bufferLength);

    // Create corresponding object
    switch(objectType) {
        // RawBuffer is special case, no metadata is actually serialized
        case DatatypeEnum::Buffer: {
            // RawBuffer is special case, no metadata is actually serialized
            auto pBuf = std::make_shared<RawBuffer>();
            pBuf->data = std::move(data);
            return pBuf;
        } break;

        case DatatypeEnum::ImgFrame:
            return parseDatatype<RawImgFrame>(jser, data);
            break;

        case DatatypeEnum::NNData:
            return parseDatatype<RawNNData>(jser, data);
            break;

        case DatatypeEnum::ImageManipConfig:
            return parseDatatype<RawImageManipConfig>(jser, data);
            break;

        case DatatypeEnum::CameraControl:
            return parseDatatype<RawCameraControl>(jser, data);
            break;

        case DatatypeEnum::ImgDetections:
            return parseDatatype<RawImgDetections>(jser, data);
            break;

        case DatatypeEnum::SpatialImgDetections:
            return parseDatatype<RawSpatialImgDetections>(jser, data);
            break;

        case DatatypeEnum::SystemInformation:
            return parseDatatype<RawSystemInformation>(jser, data);
            break;

        case DatatypeEnum::SpatialLocationCalculatorData:
            return parseDatatype<RawSpatialLocations>(jser, data);
            break;

        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return parseDatatype<RawSpatialLocationCalculatorConfig>(jser, data);
            break;

        case DatatypeEnum::Tracklets:
            return parseDatatype<RawTracklets>(jser, data);
            break;

        case DatatypeEnum::IMUData:
            return parseDatatype<RawIMUData>(jser, data);
            break;

        case DatatypeEnum::StereoDepthConfig:
            return parseDatatype<RawStereoDepthConfig>(jser, data);
            break;

        case DatatypeEnum::EdgeDetectorConfig:
            return parseDatatype<RawEdgeDetectorConfig>(jser, data);
            break;
    }

    throw std::runtime_error("Bad packet, couldn't parse");
}

std::shared_ptr<ADatatype> parsePacketToADatatype(streamPacketDesc_t* packet) {
    int serializedObjectSize = readIntLE(packet->data + packet->length - 4);
    auto objectType = static_cast<DatatypeEnum>(readIntLE(packet->data + packet->length - 8));

    if(serializedObjectSize < 0) {
        throw std::runtime_error("Bad packet, couldn't parse");
    }
    std::uint32_t bufferLength = packet->length - 8 - serializedObjectSize;
    auto* msgpackStart = packet->data + bufferLength;

    nlohmann::json jser = nlohmann::json::from_msgpack(msgpackStart, msgpackStart + serializedObjectSize);

    // copy data part
    std::vector<uint8_t> data(packet->data, packet->data + bufferLength);

    switch(objectType) {
        case DatatypeEnum::Buffer: {
            // RawBuffer is special case, no metadata is actually serialized
            auto pBuf = std::make_shared<RawBuffer>();
            pBuf->data = std::move(data);
            return std::make_shared<Buffer>(pBuf);
        } break;

        case DatatypeEnum::ImgFrame:
            return std::make_shared<ImgFrame>(parseDatatype<RawImgFrame>(jser, data));
            break;

        case DatatypeEnum::NNData:
            return std::make_shared<NNData>(parseDatatype<RawNNData>(jser, data));
            break;

        case DatatypeEnum::ImageManipConfig:
            return std::make_shared<ImageManipConfig>(parseDatatype<RawImageManipConfig>(jser, data));
            break;

        case DatatypeEnum::CameraControl:
            return std::make_shared<CameraControl>(parseDatatype<RawCameraControl>(jser, data));
            break;

        case DatatypeEnum::ImgDetections:
            return std::make_shared<ImgDetections>(parseDatatype<RawImgDetections>(jser, data));
            break;

        case DatatypeEnum::SpatialImgDetections:
            return std::make_shared<SpatialImgDetections>(parseDatatype<RawSpatialImgDetections>(jser, data));
            break;

        case DatatypeEnum::SystemInformation:
            return std::make_shared<SystemInformation>(parseDatatype<RawSystemInformation>(jser, data));
            break;

        case DatatypeEnum::SpatialLocationCalculatorData:
            return std::make_shared<SpatialLocationCalculatorData>(parseDatatype<RawSpatialLocations>(jser, data));
            break;

        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return std::make_shared<SpatialLocationCalculatorConfig>(parseDatatype<RawSpatialLocationCalculatorConfig>(jser, data));
            break;

        case DatatypeEnum::Tracklets:
            return std::make_shared<Tracklets>(parseDatatype<RawTracklets>(jser, data));
            break;

        case DatatypeEnum::IMUData:
            return std::make_shared<IMUData>(parseDatatype<RawIMUData>(jser, data));
            break;

        case DatatypeEnum::StereoDepthConfig:
            return std::make_shared<StereoDepthConfig>(parseDatatype<RawStereoDepthConfig>(jser, data));
            break;

        case DatatypeEnum::EdgeDetectorConfig:
            return std::make_shared<EdgeDetectorConfig>(parseDatatype<RawEdgeDetectorConfig>(jser, data));
            break;
    }

    throw std::runtime_error("Bad packet, couldn't parse");
}

std::vector<std::uint8_t> serializeData(const std::shared_ptr<RawBuffer>& data) {
    std::vector<std::uint8_t> ser;
    if(!data) return ser;

    // Serialization:
    // 1. fill vector with bytes from data.data
    // 2. serialize and append metadata
    // 3. append datatype enum (4B LE)
    // 4. append size (4B LE) of serialized metadata

    std::vector<std::uint8_t> metadata;
    DatatypeEnum datatype;
    data->serialize(metadata, datatype);
    uint32_t metadataSize = static_cast<uint32_t>(metadata.size());

    // 4B datatype & 4B metadata size
    std::uint8_t leDatatype[4];
    std::uint8_t leMetadataSize[4];
    for(int i = 0; i < 4; i++) leDatatype[i] = (static_cast<std::int32_t>(datatype) >> (i * 8)) & 0xFF;
    for(int i = 0; i < 4; i++) leMetadataSize[i] = (metadataSize >> i * 8) & 0xFF;

    ser.insert(ser.end(), data->data.begin(), data->data.end());
    ser.insert(ser.end(), metadata.begin(), metadata.end());
    ser.insert(ser.end(), leDatatype, leDatatype + sizeof(leDatatype));
    ser.insert(ser.end(), leMetadataSize, leMetadataSize + sizeof(leMetadataSize));

    return ser;
}

}  // namespace dai
