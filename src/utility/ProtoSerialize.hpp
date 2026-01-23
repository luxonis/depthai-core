#pragma once
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <memory>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatypes.hpp"
#include "depthai/schemas/EncodedFrame.pb.h"
#include "depthai/schemas/IMUData.pb.h"
#include "depthai/schemas/ImageAnnotations.pb.h"
#include "depthai/schemas/ImgDetections.pb.h"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/schemas/PointCloudData.pb.h"
#include "depthai/schemas/SpatialImgDetections.pb.h"
#include "depthai/schemas/common.pb.h"
#include "utility/ProtoSerializable.hpp"

namespace dai {
namespace utility {

std::vector<std::uint8_t> serializeProto(std::unique_ptr<google::protobuf::Message> protoMessage);
ProtoSerializable::SchemaPair serializeSchema(std::unique_ptr<google::protobuf::Message> protoMessage);

// Common functions for serializing
void serializeImgTransformation(proto::common::ImgTransformation* imgTransformation, const ImgTransformation& transformation);
ImgTransformation deserializeImgTransformation(const proto::common::ImgTransformation& imgTransformation);

DatatypeEnum schemaNameToDatatype(const std::string& schemaName);

// Returns true if deserialization is supported for given datatype, extend for new datatypes
bool deserializationSupported(DatatypeEnum datatype);

inline std::chrono::time_point<std::chrono::steady_clock> fromProtoTimestamp(const dai::proto::common::Timestamp& ts) {
    using namespace std::chrono;
    return time_point<steady_clock>(seconds(ts.sec()) + nanoseconds(ts.nsec()));
}

// Helpers to serialize messages to protobuf
template <typename T>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const T*, bool = false) {
    throw std::runtime_error("getProtoMessage not implemented for this type");
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgAnnotations* message, bool);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const SpatialImgDetections* message, bool metadataOnly);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const IMUData* message, bool);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgDetections* message, bool metadataOnly);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const EncodedFrame* message, bool metadataOnly);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgFrame* message, bool metadataOnly);
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const PointCloudData* message, bool metadataOnly);

// Helpers to deserialize messages from protobuf
template <typename T>
void setProtoMessage(T&, const google::protobuf::Message*, bool = false);
// template <>
// void setProtoMessage(ImgAnnotations& obj, google::protobuf::Message* msg, bool);
// template <>
// void setProtoMessage(SpatialImgDetections& obj, google::protobuf::Message* msg, bool);
// template <>
// void setProtoMessage(ImgDetections& obj, google::protobuf::Message* msg, bool);
template <>
void setProtoMessage(IMUData& obj, const google::protobuf::Message* msg, bool);
template <>
void setProtoMessage(ImgFrame& obj, const google::protobuf::Message* msg, bool metadataOnly);
template <>
void setProtoMessage(EncodedFrame& obj, const google::protobuf::Message* msg, bool metadataOnly);
template <>
void setProtoMessage(PointCloudData& obj, const google::protobuf::Message* msg, bool metadataOnly);

};  // namespace utility
};  // namespace dai
