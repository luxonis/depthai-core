#pragma once
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <chrono>
#include <memory>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/schemas/EncodedFrame.pb.h"
#include "depthai/schemas/IMUData.pb.h"
#include "depthai/schemas/ImageAnnotations.pb.h"
#include "depthai/schemas/ImgDetections.pb.h"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/schemas/PointCloudData.pb.h"
#include "depthai/schemas/RGBDData.pb.h"
#include "depthai/schemas/SpatialImgDetections.pb.h"
#include "depthai/schemas/SegmentationMask.pb.h"
#include "depthai/schemas/common.pb.h"
#include "pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/datatype/IMUData.hpp"
#include "pipeline/datatype/PointCloudData.hpp"
#include "pipeline/datatype/RGBDData.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"
#include "utility/ProtoSerializable.hpp"

#ifndef DEPTHAI_PROTO_DECLARE
#define DEPTHAI_PROTO_DECLARE(daiMsg)\
    template <> std::unique_ptr<google::protobuf::Message> getProtoMessage(const daiMsg* message, bool);\
    template <> void setProtoMessage(daiMsg& obj, const google::protobuf::Message* msg, bool);\
    void deserializeProtoMessage(daiMsg& obj, const std::vector<std::uint8_t>& bytes);
#endif

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

template <typename Clock>
inline std::chrono::time_point<Clock> fromProtoTimestamp(const dai::proto::common::Timestamp& ts) {
    using namespace std::chrono;
    using Duration = typename Clock::duration;
    auto total = seconds(ts.sec()) + nanoseconds(ts.nsec());
    auto dur = duration_cast<Duration>(total);
    return time_point<Clock>(dur);
}

template <typename Clock>
std::chrono::time_point<Clock> safeTimestamp(const dai::proto::common::Timestamp& protoTs, bool hasField) {
    using tp = std::chrono::time_point<Clock>;
    return hasField ? fromProtoTimestamp<Clock>(protoTs) : tp{};
};

// Helpers to serialize messages to protobuf
template <typename T>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const T*, bool = false) {
    throw std::runtime_error("getProtoMessage not implemented for this type");
}
// Helpers to deserialize messages from protobuf
template <typename T>
void setProtoMessage(T&, const google::protobuf::Message*, bool = false);

DEPTHAI_PROTO_DECLARE(ImgAnnotations)
DEPTHAI_PROTO_DECLARE(SpatialImgDetections)
DEPTHAI_PROTO_DECLARE(IMUData)
DEPTHAI_PROTO_DECLARE(ImgDetections)
DEPTHAI_PROTO_DECLARE(EncodedFrame)
DEPTHAI_PROTO_DECLARE(ImgFrame)
DEPTHAI_PROTO_DECLARE(SegmentationMask)
DEPTHAI_PROTO_DECLARE(PointCloudData)
DEPTHAI_PROTO_DECLARE(RGBDData)

};  // namespace utility
};  // namespace dai

#undef DEPTHAI_PROTO_DECLARE
