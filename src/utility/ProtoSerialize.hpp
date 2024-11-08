#pragma once
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <memory>

#include "utility/ProtoSerializable.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/schemas/common.pb.h"
namespace dai {
namespace utility {

std::vector<std::uint8_t> serializeProto(std::unique_ptr<google::protobuf::Message> protoMessage);
ProtoSerializable::SchemaPair serializeSchema(std::unique_ptr<google::protobuf::Message> protoMessage);

// Common functions for serializing
void serializeImgTransformation(proto::common::ImgTransformation* imgTransformation, const ImgTransformation& transformation);


};  // namespace utility
};  // namespace dai