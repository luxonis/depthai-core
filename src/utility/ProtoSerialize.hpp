#pragma once
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <memory>

#include "utility/ProtoSerializable.hpp"
namespace dai {
namespace utility {

std::vector<std::uint8_t> serializeProto(std::unique_ptr<google::protobuf::Message> protoMessage);
utility::ProtoSerializable::SchemaPair serializeSchema(std::unique_ptr<google::protobuf::Message> protoMessage);

};  // namespace utility
};  // namespace dai