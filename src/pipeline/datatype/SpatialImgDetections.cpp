#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
    #include "depthai/schemas/SpatialImgDetections.pb.h"
#endif

namespace dai {
#ifdef DEPTHAI_ENABLE_PROTOBUF
std::vector<std::uint8_t> SpatialImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}

ProtoSerializable::SchemaPair SpatialImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}
#endif
}  // namespace dai
