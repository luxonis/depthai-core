#include "depthai/pipeline/datatype/ImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
    #include "depthai/schemas/ImgDetections.pb.h"
#endif

namespace dai {

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair ImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
