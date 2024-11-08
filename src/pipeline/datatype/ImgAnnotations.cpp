
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImageAnnotations.pb.h"
    #include "utility/ProtoSerializable.hpp"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {
#ifdef DEPTHAI_ENABLE_PROTOBUF

ProtoSerializable::SchemaPair ImgAnnotations::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgAnnotations::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}

#endif

}  // namespace dai
