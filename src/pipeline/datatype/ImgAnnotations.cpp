
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerializable.hpp"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

ImgAnnotations::~ImgAnnotations() = default;

void ImgAnnotations::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImgAnnotations;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF

ProtoSerializable::SchemaPair ImgAnnotations::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgAnnotations::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}

#endif

}  // namespace dai
