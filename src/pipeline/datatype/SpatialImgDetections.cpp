#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/SpatialImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

SpatialImgDetections::~SpatialImgDetections() = default;

void SpatialImgDetections::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SpatialImgDetections;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
std::vector<std::uint8_t> SpatialImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}

ProtoSerializable::SchemaPair SpatialImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}
#endif
}  // namespace dai
