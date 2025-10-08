#include "depthai/pipeline/datatype/IMUData.hpp"

#if DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/IMUData.pb.h"
    #include "depthai/schemas/common.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

IMUData::~IMUData() = default;

void IMUData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::IMUData;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF

ProtoSerializable::SchemaPair IMUData::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> IMUData::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
