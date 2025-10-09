#include "depthai/pipeline/datatype/ImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

void ImgDetections::addDetection(const ImgDetection& detection) {
    detections.push_back(detection);
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair ImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
