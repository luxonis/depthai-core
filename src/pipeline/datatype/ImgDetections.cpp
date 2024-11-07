#include "depthai/pipeline/datatype/ImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImgDetections.pb.h"
    #include "../../utility/ProtoSerialize.hpp"
#endif

namespace dai {

#ifdef DEPTHAI_ENABLE_PROTOBUF
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgDetections* detections) {
    auto imgDetections = std::make_unique<proto::img_detections::ImgDetections>();

    imgDetections->set_sequencenum(detections->sequenceNum);
    proto::common::Timestamp* ts = imgDetections->mutable_ts();
    ts->set_sec(detections->ts.sec);
    ts->set_nsec(detections->ts.nsec);
    proto::common::Timestamp* tsDevice = imgDetections->mutable_tsdevice();
    tsDevice->set_sec(detections->tsDevice.sec);
    tsDevice->set_nsec(detections->tsDevice.nsec);

    for(const auto& detection : detections->detections) {
        proto::img_detections::ImgDetection* imgDetection = imgDetections->add_detections();
        imgDetection->set_label(detection.label);
        imgDetection->set_confidence(detection.confidence);
        imgDetection->set_xmin(detection.xmin);
        imgDetection->set_ymin(detection.ymin);
        imgDetection->set_xmax(detection.xmax);
        imgDetection->set_ymax(detection.ymax);
    }
    return imgDetections;
}

utility::ProtoSerializable::SchemaPair ImgDetections::serializeSchema() const {
    return utility::serializeSchema(getProtoMessage(this));
}


std::vector<std::uint8_t> ImgDetections::serializeProto() const {
    return utility::serializeProto(getProtoMessage(this));
}
#endif

}  // namespace dai
