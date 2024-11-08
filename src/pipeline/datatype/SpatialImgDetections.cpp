#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "../../utility/ProtoSerialize.hpp"
    #include "depthai/schemas/SpatialImgDetections.pb.h"
#endif

namespace dai {
#ifdef DEPTHAI_ENABLE_PROTOBUF
std::unique_ptr<google::protobuf::Message> getProtoMessage(const dai::SpatialImgDetections* daiSpatialImgDetections) {
    // create and populate SpatialImgDetections protobuf message
    auto spatialImgDetections = std::make_unique<proto::spatial_img_detections::SpatialImgDetections>();
    spatialImgDetections->set_sequencenum(daiSpatialImgDetections->sequenceNum);

    proto::common::Timestamp* ts = spatialImgDetections->mutable_ts();
    ts->set_sec(daiSpatialImgDetections->ts.sec);
    ts->set_nsec(daiSpatialImgDetections->ts.nsec);

    proto::common::Timestamp* tsDevice = spatialImgDetections->mutable_tsdevice();
    tsDevice->set_sec(daiSpatialImgDetections->tsDevice.sec);
    tsDevice->set_nsec(daiSpatialImgDetections->tsDevice.nsec);

    for(const auto& detection : daiSpatialImgDetections->detections) {
        proto::spatial_img_detections::SpatialImgDetection* spatialImgDetection = spatialImgDetections->add_detections();

        // populate SpatialImgDetection.ImgDetection from struct inheritance
        proto::img_detections::ImgDetection* imgDetection = spatialImgDetection->mutable_detection();
        imgDetection->set_label(detection.label);
        imgDetection->set_confidence(detection.confidence);
        imgDetection->set_xmin(detection.xmin);
        imgDetection->set_ymin(detection.ymin);
        imgDetection->set_xmax(detection.xmax);
        imgDetection->set_ymax(detection.ymax);

        // populate SpatialImgDetection.Point3f
        proto::spatial_img_detections::Point3f* spatialCoordinates = spatialImgDetection->mutable_spatialcoordinates();
        spatialCoordinates->set_x(detection.spatialCoordinates.x);
        spatialCoordinates->set_y(detection.spatialCoordinates.y);
        spatialCoordinates->set_z(detection.spatialCoordinates.z);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData
        proto::spatial_img_detections::SpatialLocationCalculatorConfigData* boundingBoxMapping = spatialImgDetection->mutable_boundingboxmapping();

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.Rect
        proto::spatial_img_detections::Rect* roi = boundingBoxMapping->mutable_roi();
        roi->set_x(detection.boundingBoxMapping.roi.x);
        roi->set_y(detection.boundingBoxMapping.roi.y);
        roi->set_width(detection.boundingBoxMapping.roi.width);
        roi->set_height(detection.boundingBoxMapping.roi.height);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorConfigThresholds
        proto::spatial_img_detections::SpatialLocationCalculatorConfigThresholds* depthTresholds = boundingBoxMapping->mutable_depththresholds();
        depthTresholds->set_lowerthreshold(detection.boundingBoxMapping.depthThresholds.lowerThreshold);
        depthTresholds->set_upperthreshold(detection.boundingBoxMapping.depthThresholds.upperThreshold);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorAlgorithm
        boundingBoxMapping->set_calculationalgorithm(
            static_cast<proto::spatial_img_detections::SpatialLocationCalculatorAlgorithm>(detection.boundingBoxMapping.calculationAlgorithm));

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.stepSize
        boundingBoxMapping->set_stepsize(detection.boundingBoxMapping.stepSize);
    }
    proto::common::ImgTransformation* imgTransformation = spatialImgDetections->mutable_transformation();
    if(daiSpatialImgDetections->transformation.has_value()) {
        utility::serializeImgTransormation(imgTransformation, daiSpatialImgDetections->transformation.value());
    }

    return spatialImgDetections;
}

std::vector<std::uint8_t> SpatialImgDetections::serializeProto() const {
    return utility::serializeProto(getProtoMessage(this));
}

ProtoSerializable::SchemaPair SpatialImgDetections::serializeSchema() const {
    return utility::serializeSchema(getProtoMessage(this));
}
#endif
}  // namespace dai
