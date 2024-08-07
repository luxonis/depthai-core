#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/protos/SpatialImgDetections.pb.h"

namespace dai {

/**
 * SpatialImgDetection structure
 *
 * Contains image detection results together with spatial location data.
 */
struct SpatialImgDetection : public ImgDetection {
    Point3f spatialCoordinates;
    SpatialLocationCalculatorConfigData boundingBoxMapping;
};

DEPTHAI_SERIALIZE_EXT(SpatialImgDetection, label, confidence, xmin, ymin, xmax, ymax, spatialCoordinates, boundingBoxMapping);

/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public Buffer, public utility::ProtoSerializable {
   public:
    /**
     * Construct SpatialImgDetections message.
     */
    SpatialImgDetections() = default;
    virtual ~SpatialImgDetections() = default;

    /**
     * Detection results.
     */
    std::vector<SpatialImgDetection> detections;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SpatialImgDetections;
    };

    google::protobuf::Message getProtoMessage() const override {
        //create and populate SpatialImgDetections protobuf message
        proto::SpatialImgDetections spatImgDets;
        spatImgDets.set_sequenceNum(this->sequenceNum);
        
        proto::Timestamp ts;
        ts.set_sec(this->ts.sec);
        ts.set_ns(this->ts.sec);
        spatImgDets->set_ts(ts);
        
        proto::Timestamp tsDevice;
        tsDevice.set_sec(this->tsDevice.sec);
        tsDevice.set_ns(this->tsDevice.nsec);
        spatImgDets->set_tsDevice(tsDevice);

        for(const auto& detection : this->detections) {
            proto::SpatialImgDetection det = spatImgDets.add_detections();

            // populate SpatialImgDetection.ImgDetection from struct inheritance
            proto::ImgDetection detection;
            detection.set_label(detection.label);
            detection.set_confidence(detection.confidence);
            detection.set_xmin(detection.xmin);
            detection.set_ymin(detection.ymin);
            detection.set_xmax(detection.xmax);
            detection.set_ymax(detection.ymax);
            det->set_detection(detection);

            // populate SpatialImgDetection.Point3f
            proto::Point3f spatialCoordinates;
            spatialCoordinates.set_x(detection.spatialCoordinates.x);
            spatialCoordinates.set_y(detection.spatialCoordinates.y);
            spatialCoordinates.set_z(detection.spatialCoordinates.z);
            det->set_spatialCoordinates(spatialCoordinates);

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData
            proto::SpatialLocationCalculatorConfigData boundingBoxMapping;
            boundingBoxMapping.set_AUTO(detection.boundingBoxMapping.AUTO);

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.Rect
            proto::Rect roi;
            roi.set_x(detection.boundingBoxMapping.roi.x);
            roi.set_y(detection.boundingBoxMapping.roi.y);
            roi.set_width(detection.boundingBoxMapping.roi.width);
            roi.set_height(detection.boundingBoxMapping.roi.height);
            boundingBoxMapping.set_roi(roi)

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorAlgorithm
            proto::SpatialLocationCalculatorAlgorithm calculationAlgorithm;
            calculationAlgorithm.set_lowerThreshold(detection.boundingBoxMapping.calculationAlgorithm.lowerThreshold);
            calculationAlgorithm.set_upperThreshold(detection.boundingBoxMapping.calculationAlgorithm.upperThreshold);
            boundingBoxMapping.set_calculationAlgorithm(calculationAlgorithm);

            boundingBoxMapping.set_stepSize(detection.boundingBoxMapping.stepSize);
    
            det->set_boundingBoxMapping(boundingBoxMapping);
        }
        
        return spatImgDets;
    }

    DEPTHAI_SERIALIZE(SpatialImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections);
};

}  // namespace dai
