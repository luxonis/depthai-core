#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/schemas/SpatialImgDetections.pb.h"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/Serialization.hpp"

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

    std::unique_ptr<google::protobuf::Message> getProtoMessage() const override {
        // create and populate SpatialImgDetections protobuf message
        auto spatialImgDetections = std::make_unique<proto::SpatialImgDetections>();
        spatialImgDetections->set_sequencenum(this->sequenceNum);

        proto::Timestamp* ts = spatialImgDetections->mutable_ts();
        ts->set_sec(this->ts.sec);
        ts->set_nsec(this->ts.nsec);

        proto::Timestamp* tsDevice = spatialImgDetections->mutable_tsdevice();
        tsDevice->set_sec(this->tsDevice.sec);
        tsDevice->set_nsec(this->tsDevice.nsec);

        for(const auto& detection : this->detections) {
            proto::SpatialImgDetection* spatialImgDetection = spatialImgDetections->add_detections();

            // populate SpatialImgDetection.ImgDetection from struct inheritance
            proto::ImgDetection* imgDetection = spatialImgDetection->mutable_detection();
            imgDetection->set_label(detection.label);
            imgDetection->set_confidence(detection.confidence);
            imgDetection->set_xmin(detection.xmin);
            imgDetection->set_ymin(detection.ymin);
            imgDetection->set_xmax(detection.xmax);
            imgDetection->set_ymax(detection.ymax);

            // populate SpatialImgDetection.Point3f
            proto::Point3f* spatialCoordinates = spatialImgDetection->mutable_spatialcoordinates();
            spatialCoordinates->set_x(detection.spatialCoordinates.x);
            spatialCoordinates->set_y(detection.spatialCoordinates.y);
            spatialCoordinates->set_z(detection.spatialCoordinates.z);

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData
            proto::SpatialLocationCalculatorConfigData* boundingBoxMapping = spatialImgDetection->mutable_boundingboxmapping();

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.Rect
            proto::Rect* roi = boundingBoxMapping->mutable_roi();
            roi->set_x(detection.boundingBoxMapping.roi.x);
            roi->set_y(detection.boundingBoxMapping.roi.y);
            roi->set_width(detection.boundingBoxMapping.roi.width);
            roi->set_height(detection.boundingBoxMapping.roi.height);

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorConfigThresholds
            proto::SpatialLocationCalculatorConfigThresholds* depthTresholds = boundingBoxMapping->mutable_depththresholds();
            depthTresholds->set_lowerthreshold(detection.boundingBoxMapping.depthThresholds.lowerThreshold);
            depthTresholds->set_upperthreshold(detection.boundingBoxMapping.depthThresholds.upperThreshold);

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorAlgorithm
            boundingBoxMapping->set_calculationalgorithm(
                static_cast<proto::SpatialLocationCalculatorAlgorithm>(detection.boundingBoxMapping.calculationAlgorithm));

            // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.stepSize
            boundingBoxMapping->set_stepsize(detection.boundingBoxMapping.stepSize);
        }

        return spatialImgDetections;
    }

    DEPTHAI_SERIALIZE(SpatialImgDetections, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, detections);
};

}  // namespace dai
