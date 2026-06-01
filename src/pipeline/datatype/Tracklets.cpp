#include "depthai/pipeline/datatype/Tracklets.hpp"

#include <memory>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {

Tracklets::~Tracklets() = default;

SpatialImgDetection createSpatialDetection(const ImgDetection& detection, const Point3f spatialCoordinates) {
    // local function because it also sets spatial coordinates to keypoints.
    SpatialImgDetection spatialDetection(detection.getBoundingBox(), spatialCoordinates, detection.labelName, detection.confidence, detection.label);

    std::vector<Keypoint> keypoints = detection.getKeypoints();
    if(!keypoints.empty()) {
        std::vector<SpatialKeypoint> spatialKeypoints;
        spatialKeypoints.reserve(keypoints.size());
        for(const auto& keypoint : keypoints) {
            spatialKeypoints.emplace_back(keypoint.imageCoordinates, spatialCoordinates, keypoint.confidence, keypoint.label, keypoint.labelName);
        }
        spatialDetection.setKeypoints(std::move(spatialKeypoints), detection.getEdges());
    }

    return spatialDetection;
}

void Tracklet::transform(const ImgTransformation& source, const ImgTransformation& target, LengthUnit lengthUnit) {
    float depth = spatialCoordinates.z * getDistanceUnitScale(LengthUnit::MILLIMETER, lengthUnit);
    if(velocity.has_value()) {
        const auto rotation = source.getRotationMatrixTo(target);
        const auto rotatedVelocity = matrix::matVecMul(rotation, {velocity->x, velocity->y, velocity->z});
        velocity = Point3f(rotatedVelocity[0], rotatedVelocity[1], rotatedVelocity[2]);
    }

    SpatialImgDetection spatialImgDetection = createSpatialDetection(srcImgDetection, spatialCoordinates);
    if(spatialImgDetection.keypoints.has_value()) {
        spatialImgDetection.keypoints->unit = lengthUnit;
    }
    spatialImgDetection.transform(source, target, lengthUnit);
    srcImgDetection = spatialImgDetection.getImgDetection();
    spatialCoordinates = spatialImgDetection.spatialCoordinates;

    auto rotatedRoi = dai::RotatedRect(roi);
    bool hasNormalized = roi.hasNormalized;
    bool isNormalized = roi.normalized;
    if(depth > 0) {
        roi = source.projectRectTo(target, rotatedRoi, depth);
    } else {
        roi = source.remapRectTo(target, rotatedRoi);
    }
    roi.hasNormalized = hasNormalized;
    roi.normalized = isNormalized;
}

void Tracklets::transformToInternal(const ImgTransformation& target) {
    if(!this->getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform tracklets.");
    }
    for(auto& tracklet : tracklets) {
        tracklet.transform(getTransformation().value(), target, unit);
    }
    setTransformation(target);
}

Tracklets Tracklets::transformTo(const ImgTransformation& target) {
    return TransformableCRTP<Tracklets>::transformTo(target);
}

void Tracklets::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::Tracklets;
}

}  // namespace dai
