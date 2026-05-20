#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/common/SpatialKeypoint.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/matrixOps.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/SpatialImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

// SpatialImgDetection functions

SpatialImgDetection::SpatialImgDetection(const dai::RotatedRect& boundingBox, Point3f spatialCoordinates, float conf, uint32_t label) {
    setBoundingBox(boundingBox);
    this->spatialCoordinates = spatialCoordinates;
    this->confidence = conf;
    this->label = label;
}
SpatialImgDetection::SpatialImgDetection(const dai::RotatedRect& boundingBox, Point3f spatialCoordinates, std::string labelName, float conf, uint32_t label)
    : SpatialImgDetection(boundingBox, spatialCoordinates, conf, label) {
    this->labelName = std::move(labelName);
}

SpatialImgDetection::SpatialImgDetection(
    const dai::RotatedRect& boundingBox, Point3f spatialCoordinates, const dai::SpatialKeypointsList& keypoints, float conf, uint32_t label)
    : SpatialImgDetection(boundingBox, spatialCoordinates, conf, label) {
    this->keypoints = keypoints;
}

SpatialImgDetection::SpatialImgDetection(const dai::RotatedRect& boundingBox,
                                         Point3f spatialCoordinates,
                                         const dai::SpatialKeypointsList& keypoints,
                                         std::string labelName,
                                         float conf,
                                         uint32_t label)
    : SpatialImgDetection(boundingBox, spatialCoordinates, conf, label) {
    this->keypoints = keypoints;
    this->labelName = std::move(labelName);
}

void SpatialImgDetection::setBoundingBox(const RotatedRect boundingBox) {
    this->boundingBox = boundingBox;

    std::array<float, 4> outerPoints = this->boundingBox->getOuterRect();
    xmin = outerPoints[0];
    ymin = outerPoints[1];
    xmax = outerPoints[2];
    ymax = outerPoints[3];
}

RotatedRect SpatialImgDetection::getBoundingBox() const {
    if(boundingBox.has_value()) {
        return boundingBox.value();
    } else if(xmin == 0.f && xmax == 0.f && ymin == 0.f && ymax == 0.f) {
        throw std::runtime_error("All bounding box values are zero, no bounding box can be built.");
    }

    // construct from legacy values
    RotatedRect rect = RotatedRect{dai::Rect{Point2f{xmin, ymin}, Point2f{xmax, ymax}}};
    return rect;
}

void SpatialImgDetection::setOuterBoundingBox(const float xmin, const float ymin, const float xmax, const float ymax) {
    this->xmin = xmin;
    this->ymin = ymin;
    this->xmax = xmax;
    this->ymax = ymax;

    Point2f topLeft{xmin, ymin};
    Point2f bottomRight{xmax, ymax};

    this->boundingBox = RotatedRect{Rect{topLeft, bottomRight}};
}

void SpatialImgDetection::setKeypoints(const SpatialKeypointsList kp) {
    this->keypoints = kp;
}

void SpatialImgDetection::setKeypoints(const std::vector<SpatialKeypoint> kp) {
    this->keypoints = SpatialKeypointsList(kp);
}

void SpatialImgDetection::setKeypoints(const std::vector<SpatialKeypoint> kps, const std::vector<std::array<uint32_t, 2>> edges) {
    this->keypoints = SpatialKeypointsList(kps, edges);
}

void SpatialImgDetection::setKeypoints(const std::vector<Point3f> kps3) {
    this->keypoints = SpatialKeypointsList();
    this->keypoints->setKeypoints(kps3);
}

void SpatialImgDetection::setSpatialCoordinate(Point3f spatialCoordinates) {
    this->spatialCoordinates = spatialCoordinates;
}

dai::ImgDetection SpatialImgDetection::getImgDetection() const {
    dai::ImgDetection imgDetection(getBoundingBox(), labelName, confidence, label);

    if(keypoints.has_value()) {
        dai::KeypointsList converted;
        std::vector<dai::Keypoint> convertedKeypoints;
        convertedKeypoints.reserve(keypoints->size());
        for(const auto& spatialKeypoint : keypoints->getKeypoints()) {
            convertedKeypoints.emplace_back(spatialKeypoint.imageCoordinates, spatialKeypoint.confidence, spatialKeypoint.label, spatialKeypoint.labelName);
        }
        converted.setKeypoints(std::move(convertedKeypoints), keypoints->getEdges());
        imgDetection.keypoints = std::move(converted);
    }

    return imgDetection;
}

std::vector<SpatialKeypoint> SpatialImgDetection::getKeypoints() const {
    if(keypoints.has_value()) {
        return keypoints->getKeypoints();
    } else {
        return {};
    }
}

std::vector<dai::Point3f> SpatialImgDetection::getKeypointSpatialCoordinates() const {
    if(keypoints.has_value()) {
        return keypoints->getSpatialCoordinates();
    } else {
        return {};
    }
}

std::vector<Edge> SpatialImgDetection::getEdges() const {
    if(keypoints.has_value()) {
        return keypoints->getEdges();
    } else {
        return {};
    }
}

void SpatialImgDetection::setEdges(const std::vector<Edge> edges) {
    if(!keypoints.has_value()) {
        throw std::runtime_error("No keypoints set, cannot set edges.");
    }
    keypoints->setEdges(edges);
}

float SpatialImgDetection::getCenterX() const {
    return getBoundingBox().center.x;
}

float SpatialImgDetection::getCenterY() const {
    return getBoundingBox().center.y;
}

float SpatialImgDetection::getWidth() const {
    return getBoundingBox().size.width;
}

float SpatialImgDetection::getHeight() const {
    return getBoundingBox().size.height;
}

float SpatialImgDetection::getAngle() const {
    return getBoundingBox().angle;
}

SpatialImgDetections::~SpatialImgDetections() = default;

void SpatialImgDetections::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SpatialImgDetections;
}

void SpatialImgDetection::transformKeypointsFallback(const ImgTransformation& source, const ImgTransformation& target, const Point3f& spatialCoordinates) {
    SpatialKeypointsList kpCopy = keypoints.value();
    std::vector<Point3f> originalCoordinates = kpCopy.getSpatialCoordinates();
    std::vector<size_t> changedIndices;
    changedIndices.reserve(kpCopy.keypoints.size());

    for(size_t i = 0; i < kpCopy.keypoints.size(); ++i) {
        if(kpCopy.keypoints[i].spatialCoordinates.z <= 0) {
            kpCopy.keypoints[i].spatialCoordinates = spatialCoordinates;
            changedIndices.push_back(i);
        }
    }

    auto kpTransformed = kpCopy.transformTo(source, target);

    for(size_t i : changedIndices) {
        kpTransformed.keypoints[i].spatialCoordinates = originalCoordinates[i];
    }
    keypoints = kpTransformed;
}

void SpatialImgDetection::transform(const ImgTransformation& source, const ImgTransformation& target, LengthUnit lengthUnit) {
    Point3f spatialCoordinates = this->spatialCoordinates;
    float depth = spatialCoordinates.z * getDistanceUnitScale(LengthUnit::MILLIMETER, lengthUnit);
    RotatedRect rect = getBoundingBox();

    if(depth > 0) {
        const auto transMatrix = source.getExtrinsicsTransformationMatrixTo(target, false, lengthUnit);
        setBoundingBox(source.projectRectTo(target, rect, depth));
        setSpatialCoordinate(matrix::transformPoint3f(transMatrix, spatialCoordinates));
    } else {
        setBoundingBox(source.remapRectTo(target, rect));
    }

    if(keypoints.has_value()) {
        float s = getDistanceUnitScale(keypoints->unit, lengthUnit);
        dai::Point3f scaledFallbackCoordinates{spatialCoordinates.x * s, spatialCoordinates.y * s, spatialCoordinates.z * s};
        transformKeypointsFallback(source, target, scaledFallbackCoordinates);
    }
}

void SpatialImgDetections::transformToInternal(const ImgTransformation& target) {
    if(!this->getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform spatial image detections.");
    }
    ImgTransformation source = *this->getTransformation();

    for(auto& detection : detections) {
        detection.transform(source, target, unit);
    }
    setTransformation(target);
}

SpatialImgDetections SpatialImgDetections::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<SpatialImgDetections>::transformTo(target);
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
