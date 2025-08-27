#include "depthai/pipeline/datatype/ImgDetections.hpp"

#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/RotatedRect.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/ImgDetections.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

// ImgDetection functions

ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, float conf = 0.f, uint32_t label = 0) {
    setBoundingBox(boundingBox);
    this->confidence = conf;
    this->label = label;
}
ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, std::string labelName, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->labelName = std::move(labelName);
}

ImgDetection::ImgDetection(const dai::RotatedRect& boundingBox, const dai::KeypointsList& keypoints, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->keypoints = keypoints;
}

ImgDetection::ImgDetection(
    const dai::RotatedRect& boundingBox, const dai::KeypointsList& keypoints, std::string labelName, float conf = 0.f, uint32_t label = 0)
    : ImgDetection(boundingBox, conf, label) {
    this->keypoints = keypoints;
    this->labelName = std::move(labelName);
}

void ImgDetection::setBoundingBox(const RotatedRect boundingBox) {
    this->boundingBox = boundingBox;
}

RotatedRect ImgDetection::getBoundingBox() const {
    if(boundingBox.has_value()) {
        return boundingBox.value();
    } else if(xmin == 0.f && xmax == 0.f && ymin == 0.f && ymax == 0.f) {
        throw std::runtime_error("All bounding box values are zero, no bounding box can be built.");
    }

    // construct from legacy values
    RotatedRect rect = RotatedRect{dai::Rect{Point2f{xmin, ymin}, Point2f{xmax, ymax}}};
    return rect;
}

void ImgDetection::setKeypoints(const KeypointsList kp) {
    this->keypoints = kp;
}

void ImgDetection::setKeypoints(const std::vector<Keypoint> kp) {
    this->keypoints = KeypointsList(kp);
}

void ImgDetection::setKeypoints(const std::vector<Keypoint> kps, const std::vector<std::array<uint32_t, 2>> edges) {
    this->keypoints = KeypointsList(kps, edges);
}

void ImgDetection::setKeypoints(const std::vector<Point3f> kps3) {
    this->keypoints = KeypointsList();
    this->keypoints->setKeypoints(kps3);
}

void ImgDetection::setKeypoints(const std::vector<Point2f> kps2) {
    this->keypoints = KeypointsList();
    this->keypoints->setKeypoints(kps2);
}

std::vector<Keypoint> ImgDetection::getKeypoints() const {
    if(keypoints.has_value()) {
        return keypoints->getKeypoints();
    } else {
        return {};
    }
}

void ImgDetection::setEdges(const std::vector<Edge> edges) {
    if(!keypoints.has_value()) {
        throw std::runtime_error("No keypoints set, cannot set edges.");
    }
    keypoints->setEdges(edges);
}

std::vector<Edge> ImgDetection::getEdges() const {
    if(keypoints.has_value()) {
        return keypoints->getEdges();
    } else {
        return {};
    }
}

float ImgDetection::getCenterX() const noexcept {
    return getBoundingBox().center.x;
}

float ImgDetection::getCenterY() const noexcept {
    return getBoundingBox().center.y;
}

float ImgDetection::getWidth() const noexcept {
    return getBoundingBox().size.width;
}

float ImgDetection::getHeight() const noexcept {
    return getBoundingBox().size.height;
}

float ImgDetection::getAngle() const noexcept {
    return getBoundingBox().angle;
}

// ImgDetections functions

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair ImgDetections::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> ImgDetections::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai
