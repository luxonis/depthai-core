#include "depthai/pipeline/datatype/ImgDetections.hpp"

#include <vector>

#include "common/Size2f.hpp"
#include "depthai/common/KeypointsList.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

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

dai::VisualizeType ImgDetections::getVisualizationMessage() const {
    auto imgAnnt = std::make_shared<dai::ImgAnnotations>();
    imgAnnt->setTimestamp(this->getTimestamp());

    auto annotation = std::make_shared<dai::ImgAnnotation>();

    for(const auto& detection : this->detections) {
        // Create points annotation for bounding box
        std::cout << "detection label: " << detection.labelName << std::endl;
        dai::RotatedRect boundingBox = detection.getBoundingBox();
        auto pointsAnnotation = std::make_shared<dai::PointsAnnotation>();
        pointsAnnotation->type = dai::PointsAnnotationType::LINE_LOOP;
        auto points = boundingBox.getPoints();
        pointsAnnotation->points.assign(points.begin(), points.end());

        // Set colors and thickness
        pointsAnnotation->outlineColor = dai::Color(float(21.0f / 255.0f), float(127.0f / 255.0f), float(88.0f / 255.0f), float(1.0f));
        pointsAnnotation->fillColor = dai::Color(float(21.0f / 255.0f), float(127.0f / 255.0f), float(88.0f / 255.0f), float(0.2f));
        pointsAnnotation->thickness = 1.0f;

        // Create text annotation
        auto text = std::make_shared<dai::TextAnnotation>();
        std::tuple<dai::Point2f, dai::Size2f> outerPoints = boundingBox.getOuterXYWH();
        text->position = std::get<0>(outerPoints);
        text->text = detection.labelName + " (" + std::to_string(int(detection.confidence * 100)) + "adddddd)";
        text->fontSize = 50.5f;
        text->textColor = dai::Color(0.5f, 0.5f, 1.0f, 1.0f);
        text->backgroundColor = dai::Color(1.0f, 1.0f, 0.5f, 1.0f);

        annotation->points.push_back(*pointsAnnotation);
        annotation->texts.push_back(*text);

        // Draw keypoints if available
        auto keypoints = detection.getKeypoints();
        if(!keypoints.empty()) {
            auto keypointsAnnotation = std::make_shared<dai::PointsAnnotation>();
            keypointsAnnotation->type = dai::PointsAnnotationType::POINTS;
            keypointsAnnotation->outlineColor = dai::Color(float(21.0f / 255.0f), float(127.0f / 255.0f), float(88.0f / 255.0f), float(1.0f));
            keypointsAnnotation->thickness = 3.0f;
            std::vector<Point2f> kpPoints;
            kpPoints.reserve(keypoints.size());
            for(Keypoint& kp : keypoints) {
                kpPoints.push_back(Point2f{kp.coordinates.x, kp.coordinates.y});
            }
            keypointsAnnotation->points = kpPoints;
            annotation->points.push_back(*keypointsAnnotation);
        }
    }

    imgAnnt->annotations.push_back(*annotation);
    return imgAnnt;
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
