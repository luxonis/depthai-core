#include "depthai/beta/datatype/Keypoints.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

namespace dai {
namespace beta {

namespace {

// Visualization constants matching the source implementation in depthai-nodes.
const Color KEYPOINT_COLOR{0.0f, 1.0f, 1.0f, 1.0f};
const Color PRIMARY_COLOR{21.0f / 255.0f, 127.0f / 255.0f, 88.0f / 255.0f, 1.0f};
constexpr float KEYPOINT_THICKNESS = 1.0f;
constexpr float EDGE_THICKNESS = 1.0f;

}  // namespace

Keypoints::~Keypoints() = default;

void Keypoints::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

std::vector<Keypoint> Keypoints::getKeypoints() const {
    return keypointsList.getKeypoints();
}

void Keypoints::setKeypoints(const std::vector<Keypoint>& keypoints) {
    keypointsList.setKeypoints(keypoints);
}

void Keypoints::setKeypoints(const std::vector<Keypoint>& keypoints, const std::vector<Edge>& edges) {
    keypointsList.setKeypoints(keypoints, edges);
}

std::vector<Edge> Keypoints::getEdges() const {
    return keypointsList.getEdges();
}

void Keypoints::setEdges(const std::vector<Edge>& edges) {
    keypointsList.setEdges(edges);
}

std::vector<Point2f> Keypoints::getPoints2f() const {
    return keypointsList.getPoints2f();
}

std::vector<Point3f> Keypoints::getPoints3f() const {
    return keypointsList.getPoints3f();
}

void Keypoints::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform keypoints.");
    }
    keypointsList = keypointsList.transformTo(*getTransformation(), target);
    setTransformation(target);
}

Keypoints Keypoints::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Keypoints>::transformTo(target);
}

dai::VisualizeType Keypoints::getVisualizationMessage() const {
    ImgAnnotation annotation;

    // Keypoints are drawn as a single POINTS annotation.
    PointsAnnotation pointsAnnotation;
    pointsAnnotation.type = PointsAnnotationType::POINTS;
    pointsAnnotation.points = keypointsList.getPoints2f();
    pointsAnnotation.outlineColor = KEYPOINT_COLOR;
    pointsAnnotation.thickness = KEYPOINT_THICKNESS;
    annotation.points.push_back(std::move(pointsAnnotation));

    // Each skeleton edge is drawn as a two-point line strip.
    const auto keypoints = keypointsList.getKeypoints();
    for(const auto& edge : keypointsList.getEdges()) {
        const auto& pt1 = keypoints[edge[0]].imageCoordinates;
        const auto& pt2 = keypoints[edge[1]].imageCoordinates;
        PointsAnnotation line;
        line.type = PointsAnnotationType::LINE_STRIP;
        line.points = {Point2f(pt1.x, pt1.y, true), Point2f(pt2.x, pt2.y, true)};
        line.outlineColor = PRIMARY_COLOR;
        line.fillColor = PRIMARY_COLOR;
        line.thickness = EDGE_THICKNESS;
        annotation.points.push_back(std::move(line));
    }

    auto annotations = std::make_shared<ImgAnnotations>();
    annotations->annotations.push_back(std::move(annotation));
    annotations->setTimestamp(getTimestamp());
    annotations->setSequenceNum(getSequenceNum());
    return annotations;
}

}  // namespace beta
}  // namespace dai
