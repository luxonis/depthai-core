#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

void DetectionNetwork::build() {
    // Default confidence threshold
    properties.parser.confidenceThreshold = 0.5;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    properties.parser.confidenceThreshold = thresh;
}

float DetectionNetwork::getConfidenceThreshold() const {
    return properties.parser.confidenceThreshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetDetectionNetwork::build() {
    properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloDetectionNetwork::build() {
    properties.parser.nnFamily = DetectionNetworkType::YOLO;
    properties.parser.iouThreshold = 0.5f;
}

void YoloDetectionNetwork::setNumClasses(const int numClasses) {
    properties.parser.classes = numClasses;
}

void YoloDetectionNetwork::setCoordinateSize(const int coordinates) {
    properties.parser.coordinates = coordinates;
}

void YoloDetectionNetwork::setAnchors(std::vector<float> anchors) {
    properties.parser.anchors = anchors;
}

void YoloDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.parser.anchorMasks = anchorMasks;
}

void YoloDetectionNetwork::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

/// Get num classes
int YoloDetectionNetwork::getNumClasses() const {
    return properties.parser.classes;
}

/// Get coordianate size
int YoloDetectionNetwork::getCoordinateSize() const {
    return properties.parser.coordinates;
}

/// Get anchors
std::vector<float> YoloDetectionNetwork::getAnchors() const {
    return properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetwork::getAnchorMasks() const {
    return properties.parser.anchorMasks;
}

/// Get Iou threshold
float YoloDetectionNetwork::getIouThreshold() const {
    return properties.parser.iouThreshold;
}

}  // namespace node
}  // namespace dai
