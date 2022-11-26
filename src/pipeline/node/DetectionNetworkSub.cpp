#include "depthai/pipeline/node/DetectionNetworkSub.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

void DetectionNetworkSub::build() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
    properties = neuralNetwork->properties;
}

void DetectionNetworkSub::setConfidenceThreshold(float thresh) {
    detectionParser->properties.parser.confidenceThreshold = thresh;
}

float DetectionNetworkSub::getConfidenceThreshold() const {
    return detectionParser->properties.parser.confidenceThreshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetDetectionNetworkSub::build() {
    DetectionNetworkSub::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloDetectionNetworkSub::build() {
    DetectionNetworkSub::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    detectionParser->properties.parser.iouThreshold = 0.5f;
}

void YoloDetectionNetworkSub::setNumClasses(const int numClasses) {
    detectionParser->properties.parser.classes = numClasses;
}

void YoloDetectionNetworkSub::setCoordinateSize(const int coordinates) {
    detectionParser->properties.parser.coordinates = coordinates;
}

void YoloDetectionNetworkSub::setAnchors(std::vector<float> anchors) {
    detectionParser->properties.parser.anchors = anchors;
}

void YoloDetectionNetworkSub::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    detectionParser->properties.parser.anchorMasks = anchorMasks;
}

void YoloDetectionNetworkSub::setIouThreshold(float thresh) {
    detectionParser->properties.parser.iouThreshold = thresh;
}

/// Get num classes
int YoloDetectionNetworkSub::getNumClasses() const {
    return detectionParser->properties.parser.classes;
}

/// Get coordianate size
int YoloDetectionNetworkSub::getCoordinateSize() const {
    return detectionParser->properties.parser.coordinates;
}

/// Get anchors
std::vector<float> YoloDetectionNetworkSub::getAnchors() const {
    return detectionParser->properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetworkSub::getAnchorMasks() const {
    return detectionParser->properties.parser.anchorMasks;
}

/// Get Iou threshold
float YoloDetectionNetworkSub::getIouThreshold() const {
    return detectionParser->properties.parser.iouThreshold;
}

}  // namespace node
}  // namespace dai
