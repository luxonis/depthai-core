#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : NeuralNetwork(par, nodeId) {
    // Default confidence threshold
    getPropertiesRef().confidenceThreshold = 0.5;
}

std::string DetectionNetwork::getName() const {
    return "DetectionNetwork";
}

DetectionNetwork::Properties& DetectionNetwork::getPropertiesRef() {
    return properties;
}
const DetectionNetwork::Properties& DetectionNetwork::getPropertiesRef() const {
    return properties;
}

std::shared_ptr<Node> DetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

nlohmann::json DetectionNetwork::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    getPropertiesRef().confidenceThreshold = thresh;
}

float DetectionNetwork::getConfidenceThreshold() const {
    return getPropertiesRef().confidenceThreshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    getPropertiesRef().nnFamily = DetectionNetworkType::MOBILENET;
}

std::shared_ptr<Node> MobileNetDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    getPropertiesRef().nnFamily = DetectionNetworkType::YOLO;
}

void YoloDetectionNetwork::setNumClasses(const int numClasses) {
    getPropertiesRef().classes = numClasses;
}

void YoloDetectionNetwork::setCoordinateSize(const int coordinates) {
    getPropertiesRef().coordinates = coordinates;
}

void YoloDetectionNetwork::setAnchors(std::vector<float> anchors) {
    getPropertiesRef().anchors = anchors;
}

void YoloDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    getPropertiesRef().anchorMasks = anchorMasks;
}

void YoloDetectionNetwork::setIouThreshold(float thresh) {
    getPropertiesRef().iouThreshold = thresh;
}

std::shared_ptr<Node> YoloDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

/// Get num classes
int YoloDetectionNetwork::getNumClasses() const {
    return getPropertiesRef().classes;
}

/// Get coordianate size
int YoloDetectionNetwork::getCoordinateSize() const {
    return getPropertiesRef().coordinates;
}

/// Get anchors
std::vector<float> YoloDetectionNetwork::getAnchors() const {
    return getPropertiesRef().anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetwork::getAnchorMasks() const {
    return getPropertiesRef().anchorMasks;
}

/// Get Iou threshold
float YoloDetectionNetwork::getIouThreshold() const {
    return getPropertiesRef().iouThreshold;
}

}  // namespace node
}  // namespace dai
