#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId, std::make_unique<Properties>()) {}
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<DetectionNetwork::Properties> props)
    : NeuralNetwork(par, nodeId, std::move(props)), properties(static_cast<Properties&>(*Node::properties)) {
    inputs = {&input};
    outputs = {&out, &passthrough};

    // Default confidence threshold
    properties.confidenceThreshold = 0.5;
}

std::string DetectionNetwork::getName() const {
    return "DetectionNetwork";
}

std::shared_ptr<Node> DetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

DetectionNetwork::Properties& DetectionNetwork::getProperties() {
    return properties;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    properties.confidenceThreshold = thresh;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = DetectionNetworkType::MOBILENET;
}

std::shared_ptr<Node> MobileNetDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = DetectionNetworkType::YOLO;
}

void YoloDetectionNetwork::setNumClasses(const int numClasses) {
    properties.classes = numClasses;
}

void YoloDetectionNetwork::setCoordinateSize(const int coordinates) {
    properties.coordinates = coordinates;
}

void YoloDetectionNetwork::setAnchors(std::vector<float> anchors) {
    properties.anchors = anchors;
}

void YoloDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.anchorMasks = anchorMasks;
}

void YoloDetectionNetwork::setIouThreshold(float thresh) {
    properties.iouThreshold = thresh;
}

std::shared_ptr<Node> YoloDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai
