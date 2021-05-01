#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : NeuralNetwork(par, nodeId) {}

std::string DetectionNetwork::getName() const {
    return "DetectionNetwork";
}

std::vector<Node::Input> DetectionNetwork::getInputs() {
    return {input};
}

std::vector<Node::Output> DetectionNetwork::getOutputs() {
    return {out, passthrough};
}

DetectionNetwork::Properties& DetectionNetwork::getPropertiesRef() {
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

}  // namespace node
}  // namespace dai
