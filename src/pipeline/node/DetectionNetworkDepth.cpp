#include "depthai/pipeline/node/DetectionNetworkDepth.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
DetectionNetworkDepth::DetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {}

std::string DetectionNetworkDepth::getName() const {
    return "DetectionNetworkDepth";
}

std::vector<Node::Input> DetectionNetworkDepth::getInputs() {
    return {input, depthInput};
}

std::vector<Node::Output> DetectionNetworkDepth::getOutputs() {
    return {out, passthroughRoi};
}

dai::NeuralNetworkProperties& DetectionNetworkDepth::getPropertiesRef() {
    return properties;
}

nlohmann::json DetectionNetworkDepth::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

void DetectionNetworkDepth::setConfidenceThreshold(float thresh) {
    properties.confidenceThreshold = thresh;
}

void DetectionNetworkDepth::setBoundingBoxScaleFactor(float factor) {
    properties.detectedBBScaleFactor = factor;
}

void DetectionNetworkDepth::setDepthLowerThresholdLimit(uint32_t lower_threshold) {
    properties.lower_threshold = lower_threshold;
}

void DetectionNetworkDepth::setDepthUpperThresholdLimit(uint32_t upper_threshold) {
    properties.upper_threshold = upper_threshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetworkDepth::MobileNetDetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetworkDepth(par, nodeId) {
    properties.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetworkDepth::YoloDetectionNetworkDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetworkDepth(par, nodeId) {
    properties.nnFamily = DetectionNetworkType::YOLO;
}

void YoloDetectionNetworkDepth::setNumClasses(const int numClasses) {
    properties.classes = numClasses;
}

void YoloDetectionNetworkDepth::setCoordinateSize(const int coordinates) {
    properties.coordinates = coordinates;
}

void YoloDetectionNetworkDepth::setAnchors(std::vector<float> anchors) {
    properties.anchors = anchors;
}

void YoloDetectionNetworkDepth::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.anchorMasks = anchorMasks;
}

void YoloDetectionNetworkDepth::setIouThreshold(float thresh) {
    properties.iouThreshold = thresh;
}

}  // namespace node
}  // namespace dai
