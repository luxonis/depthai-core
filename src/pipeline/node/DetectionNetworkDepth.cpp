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
    return {input, inputDepth};
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

void DetectionNetworkDepth::setConfidenceThreshold(float threshold) {
    properties.confidenceThreshold = threshold;
}

void DetectionNetworkDepth::setBoundingBoxScaleFactor(float factor) {
    properties.detectedBBScaleFactor = factor;
}

void DetectionNetworkDepth::setDepthLowerThreshold(uint32_t lowerThreshold) {
    properties.depthThresholds.lowerThreshold = lowerThreshold;
}

void DetectionNetworkDepth::setDepthUpperThreshold(uint32_t upperThreshold) {
    properties.depthThresholds.upperThreshold = upperThreshold;
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
