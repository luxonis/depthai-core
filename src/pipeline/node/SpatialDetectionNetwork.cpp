#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    inputs = {&input, &inputDepth};
    outputs = {&out, &boundingBoxMapping, &passthrough, &passthroughDepth};
}

std::shared_ptr<Node> SpatialDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

std::string SpatialDetectionNetwork::getName() const {
    return "SpatialDetectionNetwork";
}

SpatialDetectionNetwork::Properties& SpatialDetectionNetwork::getPropertiesRef() {
    return properties;
}

nlohmann::json SpatialDetectionNetwork::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

void SpatialDetectionNetwork::setBoundingBoxScaleFactor(float scaleFactor) {
    getPropertiesRef().detectedBBScaleFactor = scaleFactor;
}

void SpatialDetectionNetwork::setDepthLowerThreshold(uint32_t lowerThreshold) {
    getPropertiesRef().depthThresholds.lowerThreshold = lowerThreshold;
}

void SpatialDetectionNetwork::setDepthUpperThreshold(uint32_t upperThreshold) {
    getPropertiesRef().depthThresholds.upperThreshold = upperThreshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetSpatialDetectionNetwork::MobileNetSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : SpatialDetectionNetwork(par, nodeId) {
    getPropertiesRef().nnFamily = DetectionNetworkType::MOBILENET;
}

std::shared_ptr<Node> MobileNetSpatialDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloSpatialDetectionNetwork::YoloSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : SpatialDetectionNetwork(par, nodeId) {
    getPropertiesRef().nnFamily = DetectionNetworkType::YOLO;
}

std::shared_ptr<Node> YoloSpatialDetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void YoloSpatialDetectionNetwork::setNumClasses(const int numClasses) {
    getPropertiesRef().classes = numClasses;
}

void YoloSpatialDetectionNetwork::setCoordinateSize(const int coordinates) {
    getPropertiesRef().coordinates = coordinates;
}

void YoloSpatialDetectionNetwork::setAnchors(std::vector<float> anchors) {
    getPropertiesRef().anchors = anchors;
}

void YoloSpatialDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    getPropertiesRef().anchorMasks = anchorMasks;
}

void YoloSpatialDetectionNetwork::setIouThreshold(float thresh) {
    getPropertiesRef().iouThreshold = thresh;
}

}  // namespace node
}  // namespace dai
