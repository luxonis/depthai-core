#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {}

std::string SpatialDetectionNetwork::getName() const {
    return "SpatialDetectionNetwork";
}

std::vector<Node::Input> SpatialDetectionNetwork::getInputs() {
    return {input, inputDepth};
}

std::vector<Node::Output> SpatialDetectionNetwork::getOutputs() {
    return {out, passthroughRoi};
}

dai::SpatialDetectionNetworkProperties& SpatialDetectionNetwork::getPropertiesRef() {
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

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloSpatialDetectionNetwork::YoloSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : SpatialDetectionNetwork(par, nodeId) {
    getPropertiesRef().nnFamily = DetectionNetworkType::YOLO;
}

}  // namespace node
}  // namespace dai
