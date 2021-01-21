#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/datatype/DetectionNetworkType.hpp"
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
    return {out};
}

nlohmann::json DetectionNetwork::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    properties.confidenceThreshold = thresh;
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void DetectionNetwork::setBlobPath(const std::string& path) {
    blobPath = path;
    BlobAssetInfo blobInfo = loadBlob(path);
    properties.blobUri = blobInfo.uri;
    properties.blobSize = blobInfo.size;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = (uint32_t)DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = (uint32_t)DetectionNetworkType::YOLO;
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

}  // namespace node
}  // namespace dai
