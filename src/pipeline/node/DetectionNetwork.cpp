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
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<NeuralNetwork, DetectionNetwork, DetectionNetworkProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&input});
    setOutputRefs({&out, &passthrough});

    // Default confidence threshold
    properties.confidenceThreshold = 0.5;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    properties.confidenceThreshold = thresh;
}

float DetectionNetwork::getConfidenceThreshold() const {
    return properties.confidenceThreshold;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : MobileNetDetectionNetwork(par, nodeId, std::make_unique<Properties>()) {}
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DetectionNetwork, MobileNetDetectionNetwork, DetectionNetworkProperties>(par, nodeId, std::move(props)) {
    properties.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : YoloDetectionNetwork(par, nodeId, std::make_unique<Properties>()) {}
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DetectionNetwork, YoloDetectionNetwork, DetectionNetworkProperties>(par, nodeId, std::move(props)) {
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

/// Get num classes
int YoloDetectionNetwork::getNumClasses() const {
    return properties.classes;
}

/// Get coordianate size
int YoloDetectionNetwork::getCoordinateSize() const {
    return properties.coordinates;
}

/// Get anchors
std::vector<float> YoloDetectionNetwork::getAnchors() const {
    return properties.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetwork::getAnchorMasks() const {
    return properties.anchorMasks;
}

/// Get Iou threshold
float YoloDetectionNetwork::getIouThreshold() const {
    return properties.iouThreshold;
}

}  // namespace node
}  // namespace dai
