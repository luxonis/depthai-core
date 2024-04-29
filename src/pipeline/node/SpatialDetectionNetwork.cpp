#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <sstream>

#include "depthai/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
    neuralNetwork->passthrough.link(detectionParser->imageIn);
    neuralNetwork->passthrough.link(inputImg);
    detectionParser->out.link(inputDetections);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setMaxSize(1);
    detectionParser->imageIn.setBlocking(false);
    detectionParser->imageIn.setMaxSize(1);
    inputDetections.setMaxSize(1);
    inputDetections.setBlocking(true);

    isBuild = true;

    return std::static_pointer_cast<SpatialDetectionNetwork>(shared_from_this());
}

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void SpatialDetectionNetwork::setNNArchive(const NNArchive& nnArchive) {
    const auto blob = detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setBlob(blob);
}

void SpatialDetectionNetwork::setBlobPath(const dai::Path& path) {
    neuralNetwork->setBlobPath(path);
    detectionParser->setBlobPath(path);
}

void SpatialDetectionNetwork::setBlob(OpenVINO::Blob blob) {
    neuralNetwork->setBlob(blob);
    detectionParser->setBlob(blob);
}

void SpatialDetectionNetwork::setBlob(const dai::Path& path) {
    neuralNetwork->setBlob(path);
    detectionParser->setBlob(path);
}

void SpatialDetectionNetwork::setXmlModelPath(const dai::Path& xmlModelPath, const dai::Path& binModelPath) {
    neuralNetwork->setXmlModelPath(xmlModelPath, binModelPath);
}

void SpatialDetectionNetwork::setNumPoolFrames(int numFrames) {
    neuralNetwork->setNumPoolFrames(numFrames);
}

void SpatialDetectionNetwork::setNumInferenceThreads(int numThreads) {
    neuralNetwork->setNumInferenceThreads(numThreads);
}

void SpatialDetectionNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    neuralNetwork->setNumNCEPerInferenceThread(numNCEPerThread);
}

void SpatialDetectionNetwork::setNumShavesPerInferenceThread(int numShavesPerThread) {
    neuralNetwork->setNumShavesPerInferenceThread(numShavesPerThread);
}

void SpatialDetectionNetwork::setBackend(std::string backend) {
    neuralNetwork->setBackend(backend);
}

void SpatialDetectionNetwork::setBackendProperties(std::map<std::string, std::string> props) {
    neuralNetwork->setBackendProperties(props);
}

int SpatialDetectionNetwork::getNumInferenceThreads() {
    return neuralNetwork->getNumInferenceThreads();
}

void SpatialDetectionNetwork::setConfidenceThreshold(float thresh) {
    detectionParser->setConfidenceThreshold(thresh);
}

float SpatialDetectionNetwork::getConfidenceThreshold() const {
    return detectionParser->getConfidenceThreshold();
}

void SpatialDetectionNetwork::setBoundingBoxScaleFactor(float scaleFactor) {
    properties.detectedBBScaleFactor = scaleFactor;
}

void SpatialDetectionNetwork::setDepthLowerThreshold(uint32_t lowerThreshold) {
    properties.depthThresholds.lowerThreshold = lowerThreshold;
}

void SpatialDetectionNetwork::setDepthUpperThreshold(uint32_t upperThreshold) {
    properties.depthThresholds.upperThreshold = upperThreshold;
}

void SpatialDetectionNetwork::setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm calculationAlgorithm) {
    properties.calculationAlgorithm = calculationAlgorithm;
}

void SpatialDetectionNetwork::setSpatialCalculationStepSize(int stepSize) {
    properties.stepSize = stepSize;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
std::shared_ptr<MobileNetSpatialDetectionNetwork> MobileNetSpatialDetectionNetwork::build() {
    SpatialDetectionNetwork::build();
    detectionParser->setNNFamily(DetectionNetworkType::MOBILENET);
    return std::static_pointer_cast<MobileNetSpatialDetectionNetwork>(shared_from_this());
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
std::shared_ptr<YoloSpatialDetectionNetwork> YoloSpatialDetectionNetwork::build() {
    SpatialDetectionNetwork::build();
    detectionParser->setNNFamily(DetectionNetworkType::YOLO);
    return std::static_pointer_cast<YoloSpatialDetectionNetwork>(shared_from_this());
}

void YoloSpatialDetectionNetwork::setNumClasses(const int numClasses) {
    detectionParser->setNumClasses(numClasses);
}

void YoloSpatialDetectionNetwork::setCoordinateSize(const int coordinates) {
    detectionParser->setCoordinateSize(coordinates);
}

void YoloSpatialDetectionNetwork::setAnchors(std::vector<float> anchors) {
    detectionParser->setAnchors(anchors);
}

void YoloSpatialDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    detectionParser->setAnchorMasks(anchorMasks);
}

void YoloSpatialDetectionNetwork::setIouThreshold(float thresh) {
    detectionParser->setIouThreshold(thresh);
}

/// Get num classes
int YoloSpatialDetectionNetwork::getNumClasses() const {
    return detectionParser->getNumClasses();
}

std::optional<std::vector<std::string>> YoloSpatialDetectionNetwork::getClasses() const {
    return detectionParser->getClasses();
}

/// Get coordianate size
int YoloSpatialDetectionNetwork::getCoordinateSize() const {
    return detectionParser->getCoordinateSize();
}

/// Get anchors
std::vector<float> YoloSpatialDetectionNetwork::getAnchors() const {
    return detectionParser->getAnchors();
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloSpatialDetectionNetwork::getAnchorMasks() const {
    return detectionParser->getAnchorMasks();
}

/// Get Iou threshold
float YoloSpatialDetectionNetwork::getIouThreshold() const {
    return detectionParser->getIouThreshold();
}

}  // namespace node
}  // namespace dai
