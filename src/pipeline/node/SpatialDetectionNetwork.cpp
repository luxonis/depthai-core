#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <sstream>

#include "../../utility/ErrorMacros.hpp"
#include "depthai/common/DetectionNetworkType.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "nn_archive/NNArchive.hpp"
#include "openvino/BlobReader.hpp"
#include "openvino/OpenVINO.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

void SpatialDetectionNetwork::buildInternal() {
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
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& camera,
                                                                        const std::shared_ptr<StereoDepth>& stereo,
                                                                        dai::NNModelDescription modelDesc,
                                                                        float fps) {
    // Download model from zoo
    if(modelDesc.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set.");
        modelDesc.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = dai::model::readModelType(path);
    DAI_CHECK(modelType == dai::model::ModelType::NNARCHIVE,
              "Model from zoo is not NNArchive - it needs to be a NNArchive to use build(Camera, NNModelDescription, float) method");
    auto nnArchive = dai::NNArchive(path);
    return build(camera, stereo, nnArchive, fps);
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& camera,
                                                                        const std::shared_ptr<StereoDepth>& stereo,
                                                                        dai::NNArchive nnArchive,
                                                                        float fps) {
    neuralNetwork->build(camera, nnArchive, fps);
    detectionParser->setNNArchive(nnArchive);
    stereo->depth.link(inputDepth);
    stereo->setDepthAlign(camera->getBoardSocket());
    return std::static_pointer_cast<SpatialDetectionNetwork>(shared_from_this());
}

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void SpatialDetectionNetwork::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;

    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            setNNArchiveOther(nnArchive);
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void SpatialDetectionNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, numShaves);
            break;
        case dai::model::ModelType::BLOB:
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            DAI_CHECK_V(false, "NNArchive type is not SUPERBLOB. Use setNNArchive(const NNArchive& nnArchive) instead.");
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void SpatialDetectionNetwork::setFromModelZoo(NNModelDescription description, bool useCached) {
    // Download model from zoo
    if(description.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set. Use setDevice(...) first.");
        description.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(description, useCached);
    setModelPath(path);
}

void SpatialDetectionNetwork::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
}

void SpatialDetectionNetwork::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive, numShaves);
}

void SpatialDetectionNetwork::setNNArchiveOther(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::OTHER, "NNArchive type is not OTHER");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
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

void SpatialDetectionNetwork::setModelPath(const dai::Path& modelPath) {
    neuralNetwork->setModelPath(modelPath);
    detectionParser->setModelPath(modelPath);
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

std::optional<std::vector<std::string>> SpatialDetectionNetwork::getClasses() const {
    return detectionParser->getClasses();
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetSpatialDetectionNetwork::buildInternal() {
    SpatialDetectionNetwork::buildInternal();
    detectionParser->setNNFamily(DetectionNetworkType::MOBILENET);
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloSpatialDetectionNetwork::buildInternal() {
    SpatialDetectionNetwork::buildInternal();
    detectionParser->setNNFamily(DetectionNetworkType::YOLO);
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
