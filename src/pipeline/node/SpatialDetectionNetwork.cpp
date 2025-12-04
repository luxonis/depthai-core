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
    neuralNetwork->passthrough.link(inputImg);
    detectionParser->out.link(inputDetections);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setMaxSize(1);
    inputDetections.setMaxSize(1);
    inputDetections.setBlocking(true);
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& camera,
                                                                        const std::shared_ptr<StereoDepth>& stereo,
                                                                        NNModelDescription modelDesc,
                                                                        std::optional<float> fps,
                                                                        std::optional<dai::ImgResizeMode> resizeMode) {
    auto nnArchive = createNNArchive(modelDesc);
    return build(camera, stereo, nnArchive, fps, resizeMode);
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& camera,
                                                                        const std::shared_ptr<StereoDepth>& stereo,
                                                                        const NNArchive& nnArchive,
                                                                        std::optional<float> fps,
                                                                        std::optional<dai::ImgResizeMode> resizeMode) {
    neuralNetwork->build(camera, nnArchive, fps, resizeMode);
    detectionParser->setNNArchive(nnArchive);
    alignDepth(stereo, camera);
    return std::static_pointer_cast<SpatialDetectionNetwork>(shared_from_this());
}

NNArchive SpatialDetectionNetwork::createNNArchive(NNModelDescription& modelDesc) {
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
    return nnArchive;
}

void SpatialDetectionNetwork::alignDepth(const std::shared_ptr<StereoDepth>& stereo, const std::shared_ptr<Camera>& camera) {
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        switch(platform) {
            case Platform::RVC4: {
                Subnode<ImageAlign>& align = *depthAlign;
                stereo->depth.link(align->input);
                neuralNetwork->passthrough.link(align->inputAlignTo);
                align->outputAligned.link(inputDepth);
            } break;
            case Platform::RVC2:
                stereo->depth.link(inputDepth);
                neuralNetwork->passthrough.link(stereo->inputAlignTo);
                break;
            case Platform::RVC3:
            default:
                throw std::runtime_error("Unsupported platform");
                break;
        }

    } else {
        stereo->depth.link(inputDepth);
        stereo->setDepthAlign(camera->getBoardSocket());
    }
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
    DAI_CHECK_V(nnArchive.getModelType() == model::ModelType::DLC || nnArchive.getModelType() == model::ModelType::OTHER, "NNArchive type is not DLC or OTHER");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
}

void SpatialDetectionNetwork::setBlobPath(const std::filesystem::path& path) {
    neuralNetwork->setBlobPath(path);
    detectionParser->setBlobPath(path);
}

void SpatialDetectionNetwork::setBlob(OpenVINO::Blob blob) {
    neuralNetwork->setBlob(blob);
    detectionParser->setBlob(blob);
}

void SpatialDetectionNetwork::setBlob(const std::filesystem::path& path) {
    neuralNetwork->setBlob(path);
    detectionParser->setBlob(path);
}

void SpatialDetectionNetwork::setModelPath(const std::filesystem::path& modelPath) {
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

}  // namespace node
}  // namespace dai
