#include "depthai/pipeline/node/DetectionNetwork.hpp"

// libraries
#include <nlohmann/json.hpp>
#include <openvino/BlobReader.hpp>

#include "archive.h"
#include "archive_entry.h"

// internal
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "nn_archive/NNArchiveVersionedConfig.hpp"
#include "pipeline/DeviceNodeGroup.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

DetectionNetwork::DetectionNetwork(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device),
      out{detectionParser->out},
      outNetwork{neuralNetwork->out},
      input{neuralNetwork->input},
      passthrough{neuralNetwork->passthrough} {};

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void DetectionNetwork::buildInternal() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
    neuralNetwork->passthrough.link(detectionParser->imageIn);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setMaxSize(1);
    detectionParser->imageIn.setBlocking(false);
    detectionParser->imageIn.setMaxSize(1);
}

std::shared_ptr<DetectionNetwork> DetectionNetwork::build(Node::Output& input, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);
    input.link(this->input);
    return std::static_pointer_cast<DetectionNetwork>(shared_from_this());
}

std::shared_ptr<DetectionNetwork> DetectionNetwork::build(std::shared_ptr<Camera> camera, dai::NNModelDescription modelDesc, float fps) {
    setFromModelZoo(modelDesc);

    // We only support config v1 for now
    DAI_CHECK_V(detectionParser->getNNArchiveVersionedConfig().getVersion() == NNArchiveConfigVersion::V1, "Only NNConfigV1 is supported for DetectionNetwork");
    auto nnArchiveConfigV1 = detectionParser->getNNArchiveVersionedConfig().getConfig<nn_archive::v1::Config>();

    // Perform basic checks
    DAI_CHECK_V(nnArchiveConfigV1.model.inputs.size() == 1, "Only single input model is supported");
    DAI_CHECK_V(nnArchiveConfigV1.model.inputs[0].shape.size() == 4, "Only 4D input shape is supported");
    DAI_CHECK_V(nnArchiveConfigV1.model.inputs[0].shape[0] == 1 && nnArchiveConfigV1.model.inputs[0].shape[1] == 3, "Only 3 channel input is supported");

    // Unpack input dimensions
    auto inputHeight = nnArchiveConfigV1.model.inputs[0].shape[2];
    auto inputWidth = nnArchiveConfigV1.model.inputs[0].shape[3];

    auto type = dai::ImgFrame::Type::BGR888p;
    auto platform = getDevice()->getPlatform();
    if(platform == dai::Platform::RVC2 || platform == dai::Platform::RVC3) {
        type = dai::ImgFrame::Type::BGR888p;
    } else if(platform == dai::Platform::RVC4) {
        type = dai::ImgFrame::Type::BGR888i;
    } else {
        DAI_CHECK_V(false, "Unsupported platform");
    }

    auto cap = ImgFrameCapability();
    cap.size.value = std::pair(inputWidth, inputHeight);
    cap.type = type;
    cap.fps.value = fps;
    auto* input = camera->requestOutput(cap, false);
    if(!input) {
        DAI_CHECK_V(false, "Camera does not have output with requested capabilities");
    }
    input->link(this->input);
    return std::static_pointer_cast<DetectionNetwork>(shared_from_this());
}

void DetectionNetwork::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case dai::model::ModelType::DLC:
        case dai::model::ModelType::OTHER:
            setNNArchiveOther(nnArchive);
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s" __FILE__, __LINE__);
            break;
    }
}

void DetectionNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, numShaves);
            break;
        case dai::model::ModelType::BLOB:
        case dai::model::ModelType::DLC:
        case dai::model::ModelType::OTHER:
            DAI_CHECK_V(false, "NNArchive type is not SUPERBLOB. Use setNNArchive(const NNArchive& nnArchive) instead.");
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s" __FILE__, __LINE__);
            break;
    }
}

void DetectionNetwork::setFromModelZoo(NNModelDescription description, bool useCached) {
    // Download model from zoo
    if(description.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set. Use setDevice(...) first.");
        description.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(description, useCached);
    setModelPath(path);
}

void DetectionNetwork::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    neuralNetwork->setNNArchive(nnArchive);
    try {
        detectionParser->setNNArchive(nnArchive);
    } catch (const std::exception& e) {
        std::cerr << "Error setting NNArchive in DetectionParser: " << e.what() << std::endl;
        std::cerr << "In case your model has no parser heads, try switching to NeuralNetwork node instead." << std::endl;
        throw;
    }
}

void DetectionNetwork::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    neuralNetwork->setNNArchive(nnArchive, numShaves);
    try {
        detectionParser->setNNArchive(nnArchive);
    } catch (const std::exception& e) {
        std::cerr << "Error setting NNArchive in DetectionParser: " << e.what() << std::endl;
        std::cerr << "In case your model has no parser heads, try switching to NeuralNetwork node instead." << std::endl;
        throw;
    }
}

void DetectionNetwork::setNNArchiveOther(const NNArchive& nnArchive) {
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
}

void DetectionNetwork::setBlobPath(const dai::Path& path) {
    neuralNetwork->setBlobPath(path);
    detectionParser->setBlobPath(path);
}

void DetectionNetwork::setBlob(OpenVINO::Blob blob) {
    neuralNetwork->setBlob(blob);
    detectionParser->setBlob(blob);
}

void DetectionNetwork::setBlob(const dai::Path& path) {
    neuralNetwork->setBlob(path);
    detectionParser->setBlob(path);
}

void DetectionNetwork::setModelPath(const dai::Path& modelPath) {
    neuralNetwork->setModelPath(modelPath);
    detectionParser->setModelPath(modelPath);
}

void DetectionNetwork::setNumPoolFrames(int numFrames) {
    neuralNetwork->setNumPoolFrames(numFrames);
}

void DetectionNetwork::setNumInferenceThreads(int numThreads) {
    neuralNetwork->setNumInferenceThreads(numThreads);
}

void DetectionNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    neuralNetwork->setNumNCEPerInferenceThread(numNCEPerThread);
}

void DetectionNetwork::setNumShavesPerInferenceThread(int numShavesPerThread) {
    neuralNetwork->setNumShavesPerInferenceThread(numShavesPerThread);
}

void DetectionNetwork::setBackend(std::string backend) {
    neuralNetwork->setBackend(backend);
}

void DetectionNetwork::setBackendProperties(std::map<std::string, std::string> props) {
    neuralNetwork->setBackendProperties(props);
}

int DetectionNetwork::getNumInferenceThreads() {
    return neuralNetwork->getNumInferenceThreads();
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    detectionParser->setConfidenceThreshold(thresh);
}

float DetectionNetwork::getConfidenceThreshold() const {
    return detectionParser->getConfidenceThreshold();
}

std::vector<std::pair<Node::Input&, std::shared_ptr<Capability>>> DetectionNetwork::getRequiredInputs() {
    const dai::NNArchiveVersionedConfig& config = detectionParser->getNNArchiveVersionedConfig();
    DAI_CHECK(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNConfigV1 is supported for DetectionNetwork");
    const auto configV1 = config.getConfig<nn_archive::v1::Config>();

    const auto width = configV1.model.inputs[0].shape[2];
    const auto height = configV1.model.inputs[0].shape[3];

    auto cap = std::make_shared<ImgFrameCapability>();
    cap->size.value = std::pair(width, height);
    return {{input, cap}};
}

std::optional<std::vector<std::string>> DetectionNetwork::getClasses() const {
    return detectionParser->getClasses();
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetDetectionNetwork::buildInternal() {
    DetectionNetwork::buildInternal();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloDetectionNetwork::buildInternal() {
    DetectionNetwork::buildInternal();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    detectionParser->properties.parser.iouThreshold = 0.5f;
}

void YoloDetectionNetwork::setNumClasses(const int numClasses) {
    detectionParser->setNumClasses(numClasses);
}

void YoloDetectionNetwork::setCoordinateSize(const int coordinates) {
    detectionParser->setCoordinateSize(coordinates);
}

void YoloDetectionNetwork::setAnchors(std::vector<float> anchors) {
    detectionParser->setAnchors(anchors);
}

void YoloDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    detectionParser->setAnchorMasks(anchorMasks);
}

void YoloDetectionNetwork::setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors) {
    detectionParser->setAnchors(anchors);
}

void YoloDetectionNetwork::setIouThreshold(float thresh) {
    detectionParser->setIouThreshold(thresh);
}

/// Get num classes
int YoloDetectionNetwork::getNumClasses() const {
    return detectionParser->getNumClasses();
}

/// Get coordianate size
int YoloDetectionNetwork::getCoordinateSize() const {
    return detectionParser->getCoordinateSize();
}

/// Get anchors
std::vector<float> YoloDetectionNetwork::getAnchors() const {
    return detectionParser->getAnchors();
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetwork::getAnchorMasks() const {
    return detectionParser->getAnchorMasks();
}

/// Get Iou threshold
float YoloDetectionNetwork::getIouThreshold() const {
    return detectionParser->getIouThreshold();
}

}  // namespace node
}  // namespace dai
