#include "depthai/pipeline/node/DetectionNetwork.hpp"

// libraries
#include <nlohmann/json.hpp>
#include <openvino/BlobReader.hpp>

#include "archive.h"
#include "archive_entry.h"

// internal
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

class DetectionNetwork::Impl {
   public:
    Impl() = default;

    /*
     * Place for future private stuff.
     */
};

DetectionNetwork::DetectionNetwork()
    : out{detectionParser->out}, outNetwork{neuralNetwork->out}, input{neuralNetwork->input}, passthrough{neuralNetwork->passthrough} {};
DetectionNetwork::~DetectionNetwork() = default;

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void DetectionNetwork::build() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
    neuralNetwork->passthrough.link(detectionParser->imageIn);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setMaxSize(1);
    detectionParser->imageIn.setBlocking(false);
    detectionParser->imageIn.setMaxSize(1);

    isBuild = true;
}

std::shared_ptr<DetectionNetwork> DetectionNetwork::build(Node::Output& input, const NNArchive& nnArchive) {
    build();
    setNNArchive(nnArchive);
    input.link(this->input);
    return std::static_pointer_cast<DetectionNetwork>(shared_from_this());
}

void DetectionNetwork::setNNArchive(const NNArchive& nnArchive) {
    const auto blob = detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setBlob(blob);
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

void DetectionNetwork::setXmlModelPath(const dai::Path& xmlModelPath, const dai::Path& binModelPath) {
    neuralNetwork->setXmlModelPath(xmlModelPath, binModelPath);
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
    const auto* archive = detectionParser->getNNArchive();
    // TODO(jakgra) only call getRequiredInputs() in the build stage after all user code is supposed to be finished.
    DAI_CHECK_V(archive, "Please call setNNArchive(), before the linking the DetectionNetwork node.");
    auto cap = std::make_shared<ImgFrameCapability>();
    const auto& config = archive->getConfig().getConfigV1();
    DAI_CHECK_V(config, "Wrong NNArchive config version");
    const auto width = (*config).model.inputs[0].shape[2];
    const auto height = (*config).model.inputs[0].shape[3];
    cap->size.value = std::pair(width, height);
    return {{input, cap}};
}

std::optional<std::vector<std::string>> DetectionNetwork::getClasses() const {
    return detectionParser->getClasses();
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
std::shared_ptr<MobileNetDetectionNetwork> MobileNetDetectionNetwork::build() {
    DetectionNetwork::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::MOBILENET;

    return std::static_pointer_cast<MobileNetDetectionNetwork>(shared_from_this());
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
std::shared_ptr<YoloDetectionNetwork> YoloDetectionNetwork::build() {
    DetectionNetwork::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    detectionParser->properties.parser.iouThreshold = 0.5f;

    return std::static_pointer_cast<YoloDetectionNetwork>(shared_from_this());
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
