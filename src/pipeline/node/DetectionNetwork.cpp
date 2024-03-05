#include "depthai/pipeline/node/DetectionNetwork.hpp"

// C++ std
#include <sstream>

// libraries
#include <nlohmann/json.hpp>
#include <openvino/BlobReader.hpp>

// internal
#include "depthai/common/DetectionNetworkType.hpp"
#include "json_types/Generators.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

void DetectionNetwork::build() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
    neuralNetwork->passthrough.link(detectionParser->imageIn);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setQueueSize(1);
    detectionParser->imageIn.setBlocking(false);
    detectionParser->imageIn.setQueueSize(1);
}

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void DetectionNetwork::setNNArchive(const dai::Path& path) {
    // Should we allow to specify the format separately also?
    // in a function param using an enum with default AUTO which does the below?
    if(false /* path ends in .json */) {
        // check in same directory for the file specified in path
        // should we handle absolute paths here differently without looking into same directory?
    } else if(false /* path ends in .tar */) {
        // just get the file handle using libarchive, don't need to decompress anything.
    } else if(false /* path ends in .tar.xz */) {
        // decompress maybe libarchive has some magic handling of different file endings? try it.
    } else if(false /* path ends in .tar.gz */) {
        // decompress
    }
    std::cout << "USING PATH:" << path.string() << std::endl;
    std::ifstream jsonStream(path);
    nlohmann::json json = nlohmann::json::parse(jsonStream);
    dai::json_types::NnArchiveConfig config;
    dai::json_types::from_json(json, config);
    if(config.stages.size() != 1) {
        throw std::runtime_error(
            fmt::format("There should be exactly one stage in the NN Archive config file defined. Found {} stages.", config.stages.size()));
    }
    const auto stage = config.stages[0];
    std::cout << "ARCHIVE NAME WAS: " << stage.metadata.name << std::endl;
    std::cout << "ARCHIVE PATH WAS: " << stage.metadata.path << std::endl;
#if defined(_WIN32) && defined(_MSC_VER)
    const auto separator = "\\";
#else
    const auto separator = "/";
#endif
    std::string blobPath;
    const auto filepath = path.string();
    const size_t lastSlashIndex = filepath.find_last_of(separator);
    if(std::string::npos == lastSlashIndex) {
        blobPath = stage.metadata.path;
    } else {
        const auto basedir = filepath.substr(0, lastSlashIndex + 1);
        std::cout << "BASE DIR: " << basedir << std::endl;
        blobPath = basedir + separator + stage.metadata.path;
    }
    std::cout << "BLOB PATH: " << blobPath << std::endl;
    setBlobPath(blobPath);

    // TODO is NN Archive valid without this? why is this optional?
    if(!stage.heads) {
    throw std::runtime_error(
                fmt::format("Heads array is not defined in the NN Archive config file.");
    }
    // TODO for now get info from heads[0] but in the future correctly support multiple outputs and mapped heads
    if((*stage.heads).size() != 1) {
    throw std::runtime_error(
        fmt::format("There should be exactly one head per stage in the NN Archive config file defined. Found {} stages.", (*stage.heads).size()));
    }
    const auto headMeta = (*stage.heads)[0].metadata;
    if(headMeta.family == dai::json_types::Family::OBJECT_DETECTION_YOLO) {
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    }
    detectionParser->setNumClasses(headMeta.nClasses);
    if(headMeta.iouThreshold) {
    detectionParser->properties.parser.iouThreshold = *headMeta.iouThreshold;
    }
    if(headMeta.confThreshold) {
    setConfidenceThreshold(*headMeta.confThreshold);
    }
    detectionParser->setCoordinateSize(4);
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

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetDetectionNetwork::build() {
    DetectionNetwork::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloDetectionNetwork::build() {
    DetectionNetwork::build();
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
