#include "depthai/pipeline/node/DetectionNetwork.hpp"

// C++ std
#include <sstream>

// libraries
#include <nlohmann/json.hpp>
#include <openvino/BlobReader.hpp>

#include "archive.h"
#include "archive_entry.h"

// internal
#include "depthai/common/DetectionNetworkType.hpp"
#include "json_types/Generators.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/Logging.hpp"

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

void DetectionNetwork::setNNArchive(const dai::Path& path, const NNArchiveFormat format) {
    const auto filepath = path.string();
#if defined(_WIN32) && defined(_MSC_VER)
    const char separator = '\\';
#else
    const char separator = '/';
#endif
    const size_t lastSlashIndex = filepath.find_last_of(separator);
    std::string archiveName;
    if(std::string::npos == lastSlashIndex) {
        archiveName = filepath;
    } else {
        archiveName = filepath.substr(lastSlashIndex + 1);
    }
    bool isJson = format == NNArchiveFormat::RAW_FS;
    if(format == NNArchiveFormat::AUTO) {
        const auto pointIndex = filepath.find_last_of('.');
        if(pointIndex != std::string::npos) {
            isJson = filepath.substr(filepath.find_last_of('.') + 1) == "json";
        }
    }
    std::optional<nlohmann::json> maybeJson;
    if(isJson) {
        std::ifstream jsonStream(path);
        maybeJson = nlohmann::json::parse(jsonStream);
    } else {
        dai::utility::ArchiveUtil archive(filepath, format);
        struct archive_entry* entry = nullptr;
        bool foundJson = false;
        while(archive_read_next_header(archive.getA(), &entry) == ARCHIVE_OK) {
            std::string entryName(archive_entry_pathname(entry));
            if(entryName == archiveName + ".json") {
                foundJson = true;
                const auto jsonBytes = archive.readEntry(entry);
                maybeJson = nlohmann::json::parse(jsonBytes);
                break;
            }
        }
        daiCheckV(foundJson, "Didn't find the {}.json file inside the {} archive.", archiveName, filepath);
    }
    daiCheckIn(maybeJson);
    const auto json = *maybeJson;
    dai::json_types::NnArchiveConfig config;
    dai::json_types::from_json(json, config);
    const auto model = config.model;
    if(isJson) {
        std::string blobPath;
        if(std::string::npos == lastSlashIndex) {
            blobPath = model.metadata.path;
        } else {
            const auto basedir = filepath.substr(0, lastSlashIndex + 1);
            blobPath = basedir + separator + model.metadata.path;
        }
        setBlobPath(blobPath);
    } else {
        dai::utility::ArchiveUtil archive(filepath, format);
        struct archive_entry* entry = nullptr;
        bool found = false;
        while(archive_read_next_header(archive.getA(), &entry) == ARCHIVE_OK) {
            std::string entryName(archive_entry_pathname(entry));
            if(entryName == model.metadata.path) {
                found = true;
                // TODO(jakgra) maybe do this async in another thread and check for call to setBlob() in between
                // to interrupt extraction in the while loop and overwrite the blob
                // Then startPipeline should wait for the extraction to finish
                const auto blobBytes = archive.readEntry(entry);
                setBlob(OpenVINO::Blob(blobBytes));
                break;
            }
        }
        daiCheckV(found, "No blob named {} found in NN Archive {}.", model.metadata.path, filepath)
    }
    // TODO(jakgra) is NN Archive valid without this? why is this optional?
    daiCheck(model.heads, "Heads array is not defined in the NN Archive config file.");
    // TODO(jakgra) for now get info from heads[0] but in the future correctly support multiple outputs and mapped heads
    daiCheckV(
        (*model.heads).size() == 1, "There should be exactly one head per model in the NN Archive config file defined. Found {} heads.", (*model.heads).size());
    const auto headMeta = (*model.heads)[0].metadata;
    if(headMeta.family == dai::json_types::Family::OBJECT_DETECTION_YOLO) {
        detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    }
    detectionParser->setNumClasses(static_cast<int>(headMeta.nClasses));
    if(headMeta.iouThreshold) {
        detectionParser->properties.parser.iouThreshold = static_cast<float>(*headMeta.iouThreshold);
    }
    if(headMeta.confThreshold) {
        setConfidenceThreshold(static_cast<float>(*headMeta.confThreshold));
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
