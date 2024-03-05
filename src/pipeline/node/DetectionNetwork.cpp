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

// TODO support setting NNArchive from memory location
void DetectionNetwork::setNNArchive(const dai::Path& path, const NNArchiveFormat format) {
    const auto filepath = path.string();
#if defined(_WIN32) && defined(_MSC_VER)
    const auto separator = "\\";
#else
    const auto separator = "/";
#endif
    std::string blobPath;
    const size_t lastSlashIndex = filepath.find_last_of(separator);
    std::string archiveName;
    if(std::string::npos == lastSlashIndex) {
        archiveName = filepath;
    } else {
        archiveName = filepath.substr(lastSlashIndex + 1);
    }
    std::cout << "ARCHIVE NAME: " << archiveName << std::endl;
    bool isJson = format == NNArchiveFormat::RAW_FS;
    if(format == NNArchiveFormat::AUTO) {
        const auto pointIndex = filepath.find_last_of(".");
        if(pointIndex != std::string::npos) {
            isJson = filepath.substr(filepath.find_last_of(".") + 1) == "json";
        }
    }
    if(!isJson) {
        const auto a = archive_read_new();
        switch(format) {
            case NNArchiveFormat::AUTO:
                archive_read_support_filter_all(a);
                archive_read_support_format_all(a);
                break;
            case NNArchiveFormat::TAR:
                archive_read_support_filter_none(a);
                archive_read_support_format_tar(a);
                break;
            case NNArchiveFormat::TAR_GZ:
                archive_read_support_filter_gzip(a);
                archive_read_support_format_tar(a);
                break;
            case NNArchiveFormat::TAR_XZ:
                archive_read_support_filter_xz(a);
                archive_read_support_format_tar(a);
                break;
            case NNArchiveFormat::RAW_FS:
                throw std::runtime_error("");
                break;
        }
        auto rc = archive_read_open_filename(a, filepath.c_str(), 10240);
        if(rc != ARCHIVE_OK) {
            throw std::runtime_error(fmt::format("Error when decompressing {}.", filepath));
        }
        struct archive_entry* entry;
        while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
            std::string entryName(archive_entry_pathname(entry));
            printf("COMPRESSED ENTRY: %s\n", archive_entry_pathname(entry));
            if(entryName == archiveName + ".json") {
                std::cout << "FOUND JSON FILE: " << entryName << std::endl;
            }
        }
        rc = archive_read_free(a);  // Note 3
        if(rc != ARCHIVE_OK) {
            logger::warn("couldn't free archive while handling {}.", filepath);
        }
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
    if(std::string::npos == lastSlashIndex) {
        blobPath = stage.metadata.path;
    } else {
        const auto basedir = filepath.substr(0, lastSlashIndex + 1);
        std::cout << "BASE DIR: " << basedir << std::endl;
        blobPath = basedir + separator + stage.metadata.path;
    }
    std::cout << "BLOB PATH: " << blobPath << std::endl;
    // TODO handle compressed streams here...
    setBlobPath(blobPath);

    // TODO is NN Archive valid without this? why is this optional?
    if(!stage.heads) {
        throw std::runtime_error("Heads array is not defined in the NN Archive config file.");
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
