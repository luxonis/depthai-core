#include "depthai/pipeline/node/DetectionParser.hpp"

#include "common/ModelType.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "nn_archive/NNArchive.hpp"
#include "spdlog/fmt/fmt.h"

// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

void DetectionParser::setNNArchive(const NNArchive& nnArchive) {
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
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
    }
}

void DetectionParser::setModelPath(const dai::Path& modelPath) {
    switch(model::readModelType(modelPath.string())) {
        case model::ModelType::BLOB:
        case model::ModelType::SUPERBLOB:
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            break;  // Just do nothing
        case model::ModelType::NNARCHIVE:
            setNNArchive(dai::NNArchive(modelPath.string()));
            break;
        default:
            DAI_CHECK_V(false, "Unknown model type");
            break;
    }
}

const NNArchiveVersionedConfig& DetectionParser::getNNArchiveVersionedConfig() const {
    DAI_CHECK_V(archiveConfig.has_value(), "NNArchiveVersionedConfig is not set. Use setNNArchive(...) first.");
    return archiveConfig.value();
}

void DetectionParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    archiveConfig = config;

    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    auto configV1 = config.getConfig<nn_archive::v1::Config>();

    const auto model = configV1.model;
    // TODO(jakgra) is NN Archive valid without this? why is this optional?
    DAI_CHECK(model.heads, "Heads array is not defined in the NN Archive config file.");
    // TODO(jakgra) for now get info from heads[0] but in the future correctly support multiple outputs and mapped heads
    DAI_CHECK_V((*model.heads).size() == 1,
                "There should be exactly one head per model in the NN Archive config file defined. Found {} heads.",
                (*model.heads).size());
    const auto head = (*model.heads)[0];

    if(head.parser == "YOLO") {
        properties.parser.nnFamily = DetectionNetworkType::YOLO;
    } else if(head.parser == "SSD" || head.parser == "MOBILENET") {
        properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
    } else {
        DAI_CHECK_V(false, "Unsupported parser: {}", head.parser);
    }

    if(head.metadata.classes) {
        setClasses(*head.metadata.classes);
    }
    if(head.metadata.nClasses) {
        setNumClasses(static_cast<int>(*head.metadata.nClasses));
    }
    if(head.metadata.iouThreshold) {
        properties.parser.iouThreshold = static_cast<float>(*head.metadata.iouThreshold);
    }
    if(head.metadata.confThreshold) {
        setConfidenceThreshold(static_cast<float>(*head.metadata.confThreshold));
    }
    setCoordinateSize(4);
    if(head.metadata.anchors) {
        const auto anchorsIn = *head.metadata.anchors;
        std::vector<std::vector<std::vector<float>>> anchorsOut(anchorsIn.size());
        for(size_t layer = 0; layer < anchorsOut.size(); ++layer) {
            std::vector<std::vector<float>> layerOut(anchorsIn[layer].size());
            for(size_t anchor = 0; anchor < layerOut.size(); ++anchor) {
                std::vector<float> anchorOut(anchorsIn[layer][anchor].size());
                for(size_t dim = 0; dim < anchorOut.size(); ++dim) {
                    anchorOut[dim] = static_cast<float>(anchorsIn[layer][anchor][dim]);
                }
                layerOut[anchor] = anchorOut;
            }
            anchorsOut[layer] = layerOut;
        }
        setAnchors(anchorsOut);
    }
}

void DetectionParser::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    setConfig(nnArchive.getVersionedConfig());
    setBlob(nnArchive.getBlob().value());
}

void DetectionParser::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    setConfig(nnArchive.getVersionedConfig());
    setBlob(nnArchive.getSuperBlob()->getBlobWithNumShaves(numShaves));
}

void DetectionParser::setNNArchiveOther(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void DetectionParser::setBlob(OpenVINO::Blob blob) {
    properties.networkInputs = blob.networkInputs;
}

void DetectionParser::setBlobPath(const dai::Path& path) {
    setBlob(OpenVINO::Blob(path));
}

void DetectionParser::setBlob(const dai::Path& path) {
    setBlobPath(path);
}

void DetectionParser::setInputImageSize(std::tuple<int, int> size) {
    setInputImageSize(std::get<0>(size), std::get<1>(size));
}

void DetectionParser::setInputImageSize(int width, int height) {
    dai::TensorInfo tensorInfo{};
    tensorInfo.dims = std::vector<unsigned int>{static_cast<unsigned int>(width), static_cast<unsigned int>(height)};
    tensorInfo.numDimensions = 2;
    if(properties.networkInputs.size() != 0) {
        logger->error("setInputImageSize(...) can only be used if setBlob(...) is not in use. Otherwise input sizes are parsed from the blob.");
        return;
    }
    properties.networkInputs.emplace("input", tensorInfo);
}

void DetectionParser::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

int DetectionParser::getNumFramesPool() {
    return properties.numFramesPool;
}

void DetectionParser::setNNFamily(DetectionNetworkType type) {
    properties.parser.nnFamily = type;
}

DetectionNetworkType DetectionParser::getNNFamily() {
    return properties.parser.nnFamily;
}

void DetectionParser::setConfidenceThreshold(float thresh) {
    properties.parser.confidenceThreshold = thresh;
}

float DetectionParser::getConfidenceThreshold() const {
    return properties.parser.confidenceThreshold;
}

void DetectionParser::setNumClasses(const int numClasses) {
    properties.parser.classes = numClasses;
}

void DetectionParser::setCoordinateSize(const int coordinates) {
    properties.parser.coordinates = coordinates;
}

void DetectionParser::setAnchors(std::vector<float> anchors) {
    properties.parser.anchors = anchors;
}

void DetectionParser::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.parser.anchorMasks = anchorMasks;
}

void DetectionParser::setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors) {
    properties.parser.anchorsV2 = anchors;
}

void DetectionParser::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

/// Get num classes
int DetectionParser::getNumClasses() const {
    return properties.parser.classes;
}

std::optional<std::vector<std::string>> DetectionParser::getClasses() const {
    return mClasses;
}

void DetectionParser::setClasses(const std::vector<std::string>& classes) {
    mClasses = classes;
}

/// Get coordianate size
int DetectionParser::getCoordinateSize() const {
    return properties.parser.coordinates;
}

/// Get anchors
std::vector<float> DetectionParser::getAnchors() const {
    return properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> DetectionParser::getAnchorMasks() const {
    return properties.parser.anchorMasks;
}

/// Get Iou threshold
float DetectionParser::getIouThreshold() const {
    return properties.parser.iouThreshold;
}

}  // namespace node
}  // namespace dai
