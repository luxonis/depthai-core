#include "depthai/pipeline/node/DetectionParser.hpp"

#include "nn_archive/NNArchive.hpp"
#include "spdlog/fmt/fmt.h"

// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

std::reference_wrapper<const OpenVINO::Blob> DetectionParser::setNNArchive(const NNArchive& nnArchive) {
    mArchive = nnArchive;
    const auto configMaybe = nnArchive.getConfig().getConfigV1();
    DAI_CHECK(configMaybe, "Unsupported NNArchive format / version. Check which depthai version you are running.");
    const auto& config = *configMaybe;
    const auto& blob = nnArchive.getBlob().getOpenVINOBlob();
    DAI_CHECK_IN(blob);
    setBlob(*blob);
    const auto model = config.model;
    // TODO(jakgra) is NN Archive valid without this? why is this optional?
    DAI_CHECK(model.heads, "Heads array is not defined in the NN Archive config file.");
    // TODO(jakgra) for now get info from heads[0] but in the future correctly support multiple outputs and mapped h  eads
    DAI_CHECK_V((*model.heads).size() == 1,
                "There should be exactly one head per model in the NN Archive config file define  d. Found {} heads.",
                (*model.heads).size());
    const auto head = (*model.heads)[0];
    if(head.family == "ObjectDetectionYOLO") {
        properties.parser.nnFamily = DetectionNetworkType::YOLO;
    }
    setClasses(head.classes);
    setNumClasses(static_cast<int>(head.nClasses));
    if(head.iouThreshold) {
        properties.parser.iouThreshold = static_cast<float>(*head.iouThreshold);
    }
    if(head.confThreshold) {
        setConfidenceThreshold(static_cast<float>(*head.confThreshold));
    }
    setCoordinateSize(4);
    if(head.anchors) {
        const auto anchorsIn = *head.anchors;
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
    return *blob;
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

const NNArchive* DetectionParser::getNNArchive() const {
    return mArchive ? &(*mArchive) : nullptr;
}

}  // namespace node
}  // namespace dai
