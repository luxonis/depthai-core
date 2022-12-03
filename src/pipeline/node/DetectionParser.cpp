#include "depthai/pipeline/node/DetectionParser.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

void DetectionParser::setBlob(const OpenVINO::Blob& blob) {
    properties.networkInputs = blob.networkOutputs;
}

void DetectionParser::setInputImageSize(std::tuple<int, int> size) {
    setInputImageSize(std::get<0>(size), std::get<1>(size));
}

void DetectionParser::setInputImageSize(int width, int height) {
    dai::TensorInfo tensorInfo{};
    tensorInfo.dims = std::vector<unsigned int>{static_cast<unsigned int>(width), static_cast<unsigned int>(height)};
    tensorInfo.numDimensions = 2;
    if(properties.networkInputs.size() != 0){
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

void DetectionParser::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

/// Get num classes
int DetectionParser::getNumClasses() const {
    return properties.parser.classes;
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
