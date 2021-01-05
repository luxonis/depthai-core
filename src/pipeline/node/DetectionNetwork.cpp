#include "depthai/pipeline/node/DetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/datatype/DetectionNetworkTypeEnum.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
DetectionNetwork::DetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string DetectionNetwork::getName() const {
    return "DetectionNetwork";
}

std::vector<Node::Input> DetectionNetwork::getInputs() {
    return {input};
}

std::vector<Node::Output> DetectionNetwork::getOutputs() {
    return {out};
}

nlohmann::json DetectionNetwork::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> DetectionNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void DetectionNetwork::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void DetectionNetwork::setConfidenceThreshold(float thresh) {
    properties.confidenceThreshold = thresh;
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void DetectionNetwork::setNNBlobPath(const std::string& path) {
    // Each Node has its own asset manager

    // Load blob in blobPath into asset
    // And mark in properties where to look for it
    std::ifstream blobStream(path, std::ios::in | std::ios::binary);
    if(!blobStream.is_open()) throw std::runtime_error("DetectionNetwork node | Blob at path: " + path + " doesn't exist");

    // Create an asset (alignment 64)
    Asset blobAsset;
    blobAsset.alignment = 64;
    blobAsset.data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(blobStream), {});

    // Read blobs header to determine openvino version
    BlobReader reader;
    reader.parse(blobAsset.data);
    networkOpenvinoVersion = OpenVINO::getBlobLatestSupportedVersion(reader.getVersionMajor(), reader.getVersionMinor());

    // Create asset key
    std::string assetKey = std::to_string(id) + "/blob";

    // set asset (replaces previous asset without throwing)
    assetManager.set(assetKey, blobAsset);

    // Set properties URI to asset:id/blob
    properties.blobUri = std::string("asset:") + assetKey;
    properties.blobSize = blobAsset.data.size();
}

void DetectionNetwork::setNumPoolFrames(int numFrames) {
    properties.numFrames = numFrames;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetDetectionNetwork::MobileNetDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = dntype_mobilenet;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloDetectionNetwork::YoloDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : DetectionNetwork(par, nodeId) {
    properties.nnFamily = dntype_yolo;
}

void YoloDetectionNetwork::setNumClasses(const int numClasses) {
    properties.classes = numClasses;
}

void YoloDetectionNetwork::setCoordinateSize(const int coordinates) {
    properties.coordinates = coordinates;
}

void YoloDetectionNetwork::setAnchors(std::vector<float> anchors) {
    properties.anchors = anchors;
}

void YoloDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.anchorMasks = anchorMasks;
}

void YoloDetectionNetwork::setIouThreshold(float thresh) {
    properties.iouThreshold = thresh;
}

}  // namespace node
}  // namespace dai
