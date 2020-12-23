#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

NeuralNetwork::NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string NeuralNetwork::getName() const {
    return "NeuralNetwork";
}

std::vector<Node::Output> NeuralNetwork::getOutputs() {
    return {out};
}

std::vector<Node::Input> NeuralNetwork::getInputs() {
    return {input};
}

nlohmann::json NeuralNetwork::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> NeuralNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

tl::optional<OpenVINO::Version> NeuralNetwork::getRequiredOpenVINOVersion() {
    return networkOpenvinoVersion;
}

void NeuralNetwork::loadBlob(const std::string& path) {
    // Each Node has its own asset manager

    // Load blob in blobPath into asset
    // And mark in properties where to look for it
    std::ifstream blobStream(blobPath, std::ios::in | std::ios::binary);
    if(!blobStream.is_open()) throw std::runtime_error("NeuralNetwork node | Blob at path: " + blobPath + " doesn't exist");

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

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const std::string& path) {
    blobPath = path;
    loadBlob(path);
}

void NeuralNetwork::setNumPoolFrames(int numFrames) {
    properties.numFrames = numFrames;
}

}  // namespace node
}  // namespace dai
