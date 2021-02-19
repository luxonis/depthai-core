#include "depthai/pipeline/node/Micropython.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

Micropython::Micropython(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string Micropython::getName() const {
    return "Micropython";
}

std::vector<Node::Output> Micropython::getOutputs() {
    return {};
}

std::vector<Node::Input> Micropython::getInputs() {
    return {input};
}

nlohmann::json Micropython::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> Micropython::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

Micropython::BlobAssetInfo Micropython::loadBlob(const std::string& path) {
    // Each Node has its own asset manager

    // Load blob in blobPath into asset
    // And mark in properties where to look for it
    std::ifstream blobStream(blobPath, std::ios::in | std::ios::binary);
    if(!blobStream.is_open()) throw std::runtime_error("Micropython node | Blob at path: " + blobPath + " doesn't exist");

    // Create an asset (alignment 64)
    Asset blobAsset;
    blobAsset.alignment = 64;
    blobAsset.data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(blobStream), {});

    // Create asset key
    std::string assetKey = std::to_string(id) + "/micropython";

    // set asset (replaces previous asset without throwing)
    assetManager.set(assetKey, blobAsset);

    // Set properties URI to asset:id/blob
    BlobAssetInfo blobInfo;
    blobInfo.uri = std::string("asset:") + assetKey;
    blobInfo.size = blobAsset.data.size();

    return blobInfo;
}




// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void Micropython::setBlobPath(const std::string& path) {
    blobPath = path;
    BlobAssetInfo blobInfo = loadBlob(path);
    properties.blobUri = blobInfo.uri;
    properties.blobSize = blobInfo.size;
}

void Micropython::setNumPoolFrames(int numFrames) {
    properties.numFrames = numFrames;
}

}  // namespace node
}  // namespace dai
