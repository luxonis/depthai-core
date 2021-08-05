#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

NeuralNetwork::NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    inputs = {&input};

    outputs = {&out, &passthrough};
}

std::string NeuralNetwork::getName() const {
    return "NeuralNetwork";
}

NeuralNetwork::Properties& NeuralNetwork::getPropertiesRef() {
    return properties;
}

nlohmann::json NeuralNetwork::getProperties() {
    nlohmann::json j;
    Properties& properties = getPropertiesRef();
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> NeuralNetwork::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

tl::optional<OpenVINO::Version> NeuralNetwork::getRequiredOpenVINOVersion() {
    return networkOpenvinoVersion;
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const std::string& path) {
    auto asset = assetManager.set("__blob", path);

    // Read blobs header to determine openvino version
    BlobReader reader;
    reader.parse(asset->data);
    networkOpenvinoVersion = OpenVINO::getBlobLatestSupportedVersion(reader.getVersionMajor(), reader.getVersionMinor());

    NeuralNetworkProperties& properties = getPropertiesRef();
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = asset->data.size();
}

void NeuralNetwork::setNumPoolFrames(int numFrames) {
    Properties& properties = getPropertiesRef();
    properties.numFrames = numFrames;
}

void NeuralNetwork::setNumInferenceThreads(int numThreads) {
    Properties& properties = getPropertiesRef();
    properties.numThreads = numThreads;
}

void NeuralNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    Properties& properties = getPropertiesRef();
    properties.numNCEPerThread = numNCEPerThread;
}

int NeuralNetwork::getNumInferenceThreads() {
    Properties& properties = getPropertiesRef();
    return properties.numThreads;
}

}  // namespace node
}  // namespace dai
