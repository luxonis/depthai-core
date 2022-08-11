#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

NeuralNetwork::NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NeuralNetwork(par, nodeId, std::make_unique<NeuralNetwork::Properties>()) {}
NeuralNetwork::NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, NeuralNetwork, NeuralNetworkProperties>(par, nodeId, std::move(props)),
      inputs("inputs", Input(*this, "", Input::Type::SReceiver, false, 1, true, {{DatatypeEnum::Buffer, true}})),
      passthroughs("passthroughs", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})) {
    setInputRefs({&input});
    setOutputRefs({&out, &passthrough});
    setInputMapRefs({&inputs});
    setOutputMapRefs({&passthroughs});
}

tl::optional<OpenVINO::Version> NeuralNetwork::getRequiredOpenVINOVersion() {
    return networkOpenvinoVersion;
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const dai::Path& path) {
    setBlob(OpenVINO::Blob(path));
}

void NeuralNetwork::setBlob(const dai::Path& path) {
    setBlobPath(path);
}

void NeuralNetwork::setBlob(OpenVINO::Blob blob) {
    this->networkOpenvinoVersion = blob.version;
    auto asset = assetManager.set("__blob", std::move(blob.data));
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = static_cast<uint32_t>(asset->data.size());
}

void NeuralNetwork::setModelPath(const dai::Path& xmlModelPath, const dai::Path& binModelPath) {
    auto xmlAsset = assetManager.set("__xmlModel", xmlModelPath);
    dai::Path localBinModelPath;
    if(!binModelPath.empty()) {  // Path for the bin file IS set
        localBinModelPath = binModelPath;
    } else {  // Path for the bin file IS NOT set
        auto lastDotPos = xmlModelPath.string().find_last_of('.');
        localBinModelPath = xmlModelPath.string().substr(0, lastDotPos) + ".bin";
    }
    auto binAsset = assetManager.set("__binModel", localBinModelPath);
    properties.xmlUri = xmlAsset->getRelativeUri();
    properties.binUri = binAsset->getRelativeUri();
}

void NeuralNetwork::setNumPoolFrames(int numFrames) {
    properties.numFrames = numFrames;
}

void NeuralNetwork::setNumInferenceThreads(int numThreads) {
    properties.numThreads = numThreads;
}

void NeuralNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    properties.numNCEPerThread = numNCEPerThread;
}

int NeuralNetwork::getNumInferenceThreads() {
    return properties.numThreads;
}

}  // namespace node
}  // namespace dai
