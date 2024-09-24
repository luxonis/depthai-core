#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include <stdexcept>

#include "common/ModelType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "modelzoo/Zoo.hpp"
#include "nn_archive/NNArchive.hpp"
#include "openvino/BlobReader.hpp"
#include "openvino/OpenVINO.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

std::optional<OpenVINO::Version> NeuralNetwork::getRequiredOpenVINOVersion() {
    return networkOpenvinoVersion;
}

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(Node::Output& input, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);
    input.link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const NNArchiveConfig& nnArchiveConfig, std::shared_ptr<Camera> input, dai::NNModelDescription modelDesc, float fps) {
    return build(NNArchiveVersionedConfig(nnArchiveConfig), input, modelDesc, fps);
}

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const NNArchiveVersionedConfig& nnArchiveCfg, std::shared_ptr<Camera> camera, dai::NNModelDescription modelDesc, float fps){
    setFromModelZoo(modelDesc);

    DAI_CHECK_V(nnArchiveCfg.getVersion() == dai::NNArchiveConfigVersion::V1, "Only V1 configs are supported for NeuralNetwork.build method");

    const auto& configV1 = nnArchiveCfg.getConfig<dai::nn_archive::v1::Config>();

    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Only single input model is supported");
    DAI_CHECK_V(configV1.model.inputs[0].shape.size() == 4, "Only 4D input shape is supported");
    DAI_CHECK_V(configV1.model.inputs[0].shape[0] == 1 && configV1.model.inputs[0].shape[1] == 3, "Only 3 channel input is supported");

    auto inputHeight = configV1.model.inputs[0].shape[2];
    auto inputWidth = configV1.model.inputs[0].shape[3];

    auto type = dai::ImgFrame::Type::BGR888p;
    auto platform = getDevice()->getPlatform();
    if(platform == dai::Platform::RVC2 || platform == dai::Platform::RVC3) {
        type = dai::ImgFrame::Type::BGR888p;
    } else if(platform == dai::Platform::RVC4) {
        type = dai::ImgFrame::Type::BGR888i;
    } else {
        DAI_CHECK_V(false, "Unsupported platform");
    }

    auto cap = ImgFrameCapability();
    cap.size.value = std::pair(inputWidth, inputHeight);
    cap.type = type;
    cap.fps.value = fps;
    auto* input = camera->requestOutput(cap, false);
    DAI_CHECK_V(input != nullptr, "Camera does not have output with requested capabilities");
    input->link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}


void NeuralNetwork::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            setNNArchiveOther(nnArchive);
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void NeuralNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, numShaves);
            break;
        case dai::model::ModelType::BLOB:
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            DAI_CHECK_V(false, "NNArchive type is not SUPERBLOB. Use setNNArchive(const NNArchive& nnArchive) instead.");
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void NeuralNetwork::setFromModelZoo(NNModelDescription description, bool useCached) {
    // Set platform if not set
    if(description.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set. Use setDevice(...) first.");
        description.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(description, useCached);
    setModelPath(path);
}

void NeuralNetwork::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    dai::OpenVINO::Blob blob = *nnArchive.getBlob();
    setBlob(blob);
}

void NeuralNetwork::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    dai::OpenVINO::Blob blob = nnArchive.getSuperBlob()->getBlobWithNumShaves(numShaves);
    setBlob(blob);
}

void NeuralNetwork::setNNArchiveOther(const NNArchive& nnArchive) {
    setModelPath(nnArchive.getModelPath().value());
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const dai::Path& path) {
    setBlob(OpenVINO::Blob(path));
}

void NeuralNetwork::setBlob(const dai::Path& path) {
    setBlobPath(path);
}

void NeuralNetwork::setBlob(OpenVINO::Blob blob) {
    if(device) {
        if(blob.device == OpenVINO::Device::VPUX && device->getPlatform() != Platform::RVC3) {
            throw std::runtime_error(fmt::format("Loaded model is for RVC3, but the device is {}", device->getPlatformAsString()));
        }
        if(blob.device == OpenVINO::Device::VPU && device->getPlatform() != Platform::RVC2) {
            throw std::runtime_error(fmt::format("Loaded model is for RVC2, but the device is {}", device->getPlatformAsString()));
        }
    }
    networkOpenvinoVersion = blob.version;
    auto asset = assetManager.set("__blob", std::move(blob.data));
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = static_cast<uint32_t>(asset->data.size());
    properties.modelSource = Properties::ModelSource::BLOB;
}

void NeuralNetwork::setModelPath(const dai::Path& modelPath) {
    switch(model::readModelType(modelPath.string())) {
        case model::ModelType::BLOB:
            setBlob(OpenVINO::Blob(modelPath.string()));
            break;
        case model::ModelType::SUPERBLOB:
            setBlob(OpenVINO::SuperBlob(modelPath.string()).getBlobWithNumShaves(8));
            break;
        case model::ModelType::NNARCHIVE:
            setNNArchive(NNArchive(modelPath.string()));
            break;
        case model::ModelType::DLC:
        case model::ModelType::OTHER: {
            auto modelAsset = assetManager.set("__model", modelPath);
            properties.modelUri = modelAsset->getRelativeUri();
            properties.modelSource = Properties::ModelSource::CUSTOM_MODEL;
        } break;
    }
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

void NeuralNetwork::setNumShavesPerInferenceThread(int numShavesPerThread) {
    properties.numShavesPerThread = numShavesPerThread;
}

void NeuralNetwork::setBackend(std::string backend) {
    properties.backend = backend;
}

void NeuralNetwork::setBackendProperties(std::map<std::string, std::string> props) {
    properties.backendProperties = props;
}

int NeuralNetwork::getNumInferenceThreads() {
    return properties.numThreads;
}

}  // namespace node
}  // namespace dai
