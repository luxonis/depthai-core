#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include <magic_enum/magic_enum.hpp>
#include <optional>
#include <stdexcept>

#include "capabilities/ImgFrameCapability.hpp"
#include "common/ModelType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "nn_archive/NNArchive.hpp"
#include "openvino/BlobReader.hpp"
#include "openvino/OpenVINO.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

NeuralNetwork::~NeuralNetwork() = default;

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(Node::Output& input, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);
    input.link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const std::shared_ptr<Camera>& input,
                                                    const Model& model,
                                                    std::optional<float> fps,
                                                    std::optional<dai::ImgResizeMode> resizeMode) {
    ImgFrameCapability cap;
    if(fps.has_value()) cap.fps.value = *fps;
    if(resizeMode.has_value()) cap.resizeMode = *resizeMode;

    return build(input, model, cap);
}

std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const std::shared_ptr<Camera>& input, const Model& model, const ImgFrameCapability& capability) {
    decodeModel(model);
    ImgFrameCapability cap = getFrameCapability(*nnArchive, capability);
    auto* camInput = input->requestOutput(cap, false);
    DAI_CHECK_V(camInput != nullptr, "Camera does not have output with requested capabilities");
    camInput->link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
std::shared_ptr<NeuralNetwork> NeuralNetwork::build(const std::shared_ptr<ReplayVideo>& input, const Model& model, std::optional<float> fps) {
    decodeModel(model);

    ImgFrameCapability cap;
    if(fps.has_value()) cap.fps.value = *fps;
    cap = getFrameCapability(*nnArchive, cap);
    input->setOutFrameType(cap.type.value());
    if(fps.has_value()) {
        input->setFps(*fps);
    }
    input->setSize(std::get<std::pair<unsigned int, unsigned int>>(cap.size.value.value()));
    input->out.link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
}
#endif

void NeuralNetwork::decodeModel(const Model& model) {
    std::optional<NNArchive> nnArchive;

    if(const auto* s = std::get_if<std::string>(&model)) {
        NNModelDescription description;
        description.model = *s;
        nnArchive = createNNArchive(description);
    } else if(const auto* desc = std::get_if<NNModelDescription>(&model)) {
        NNModelDescription tmpDesc = *desc;
        nnArchive = createNNArchive(tmpDesc);
    } else if(const auto* archive = std::get_if<NNArchive>(&model)) {
        nnArchive = *archive;
    }

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to NeuralNetwork::build");
    setNNArchive(*nnArchive);
}

ImgFrameCapability NeuralNetwork::getFrameCapability(const NNArchive& nnArchive, std::optional<ImgFrameCapability> expectedCapability) {
    const auto& nnArchiveCfg = nnArchive.getVersionedConfig();

    DAI_CHECK_V(nnArchiveCfg.getVersion() == NNArchiveConfigVersion::V1, "Only V1 configs are supported for NeuralNetwork.build method");
    auto platform = getDevice()->getPlatform();
    auto supportedPlatforms = nnArchive.getSupportedPlatforms();
    bool platformSupported = std::find(supportedPlatforms.begin(), supportedPlatforms.end(), platform) != supportedPlatforms.end();
    DAI_CHECK_V(platformSupported, "Platform not supported by the neural network model");

    const auto& configV1 = nnArchiveCfg.getConfig<nn_archive::v1::Config>();
    // Check if the model has multiple inputs
    DAI_CHECK_V(configV1.model.inputs.size() == 1, "Model has multiple inputs, it has to be linked manually");

    auto inputHeight = nnArchive.getInputHeight();
    auto inputWidth = nnArchive.getInputWidth();
    if(!inputHeight.has_value() || !inputWidth.has_value()) {
        DAI_CHECK_V(false, "Input height and width not specified in the model");
    }

    auto type = ImgFrame::Type::BGR888p;
    auto inputType = configV1.model.inputs[0].preprocessing.daiType;
    if(inputType.has_value()) {
        auto convertedInputType = magic_enum::enum_cast<ImgFrame::Type>(inputType.value());
        if(!convertedInputType.has_value()) {
            DAI_CHECK_V(false, "Unsupported input type: {}", inputType.value());
        }
        type = convertedInputType.value();
    } else {
        if(platform == Platform::RVC2 || platform == Platform::RVC3) {
            type = ImgFrame::Type::BGR888p;
        } else if(platform == Platform::RVC4) {
            type = ImgFrame::Type::BGR888i;
        } else {
            DAI_CHECK_V(false, "Unsupported platform");
        }
    }
    auto cap = ImgFrameCapability();
    cap.size.value = std::pair(*inputWidth, *inputHeight);
    cap.type = type;

    if(expectedCapability.has_value()) {
        const auto& expected = *expectedCapability;
        if(expected.fps.value.has_value()) {
            cap.fps.value = expected.fps.value;
        }
        cap.resizeMode = expected.resizeMode;
        if(expected.enableUndistortion.has_value()) {
            cap.enableUndistortion = expected.enableUndistortion;
        }
    }
    return cap;
}

NNArchive NeuralNetwork::createNNArchive(NNModelDescription& modelDesc) {
    // Download model from zoo
    if(modelDesc.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set.");
        modelDesc.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = model::readModelType(path);
    DAI_CHECK(modelType == model::ModelType::NNARCHIVE,
              "Model from zoo is not NNArchive - it needs to be a NNArchive to use build(Camera, NNModelDescription, float) method");
    auto nnArchive = NNArchive(path);
    setNNArchive(nnArchive);
    return nnArchive;
}

std::optional<std::reference_wrapper<const NNArchive>> NeuralNetwork::getNNArchive() const {
    if(nnArchive) return std::cref(*nnArchive);
    return std::nullopt;
}

void NeuralNetwork::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;
    this->nnArchive = nnArchive;
    switch(nnArchive.getModelType()) {
        case model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
            setNNArchiveOther(nnArchive);
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void NeuralNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    switch(nnArchive.getModelType()) {
        case model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, numShaves);
            break;
        case model::ModelType::BLOB:
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
            DAI_CHECK_V(false, "NNArchive type is not SUPERBLOB. Use setNNArchive(const NNArchive& nnArchive) instead.");
            break;
        case model::ModelType::NNARCHIVE:
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
    DAI_CHECK_V(nnArchive.getModelType() == model::ModelType::BLOB, "NNArchive type is not BLOB");
    OpenVINO::Blob blob = *nnArchive.getBlob();
    setBlob(blob);
}

void NeuralNetwork::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    OpenVINO::Blob blob = nnArchive.getSuperBlob()->getBlobWithNumShaves(numShaves);
    setBlob(blob);
}

void NeuralNetwork::setNNArchiveOther(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == model::ModelType::DLC || nnArchive.getModelType() == model::ModelType::OTHER, "NNArchive type is not DLC or OTHER");
    DAI_CHECK_V(nnArchive.getOtherModelFormat().has_value(), "Expected model format for DLC/OTHER type");
    setOtherModelFormat(std::move(nnArchive.getOtherModelFormat().value()));
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const std::filesystem::path& path) {
    setBlob(OpenVINO::Blob(path));
}

void NeuralNetwork::setBlob(const std::filesystem::path& path) {
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
    auto asset = assetManager.set("__blob", std::move(blob.data));
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = static_cast<uint32_t>(asset->data.size());
    properties.modelSource = Properties::ModelSource::BLOB;
}

void NeuralNetwork::setOtherModelFormat(std::vector<uint8_t> otherModel) {
    auto asset = assetManager.set("__model", std::move(otherModel));
    properties.modelUri = asset->getRelativeUri();
    properties.modelSource = Properties::ModelSource::CUSTOM_MODEL;
}

void NeuralNetwork::setOtherModelFormat(const std::filesystem::path& path) {
    auto modelAsset = assetManager.set("__model", path);
    properties.modelUri = modelAsset->getRelativeUri();
    properties.modelSource = Properties::ModelSource::CUSTOM_MODEL;
}

void NeuralNetwork::setModelPath(const std::filesystem::path& modelPath) {
    switch(model::readModelType(modelPath)) {
        case model::ModelType::BLOB:
            setBlob(OpenVINO::Blob(modelPath));
            break;
        case model::ModelType::SUPERBLOB:
            setBlob(OpenVINO::SuperBlob(modelPath).getBlobWithNumShaves(8));
            break;
        case model::ModelType::NNARCHIVE:
            setNNArchive(NNArchive(modelPath));
            break;
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            setOtherModelFormat(modelPath);
            break;
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

void NeuralNetwork::setModelFromDeviceZoo(DeviceModelZoo model) {
    properties.deviceModel = model;
}

}  // namespace node
}  // namespace dai
