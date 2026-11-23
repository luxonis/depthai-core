#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

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

namespace {
constexpr const char* UDO_ASSET_PREFIX = "__udo/";

// Each model setter replaces the whole model. A blob, a custom model, a device
// zoo model and the UDO libraries of an NNArchive never mix.
void clearModel(AssetManager& assetManager, NeuralNetworkProperties& properties) {
    // Find the libraries by key: the public properties may have been edited.
    for(const auto& asset : assetManager.getAll()) {
        if(asset->key.rfind(UDO_ASSET_PREFIX, 0) == 0) assetManager.remove(asset->key);
    }
    properties.udoPackages.clear();
    properties.udoLibraryUris.clear();
    assetManager.remove("__blob");
    assetManager.remove("__model");
    properties.blobUri.clear();
    properties.blobSize.reset();
    properties.modelUri.clear();
    properties.deviceModel.reset();
}
}  // namespace

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
    decodeModel(model);
    ImgFrameCapability cap = getFrameCapability(*nnArchive);
    if(fps.has_value()) cap.fps.value = *fps;
    if(resizeMode.has_value()) {
        cap.resizeMode = *resizeMode;
    } else {
        cap.resizeMode = nnArchive->getInputResizeMode().value_or(ImgResizeMode::CROP);
    }
    auto* camInput = input->requestOutput(cap, false);
    DAI_CHECK_V(camInput != nullptr, "Camera does not have output with requested capabilities");
    camInput->link(this->input);
    return std::static_pointer_cast<NeuralNetwork>(shared_from_this());
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

ImgFrameCapability NeuralNetwork::getFrameCapability(const NNArchive& nnArchive, const std::optional<ImgFrameCapability>& expectedCapability) {
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
        DAI_CHECK_V(convertedInputType.has_value(), "Unsupported input type: {}", inputType.value());
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
    if(expectedCapability.has_value()) {
        cap = *expectedCapability;
    }
    cap.size.value = std::pair(*inputWidth, *inputHeight);
    cap.type = type;

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
    // Store the archive only once its model is set. The argument may refer to the stored archive.
    const NNArchive archive = nnArchive;
    switch(archive.getModelType()) {
        case model::ModelType::BLOB:
            setNNArchiveBlob(archive);
            break;
        case model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(archive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
            setNNArchiveOther(archive);
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. %s: %s", __FILE__, __LINE__);
            break;
    }
    this->nnArchive = archive;
}

void NeuralNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    // Store the archive only once its model is set. The argument may refer to the stored archive.
    const NNArchive archive = nnArchive;
    switch(archive.getModelType()) {
        case model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(archive, numShaves);
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
    this->nnArchive = archive;
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
    const auto& dependencies = nnArchive.getConfig<nn_archive::v1::Config>().model.runtimeDependencies;
    DAI_CHECK(!dependencies || !device || device->getPlatform() == Platform::RVC4, "NNArchive runtime dependencies require RVC4");

    // The device runs UDOs on the DSP only, so it needs the registration and DSP libraries.
    std::vector<NeuralNetworkProperties::UdoPackage> packages;
    std::set<std::string> paths;
    std::map<std::string, std::string> libraryNames;  // device file name -> first archive path with that name
    const auto& libraries = nnArchive.getRuntimeLibraries();
    const auto addLibrary = [&](const std::string& path) {
        paths.insert(path);
        // The device stores and looks up the libraries by file name only. Paths are
        // validated POSIX paths; std::filesystem would re-encode them on Windows.
        const auto name = path.substr(path.rfind('/') + 1);
        const auto existing = libraryNames.emplace(name, path).first;
        DAI_CHECK_V(existing->second == path || libraries.at(existing->second) == libraries.at(path),
                    "Conflicting NNArchive UDO libraries {} and {}: the device needs distinct file names for different libraries",
                    existing->second,
                    path);
    };
    if(dependencies) {
        for(const auto& package : dependencies->snpeUdo) {
            NeuralNetworkProperties::UdoPackage udoPackage{package.registration, {}};
            for(const auto& implementation : package.implementations) {
                std::string runtime = implementation.runtime;
                std::transform(runtime.begin(), runtime.end(), runtime.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if(runtime == "dsp") udoPackage.dspImplementations.push_back(implementation.path);
            }
            DAI_CHECK_V(
                !udoPackage.dspImplementations.empty(), "NNArchive UDO package {} has no implementation for the SNPE DSP runtime", package.registration);
            addLibrary(udoPackage.registration);
            for(const auto& path : udoPackage.dspImplementations) addLibrary(path);
            packages.push_back(std::move(udoPackage));
        }
    }

    setOtherModelFormat(std::move(nnArchive.getOtherModelFormat().value()));
    for(const auto& path : paths) {
        const auto asset = assetManager.set(UDO_ASSET_PREFIX + path, libraries.at(path));
        properties.udoLibraryUris.emplace(path, asset->getRelativeUri());
    }
    properties.udoPackages = std::move(packages);
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
    clearModel(assetManager, properties);
    auto asset = assetManager.set("__blob", std::move(blob.data));
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = static_cast<uint32_t>(asset->getSize());
    properties.modelSource = Properties::ModelSource::BLOB;
}

void NeuralNetwork::setOtherModelFormat(std::vector<uint8_t> otherModel) {
    clearModel(assetManager, properties);
    auto asset = assetManager.set("__model", std::move(otherModel));
    properties.modelUri = asset->getRelativeUri();
    properties.modelSource = Properties::ModelSource::CUSTOM_MODEL;
}

void NeuralNetwork::setOtherModelFormat(const std::filesystem::path& path) {
    // Check the file first, so a bad path keeps the current model.
    DAI_CHECK_V(std::ifstream(path, std::ios::binary).is_open(), "Cannot load model, file at path {} doesn't exist.", path);
    clearModel(assetManager, properties);
    auto modelAsset = assetManager.setLazy("__model", path);
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

void NeuralNetwork::setBackend(const std::string& backend) {
    properties.backend = backend;
}

void NeuralNetwork::setBackendProperties(const std::map<std::string, std::string>& props) {
    properties.backendProperties = props;
}

int NeuralNetwork::getNumInferenceThreads() const {
    return properties.numThreads;
}

void NeuralNetwork::setModelFromDeviceZoo(DeviceModelZoo model) {
    // The device loads its own model, so do not transfer a previously set one.
    clearModel(assetManager, properties);
    properties.deviceModel = model;
}

}  // namespace node
}  // namespace dai
