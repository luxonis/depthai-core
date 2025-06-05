#include "depthai/nn_archive/NNArchive.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"

// internal private
#include "common/ModelType.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

NNArchive::NNArchive(const Path& archivePath, NNArchiveOptions options) : archiveOptions(options) {
    // Make sure archive exits
    if(!std::filesystem::exists(archivePath)) DAI_CHECK_V(false, "Archive file does not exist: {}", archivePath.string());

    // Read config
    archiveVersionedConfigPtr.reset(new NNArchiveVersionedConfig(archivePath, archiveOptions.compression()));

    // Only V1 config is supported at the moment
    DAI_CHECK(archiveVersionedConfigPtr->getVersion() == NNArchiveConfigVersion::V1, "Only V1 config is supported at the moment");
    std::string modelPathInArchive = archiveVersionedConfigPtr->getConfig<nn_archive::v1::Config>().model.metadata.path;

    // Read archive type
    modelType = model::readModelType(modelPathInArchive);

    // Unpack model
    unpackArchiveInDirectory(archivePath, (std::filesystem::path(archiveOptions.extractFolder()) / std::filesystem::path(archivePath).filename()).string());
    unpackedModelPath = (std::filesystem::path(archiveOptions.extractFolder()) / std::filesystem::path(archivePath).filename() / modelPathInArchive).string();

    switch(modelType) {
        case model::ModelType::BLOB:
            blobPtr.reset(new OpenVINO::Blob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case model::ModelType::SUPERBLOB:
            superblobPtr.reset(new OpenVINO::SuperBlob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            break;  // Just do nothing, model is already unpacked
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<OpenVINO::Blob> NNArchive::getBlob() const {
    switch(modelType) {
        case model::ModelType::BLOB:
            return *blobPtr;
            break;
        case model::ModelType::SUPERBLOB:
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            return std::nullopt;
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<OpenVINO::SuperBlob> NNArchive::getSuperBlob() const {
    switch(modelType) {
        case model::ModelType::SUPERBLOB:
            return *superblobPtr;
            break;
        case model::ModelType::BLOB:
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
            return std::nullopt;
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<std::string> NNArchive::getModelPath() const {
    switch(modelType) {
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
        case model::ModelType::BLOB:
        case model::ModelType::SUPERBLOB:
            return unpackedModelPath;
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

model::ModelType NNArchive::getModelType() const {
    return modelType;
}

const NNArchiveVersionedConfig& NNArchive::getVersionedConfig() const {
    return *archiveVersionedConfigPtr;
}

std::vector<uint8_t> NNArchive::readModelFromArchive(const Path& archivePath, const std::string& modelPathInArchive) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
    std::vector<uint8_t> modelBytes;
    const bool success = archive.readEntry(modelPathInArchive, modelBytes);
    DAI_CHECK_V(success, "No model {} found in NNArchive {} | Please check your NNArchive.", modelPathInArchive, archivePath);
    return modelBytes;
}

void NNArchive::unpackArchiveInDirectory(const Path& archivePath, const Path& directory) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
    archive.unpackArchiveInDirectory(directory);
}

std::optional<std::pair<uint32_t, uint32_t>> NNArchive::getInputSize(uint32_t index) const {
    auto inputs = archiveVersionedConfigPtr->getConfig<nn_archive::v1::Config>().model.inputs;
    if(inputs.size() <= index) {
        throw std::runtime_error("Input index in NNArchive is out of bounds");
    }

    auto input = inputs[index];
    auto layout = input.layout;
    if(!layout) {
        return std::nullopt;
    }
    if((*layout != "NCHW" && *layout != "NHWC") || input.shape.size() != 4) {
        return std::nullopt;
    }

    for(auto& dim : input.shape) {
        if(dim < 0) {
            throw std::runtime_error("Input shape must be positive in NNArchive");
        }
    }
    if(*layout == "NCHW") {
        uint32_t width = input.shape[3];
        uint32_t height = input.shape[2];
        return std::make_pair(width, height);
    }
    if(*layout == "NHWC") {
        uint32_t width = input.shape[2];
        uint32_t height = input.shape[1];
        return std::make_pair(width, height);
    }

    // Should never happen
    throw std::runtime_error("Unknown layout in NNArchive");
}

std::optional<uint32_t> NNArchive::getInputWidth(uint32_t index) const {
    auto size = getInputSize(index);
    if(size) {
        return size->first;
    }
    return std::nullopt;
}

std::optional<uint32_t> NNArchive::getInputHeight(uint32_t index) const {
    auto size = getInputSize(index);
    if(size) {
        return size->second;
    }
    return std::nullopt;
}

std::vector<dai::Platform> NNArchive::getSupportedPlatforms() const {
    auto pathToModel = getModelPath();
    if(!pathToModel) {
        return {};
    }
    auto pathToModelChecked = *pathToModel;
    // Check if .dlc - in that case add RVC4 to supported platforms
    if(pathToModelChecked.substr(pathToModelChecked.size() - 4) == ".dlc") {
        return {Platform::RVC4};
    }
    if(pathToModelChecked.substr(pathToModelChecked.size() - 10) == ".superblob") {
        return {Platform::RVC2};
    }
    if(pathToModelChecked.substr(pathToModelChecked.size() - 5) == ".blob") {
        // Check if it's a blob for RVC3 or RVC2
        auto model = OpenVINO::Blob(pathToModelChecked);
        if(model.device == OpenVINO::Device::VPUX) {
            return {Platform::RVC3};
        }
        if(model.device == OpenVINO::Device::VPU) {
            return {Platform::RVC2};
        }
        // Should never get here
        return {};
    }
    return {};
}

}  // namespace dai