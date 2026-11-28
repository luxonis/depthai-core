#include "depthai/nn_archive/NNArchive.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <unordered_set>

#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"

// internal private
#include "common/ModelType.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/Platform.hpp"

namespace dai {

NNArchive::NNArchive(const std::filesystem::path& archivePath, NNArchiveOptions options) : archiveOptions(std::move(options)) {
    // Make sure archive exits
    if(!std::filesystem::exists(archivePath)) DAI_CHECK_V(false, "Archive file does not exist: {}", archivePath);

    // Read config
    archiveVersionedConfigPtr.reset(new NNArchiveVersionedConfig(archivePath, archiveOptions.compression()));

    // Only V1 config is supported at the moment
    DAI_CHECK(archiveVersionedConfigPtr->getVersion() == NNArchiveConfigVersion::V1, "Only V1 config is supported at the moment");
    std::string modelPathInArchive = archiveVersionedConfigPtr->getConfig<nn_archive::v1::Config>().model.metadata.path;

    // Read archive type
    modelType = model::readModelType(modelPathInArchive);

    switch(modelType) {
        case model::ModelType::BLOB:
            blobPtr.reset(new OpenVINO::Blob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case model::ModelType::SUPERBLOB:
            superblobPtr.reset(new OpenVINO::SuperBlob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case model::ModelType::DLC:
        case model::ModelType::OTHER:
            otherModelFormatPtr = std::make_shared<std::vector<uint8_t>>(readModelFromArchive(archivePath, modelPathInArchive));
            break;
        case model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "NNArchive inside NNArchive is not supported. Please unpack the inner archive first.");
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }

    const auto& dependencies = getConfig<nn_archive::v1::Config>().model.runtimeDependencies;
    if(dependencies) {
        DAI_CHECK(modelType == model::ModelType::DLC, "NNArchive runtime dependencies require an RVC4 SNPE DLC model");
        for(const auto& package : dependencies->snpeUdo) {
            runtimeLibraries->try_emplace(package.registration);
            for(const auto& implementation : package.implementations) runtimeLibraries->try_emplace(implementation.path);
        }
        utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
        std::unordered_set<std::string> found;
        struct archive_entry* entry = nullptr;
        int status = ARCHIVE_OK;
        // ARCHIVE_WARN reports a usable header, for example a name that could not be converted.
        while((status = archive_read_next_header(archive.getA(), &entry)) == ARCHIVE_OK || status == ARCHIVE_WARN) {
            // config.json declares UTF-8 paths; the locale form is the fallback for names that do not convert.
            const char* entryPath = archive_entry_pathname_utf8(entry);
            if(entryPath == nullptr) entryPath = archive_entry_pathname(entry);
            // An entry without a name cannot be a declared library; a missing library is reported below.
            if(entryPath == nullptr) continue;
            const std::string path = entryPath;
            const auto library = runtimeLibraries->find(path);
            if(library == runtimeLibraries->end()) continue;
            DAI_CHECK_V(found.insert(path).second, "Duplicate NNArchive runtime library: {}", path);
            DAI_CHECK_V(archive_entry_filetype(entry) == AE_IFREG && archive_entry_hardlink(entry) == nullptr,
                        "NNArchive runtime library must be a regular file: {}",
                        path);
            archive.readEntry(entry, library->second);
            DAI_CHECK_V(!library->second.empty(), "Empty NNArchive runtime library: {}", path);
        }
        const auto* error = archive_error_string(archive.getA());
        DAI_CHECK_V(status == ARCHIVE_EOF, "Failed reading NNArchive runtime libraries: {}", error ? error : "unknown error");
        for(const auto& library : *runtimeLibraries) {
            DAI_CHECK_V(found.count(library.first) != 0, "Missing NNArchive runtime library: {}", library.first);
        }
    }
}

const std::map<std::string, std::vector<uint8_t>>& NNArchive::getRuntimeLibraries() const {
    return *runtimeLibraries;
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

std::optional<std::vector<uint8_t>> NNArchive::getOtherModelFormat() const {
    switch(modelType) {
        case model::ModelType::OTHER:
        case model::ModelType::DLC:
            return *otherModelFormatPtr;
        case model::ModelType::BLOB:
        case model::ModelType::SUPERBLOB:
            return std::nullopt;
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

std::vector<uint8_t> NNArchive::readModelFromArchive(const std::filesystem::path& archivePath, const std::string& modelPathInArchive) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
    std::vector<uint8_t> modelBytes;
    const bool success = archive.readEntry(modelPathInArchive, modelBytes);
    DAI_CHECK_V(success, "No model {} found in NNArchive {} | Please check your NNArchive.", modelPathInArchive, archivePath);
    return modelBytes;
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

std::optional<ImgResizeMode> NNArchive::getInputResizeMode(uint32_t index) const {
    const auto& inputs = archiveVersionedConfigPtr->getConfig<nn_archive::v1::Config>().model.inputs;
    if(inputs.size() <= index) {
        throw std::runtime_error("Input index in NNArchive is out of bounds");
    }

    const auto& resizeMode = inputs[index].preprocessing.resizeMode;
    if(!resizeMode) {
        return std::nullopt;
    }
    if(*resizeMode == "CROP") {
        return ImgResizeMode::CROP;
    }
    if(*resizeMode == "STRETCH") {
        return ImgResizeMode::STRETCH;
    }
    if(*resizeMode == "LETTERBOX") {
        return ImgResizeMode::LETTERBOX;
    }
    throw std::runtime_error("Unsupported preprocessing resize_mode in NNArchive: " + *resizeMode);
}

std::vector<dai::Platform> NNArchive::getSupportedPlatforms() const {
    switch(modelType) {
        case model::ModelType::DLC:
            return {Platform::RVC4};

        case model::ModelType::SUPERBLOB:
            return {Platform::RVC2};

        case model::ModelType::BLOB:
            if(blobPtr->device == OpenVINO::Device::VPUX) {
                return {Platform::RVC3};
            }
            if(blobPtr->device == OpenVINO::Device::VPU) {
                return {Platform::RVC2};
            }
            // Should never get here
            return {};

        case model::ModelType::NNARCHIVE:
        case model::ModelType::OTHER:
        default:
            return {};
    }
}

dai::nn_archive::v1::Head NNArchive::getHeadConfig(uint32_t headIndex) const {
    const auto& configV1 = getConfig<dai::nn_archive::v1::Config>();
    DAI_CHECK_V(configV1.model.heads, "No heads defined in NNArchive config.");
    DAI_CHECK_V(headIndex < configV1.model.heads->size(), "Head index out of bounds in NNArchive config.");

    return configV1.model.heads->at(headIndex);
}

}  // namespace dai
