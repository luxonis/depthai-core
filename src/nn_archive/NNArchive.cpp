#include "depthai/nn_archive/NNArchive.hpp"

// internal private
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

NNArchive::NNArchive(const std::string& archivePath, NNArchiveOptions options) : archiveOptions(options) {
    // Make sure archive exits
    if(!std::filesystem::exists(archivePath)) DAI_CHECK_V(false, "Archive file does not exist: {}", archivePath);

    // Read config
    archiveConfigPtr.reset(new NNArchiveConfig(archivePath, archiveOptions.compression()));

    // Based on the config, read model path in archive
    std::string modelPathInArchive = archiveConfigPtr->getConfigV1()->model.metadata.path;

    // Read archive type
    archiveType = readArchiveType(modelPathInArchive);

    switch(archiveType) {
        case NNArchiveType::BLOB:
            blobPtr.reset(new OpenVINO::Blob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case NNArchiveType::SUPERBLOB:
            superblobPtr.reset(new OpenVINO::SuperBlob(readModelFromArchive(archivePath, modelPathInArchive)));
            break;
        case NNArchiveType::OTHER:
            unpackArchiveInDirectory(archivePath, std::filesystem::path(archiveOptions.extractFolder()) / std::filesystem::path(archivePath).filename());
            unpackedModelPath = std::filesystem::path(archiveOptions.extractFolder()) / std::filesystem::path(archivePath).filename() / modelPathInArchive;
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<OpenVINO::Blob> NNArchive::getBlob() const {
    switch(archiveType) {
        case NNArchiveType::BLOB:
            return *blobPtr;
            break;
        case NNArchiveType::SUPERBLOB:
        case NNArchiveType::OTHER:
            return std::nullopt;
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<OpenVINO::SuperBlob> NNArchive::getSuperBlob() const {
    switch(archiveType) {
        case NNArchiveType::SUPERBLOB:
            return *superblobPtr;
            break;
        case NNArchiveType::BLOB:
        case NNArchiveType::OTHER:
            return std::nullopt;
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

std::optional<std::string> NNArchive::getModelPath() const {
    switch(archiveType) {
        case NNArchiveType::OTHER:
            return unpackedModelPath;
            break;
        case NNArchiveType::BLOB:
        case NNArchiveType::SUPERBLOB:
            return std::nullopt;
            break;
        default:
            DAI_CHECK(false, "Unknown archive type");
            break;
    }
}

NNArchiveType NNArchive::getArchiveType() const {
    return archiveType;
}

const NNArchiveConfig& NNArchive::getConfig() const {
    return *archiveConfigPtr;
}

NNArchiveType NNArchive::readArchiveType(const std::string& modelPathInArchive) {
    auto endsWith = [](const std::string& path, const std::string& ending) {
        if(ending.size() > path.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), path.rbegin());
    };

    if(endsWith(modelPathInArchive, ".blob")) return NNArchiveType::BLOB;
    if(endsWith(modelPathInArchive, ".superblob")) return NNArchiveType::SUPERBLOB;
    return NNArchiveType::OTHER;
}

std::vector<uint8_t> NNArchive::readModelFromArchive(const std::string& archivePath, const std::string& modelPathInArchive) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
    std::vector<uint8_t> modelBytes;
    const bool success = archive.readEntry(modelPathInArchive, modelBytes);
    DAI_CHECK_V(success, "No model {} found in NNArchive {} | Please check your NNArchive.", modelPathInArchive, archivePath);
    return modelBytes;
}

void NNArchive::unpackArchiveInDirectory(const std::string& archivePath, const std::string& directory) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression());
    archive.unpackArchiveInDirectory(directory);
}

}  // namespace dai