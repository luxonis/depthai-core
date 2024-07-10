#include "depthai/nn_archive/NNArchive.hpp"

#include <iostream>

// internal private
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

NNArchive::NNArchive(const std::string& archivePath, NNArchiveOptions options) : archiveOptions(options) {
    // Make sure archive exits
    if(!std::filesystem::exists(archivePath)) DAI_CHECK_V(false, "Archive file does not exist: {}", archivePath);

    // Read config
    archiveConfigPtr.reset(new NNArchiveConfig(archivePath, archiveOptions.compression));

    // Based on the config, read model path in archive
    std::string modelPathInArchive = archiveConfigPtr->getConfigV1()->model.metadata.path;

    // Read archive type
    archiveType = readArchiveType(modelPathInArchive);

    // Read model from archive
    std::vector<uint8_t> modelBytes = readModelFromArchive(archivePath, modelPathInArchive);

    switch(archiveType) {
        case NNArchiveType::BLOB:
            blobPtr.reset(new OpenVINO::Blob(modelBytes));
            if(archiveOptions.numShaves() > 0) {
                std::cout << "Number of shaves is set to " << archiveOptions.numShaves()
                          << " but it will be ignored for a blob archive. Loaded blob compiled for " << blobPtr->numShaves << " shaves." << std::endl;
            }
            break;
        case NNArchiveType::SUPERBLOB:
            DAI_CHECK_V(archiveOptions.numShaves() > 0,
                        "Number of shaves uninitialized ({}) for a superblob archive. Pass NNArchiveOptions to the NNArchive's with desired number of "
                        "shaves set. The number must be positive.",
                        archiveOptions.numShaves());
            superblobPtr.reset(new OpenVINO::SuperBlob(modelBytes));
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
            return superblobPtr->getBlobWithNumShaves(archiveOptions.numShaves());
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

void NNArchive::optionsSetNumberOfShaves(int numShaves) {
    archiveOptions.numShaves(numShaves);
}

NNArchiveType NNArchive::readArchiveType(const std::string& modelPathInArchive) const {
    auto endsWith = [](const std::string& path, const std::string& ending) {
        if(ending.size() > path.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), path.rbegin());
    };

    if(endsWith(modelPathInArchive, ".blob")) return NNArchiveType::BLOB;
    if(endsWith(modelPathInArchive, ".superblob")) return NNArchiveType::SUPERBLOB;

    DAI_CHECK_V(false, "Unknown model path extension: {}", modelPathInArchive);
}

std::vector<uint8_t> NNArchive::readModelFromArchive(const std::string& archivePath, const std::string& modelPathInArchive) const {
    utility::ArchiveUtil archive(archivePath, archiveOptions.compression);
    std::vector<uint8_t> modelBytes;
    const bool success = archive.readEntry(modelPathInArchive, modelBytes);
    DAI_CHECK_V(success, "No model {} found in NNArchive {} | Please check your NNArchive.", modelPathInArchive, archivePath);
    return modelBytes;
}

}  // namespace dai
