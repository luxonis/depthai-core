#pragma once
#include <cstdint>
#include <memory>

#include "depthai/nn_archive/NNArchiveConfig.hpp"
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/arg.hpp"

namespace dai {

struct NNArchiveOptions {
    // General parameters
    DEPTAHI_ARG_DEFAULT(NNArchiveEntry::Compression, compression, NNArchiveEntry::Compression::AUTO);

    // Blob parameters
    // ...

    // Superblob parameters
    // ...

    // Parameters for other formats, ONNX, PT, etc..
    DEPTAHI_ARG_DEFAULT(std::string, extractFolder, "/tmp/");
};

enum class NNArchiveType { BLOB, SUPERBLOB, OTHER };

class NNArchive {
   public:
    /**
     * @brief Construct a new NNArchive object - a container holding a model and its configuration
     *
     * @param archivePath: Path to the archive file
     * @param options: Archive options such as compression, number of shaves, etc. See NNArchiveOptions.
     */
    NNArchive(const std::string& archivePath, NNArchiveOptions options = {});

    /**
     * @brief Return a SuperVINO::Blob from the archive if getArchiveType() returns BLOB, nothing otherwise
     *
     * @return std::optional<OpenVINO::Blob>: Model blob
     */
    std::optional<OpenVINO::Blob> getBlob() const;

    /**
     * @brief Return a SuperVINO::SuperBlob from the archive if getArchiveType() returns SUPERBLOB, nothing otherwise
     *
     * @return std::optional<OpenVINO::SuperBlob>: Model superblob
     */
    std::optional<OpenVINO::SuperBlob> getSuperBlob() const;

    /**
     * @brief Return a path to the model inside the archive if getArchiveType() returns OTHER, nothing otherwise
     *
     * @return std::optional<std::string>: Model path
     */
    std::optional<std::string> getModelPath() const;

    /**
     * @brief Get NNArchive config, i.e. contents of `config.json` inside the archive.
     *
     * @return NNArchiveConfig: Archive config
     */
    const NNArchiveConfig& getConfig() const;

    /**
     * @brief Return loaded archive type. Can be either BLOB or SUPERBLOB.
     *
     * @return NNArchiveType: Archive type
     */
    NNArchiveType getArchiveType() const;

   private:
    // Helper functions:
    // Check what kind of archive is read
    static NNArchiveType readArchiveType(const std::string& modelPathInArchive);

    // Read model from archive
    std::vector<uint8_t> readModelFromArchive(const std::string& archivePath, const std::string& modelPathInArchive) const;

    // Unpack archive to tmp directory
    void unpackArchiveInDirectory(const std::string& archivePath, const std::string& directory) const;

    NNArchiveType archiveType;
    NNArchiveOptions archiveOptions;

    // Archive config
    std::shared_ptr<NNArchiveConfig> archiveConfigPtr;

    // Blob related stuff
    std::shared_ptr<OpenVINO::Blob> blobPtr;

    // Superblob related stuff
    std::shared_ptr<OpenVINO::SuperBlob> superblobPtr;

    // Other formats - return path to the unpacked archive
    std::string unpackedModelPath;
};

}  // namespace dai
