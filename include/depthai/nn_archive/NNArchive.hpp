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
    NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO;

    // Blob parameters
    // ...

    // Superblob parameters
    DEPTAHI_ARG_DEFAULT(int, numShaves, -1);
};

enum class NNArchiveType { BLOB, SUPERBLOB };

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
     * @brief Generate a SuperVINO::Blob from the archive. The return value is std::optional<OpenVINO::SuperBlob> for backward compatibility.
     *
     * @return std::optional<OpenVINO::Blob>: Model blob
     */
    std::optional<OpenVINO::Blob> getBlob() const;

    /**
     * @brief Generate a SuperVINO::SuperBlob from the archive
     *
     * @return OpenVINO::SuperBlob: Model superblob
     */
    const NNArchiveConfig& getConfig() const;

    /**
     * @brief Return loaded archive type. Can be either BLOB or SUPERBLOB.
     *
     * @return NNArchiveType: Archive type
     */
    NNArchiveType getArchiveType() const;

    /**
     * @brief Set number of shaves for the superblob
     *
     * @param numShaves: Number of shaves. Must be greater than 0.
     */
    void optionsSetNumberOfShaves(int numShaves);

   private:
    // Helper functions:
    // Check what kind of archive is read
    NNArchiveType readArchiveType(const std::string& modelPathInArchive) const;

    // Read model from archive
    std::vector<uint8_t> readModelFromArchive(const std::string& archivePath, const std::string& modelPathInArchive) const;

    NNArchiveType archiveType;
    NNArchiveOptions archiveOptions;

    // Archive config
    std::shared_ptr<NNArchiveConfig> archiveConfigPtr;

    // Blob related stuff
    std::shared_ptr<OpenVINO::Blob> blobPtr;

    // Superblob related stuff
    std::shared_ptr<OpenVINO::SuperBlob> superblobPtr;
};

}  // namespace dai
