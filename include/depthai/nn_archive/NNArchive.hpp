#pragma once
#include <cstdint>
#include <depthai/common/ModelType.hpp>
#include <filesystem>
#include <memory>
#include <optional>

#include "depthai/device/Device.hpp"  // For platform enum
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
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
    // ...
};

class NNArchive {
   public:
    /**
     * @brief Construct a new NNArchive object - a container holding a model and its configuration
     *
     * @param archivePath: Path to the archive file
     * @param options: Archive options such as compression, number of shaves, etc. See NNArchiveOptions.
     */
    NNArchive(const std::filesystem::path& archivePath, NNArchiveOptions options = {});

    /**
     * @brief Return a SuperVINO::Blob from the archive if getModelType() returns BLOB, nothing otherwise
     *
     * @return std::optional<OpenVINO::Blob>: Model blob
     */
    std::optional<OpenVINO::Blob> getBlob() const;

    /**
     * @brief Return a SuperVINO::SuperBlob from the archive if getModelType() returns SUPERBLOB, nothing otherwise
     *
     * @return std::optional<OpenVINO::SuperBlob>: Model superblob
     */
    std::optional<OpenVINO::SuperBlob> getSuperBlob() const;

    /**
     * @brief Return a model from the archive if getModelType() returns OTHER or DLC, nothing otherwise
     *
     * @return std::optional<std::vector<uint8_t>>: Model
     */
    std::optional<std::vector<uint8_t>> getOtherModelFormat() const;

    /**
     * @brief Get NNArchive config wrapper
     *
     * @return NNArchiveVersionedConfig: Archive config
     */
    const NNArchiveVersionedConfig& getVersionedConfig() const;

    /**
     * @brief Get inputSize of the model
     * @param index: Index of input
     * @note this function is only valid for models with NCHW and NHWC input formats
     * @return std::vector<std::pair<int, int>>: inputSize
     */
    std::optional<std::pair<uint32_t, uint32_t>> getInputSize(uint32_t index = 0) const;

    /**
     * @brief Get inputWidth of the model
     * @param index: Index of input
     * @return int: inputWidth
     */
    std::optional<uint32_t> getInputWidth(uint32_t index = 0) const;

    /**
     * @brief Get inputHeight of the model
     * @param index: Index of input
     * @return int: inputHeight
     */
    std::optional<uint32_t> getInputHeight(uint32_t index = 0) const;

    /**
     * @brief Get supported platforms
     *
     * @return std::vector<dai::Platform>: Supported platforms
     */
    std::vector<dai::Platform> getSupportedPlatforms() const;
    /**
     * @brief Get NNArchive config.
     *
     * @tparam T: Type of config to get
     * @return const T&: Config
     */
    template <typename T>
    const T& getConfig() const {
        return getVersionedConfig().getConfig<T>();
    }

    /**
     * @brief Get type of model contained in NNArchive
     *
     * @return model::ModelType: type of model in archive
     */
    model::ModelType getModelType() const;

   private:
    // Read model from archive
    std::vector<uint8_t> readModelFromArchive(const std::filesystem::path& archivePath, const std::string& modelPathInArchive) const;

    model::ModelType modelType;
    NNArchiveOptions archiveOptions;

    // Archive config
    std::shared_ptr<NNArchiveVersionedConfig> archiveVersionedConfigPtr;

    // Blob related stuff
    std::shared_ptr<OpenVINO::Blob> blobPtr;

    // Superblob related stuff
    std::shared_ptr<OpenVINO::SuperBlob> superblobPtr;

    // Other formats
    std::shared_ptr<std::vector<uint8_t>> otherModelFormatPtr;
};

}  // namespace dai
