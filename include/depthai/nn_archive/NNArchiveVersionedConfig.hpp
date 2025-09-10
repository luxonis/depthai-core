#pragma once

// C std
#include <cstdint>

// C++ std
#include <filesystem>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <variant>
#include <vector>

// libraries
#include "depthai/utility/spimpl.h"

// internal
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/nn_archive/v1/Config.hpp"

namespace dai {

enum class NNArchiveConfigVersion {
    V1,
};

using NNArchiveConfig = std::variant<dai::nn_archive::v1::Config>;

// This assumes that the config is always stored in the same order as the enum
inline NNArchiveConfigVersion getNNArchiveConfigVersion(const NNArchiveConfig& config) {
    return static_cast<NNArchiveConfigVersion>(config.index());
}

class NNArchiveVersionedConfig {
   public:
    /**
     * @data Should point to a whole compressed NNArchive read to memory if compression is not set to RAW_FS.
     * If compression is set to RAW_FS, then this should point to just the config.json file read to memory.
     */
    explicit NNArchiveVersionedConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    /**
     * @path Should point to:
     * 1) if compression is set to RAW_FS: to just the config.json file.
     * 2) if compression is set to AUTO: to whole compressed NNArchive or just the config.json file which must end in .json .
     * 3) else: to whole compressed NNArchive.
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveVersionedConfig(const std::filesystem::path& path, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    /**
     *  Returned data should be just the config.json if compression == RAW_FS or the whole NNArchive otherwise
     * see NNArchive class for parameter explanation
     */
    NNArchiveVersionedConfig(const std::function<int()>& openCallback,
                             const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                             const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                             const std::function<int64_t(int64_t request)>& skipCallback,
                             const std::function<int()>& closeCallback,
                             NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    /**
     * @brief Construct NNArchiveVersionedConfig from a specific version of a config.
     * @param version: Version of the config.
     */
    NNArchiveVersionedConfig(const NNArchiveConfig& config) : version(getNNArchiveConfigVersion(config)), config(config) {}

    /**
     * @brief Get version of the underlying config.
     */
    NNArchiveConfigVersion getVersion() const {
        return version;
    }

    /**
     * @brief Get stored config cast to a specific version.
     * @tparam T: Config type to cast to.
     */
    template <typename T>
    const T& getConfig() const {
        return std::get<T>(config);
    }

   private:
    void initConfig(const std::optional<nlohmann::json>& maybeJson);

    NNArchiveConfigVersion version;
    NNArchiveConfig config;
};

template <>
inline const NNArchiveConfig& NNArchiveVersionedConfig::getConfig() const {
    return config;
}

}  // namespace dai
