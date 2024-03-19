#include "depthai/nn_archive/NNArchiveConfig.hpp"

// C++ std
#include <fstream>
#include <optional>
#include <variant>

// libraries
#include <spimpl.h>

#include <nlohmann/json.hpp>

// internal public
#include "depthai/nn_archive/v1/Config.hpp"

// internal private
#include "nn_archive/v1/Generators.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchiveConfig::Impl {
   public:
    // in the future we will have <nn_archive::v1::Config, nn_archive::v2::Config>
    std::optional<std::variant<dai::nn_archive::v1::Config>> mConfig;

    void initConfig(const std::optional<nlohmann::json>& maybeJson) {
        DAI_CHECK_IN(maybeJson);
        dai::nn_archive::v1::Config config;
        dai::nn_archive::v1::from_json(*maybeJson, config);
        mConfig = config;
    }

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {
        bool isJson = compression == NNArchiveEntry::Compression::RAW_FS;
        std::optional<nlohmann::json> maybeJson;
        if(isJson) {
            maybeJson = nlohmann::json::parse(data);
        } else {
            utility::ArchiveUtil archive(data, compression);
            std::vector<uint8_t> jsonBytes;
            const bool success = archive.readEntry("config.json", jsonBytes);
            DAI_CHECK(success, "Didn't find the config.json file inside the NNArchive read from memory.");
            maybeJson = nlohmann::json::parse(jsonBytes);
        }
        initConfig(maybeJson);
    }

    Impl(const Path& path, NNArchiveEntry::Compression compression) {
        bool isJson =
            compression == NNArchiveEntry::Compression::RAW_FS || (compression == NNArchiveEntry::Compression::AUTO && utility::ArchiveUtil::isJsonPath(path));
        std::optional<nlohmann::json> maybeJson;
        if(isJson) {
            std::ifstream jsonStream(path);
            maybeJson = nlohmann::json::parse(jsonStream);
        } else {
            utility::ArchiveUtil archive(path, compression);
            std::vector<uint8_t> jsonBytes;
            const bool success = archive.readEntry("config.json", jsonBytes);
            DAI_CHECK_V(success, "Didn't find the config.json file inside the {} archive.", path);
            maybeJson = nlohmann::json::parse(jsonBytes);
        }
        initConfig(maybeJson);
    }

    Impl(const std::function<int()>& openCallback,
         const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
         const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
         const std::function<int64_t(int64_t request)>& skipCallback,
         const std::function<int()>& closeCallback,
         NNArchiveEntry::Compression compression) {
        std::optional<nlohmann::json> maybeJson;
        if(compression == NNArchiveEntry::Compression::RAW_FS) {
            DAI_CHECK(false, "RAW_FS with callbacks NOT IMPLEMENTED YET for NNArchiveConfig");
        } else {
            utility::ArchiveUtil archive(openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression);
            std::vector<uint8_t> jsonBytes;
            const bool success = archive.readEntry("config.json", jsonBytes);
            DAI_CHECK(success, "Didn't find the config.json file inside the NNArchive.");
            maybeJson = nlohmann::json::parse(jsonBytes);
        }
        initConfig(maybeJson);
    }

    template <typename T>
    std::optional<T> getConfig() const {
        DAI_CHECK_IN(mConfig);
        if(const auto* configPtr(std::get_if<T>(&(*mConfig))); configPtr) {
            return *configPtr;
        }
        return std::nullopt;
    }
};

NNArchiveConfig::NNArchiveConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchiveConfig::NNArchiveConfig(const Path& path, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(path, compression)) {}

NNArchiveConfig::NNArchiveConfig(const std::function<int()>& openCallback,
                                 const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                                 const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                                 const std::function<int64_t(int64_t request)>& skipCallback,
                                 const std::function<int()>& closeCallback,
                                 NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression)) {}

std::optional<nn_archive::v1::Config> NNArchiveConfig::getConfigV1() const {
    return pimpl->getConfig<nn_archive::v1::Config>();
}

}  // namespace dai
