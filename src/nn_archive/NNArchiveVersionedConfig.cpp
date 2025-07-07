#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"

// C++ std
#include <fstream>
#include <optional>
#include <variant>

// libraries
#include <nlohmann/json.hpp>

#include "depthai/utility/spimpl.h"

// internal public
#include "depthai/nn_archive/v1/Config.hpp"

// internal private
#include "nn_archive/v1/Generators.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

NNArchiveVersionedConfig::NNArchiveVersionedConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {
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

NNArchiveVersionedConfig::NNArchiveVersionedConfig(const std::filesystem::path& path, NNArchiveEntry::Compression compression) {
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

NNArchiveVersionedConfig::NNArchiveVersionedConfig(const std::function<int()>& openCallback,
                                                   const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                                                   const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                                                   const std::function<int64_t(int64_t request)>& skipCallback,
                                                   const std::function<int()>& closeCallback,
                                                   NNArchiveEntry::Compression compression) {
    std::optional<nlohmann::json> maybeJson;
    if(compression == NNArchiveEntry::Compression::RAW_FS) {
        DAI_CHECK(false, "RAW_FS with callbacks NOT IMPLEMENTED YET for NNArchiveVersionedConfig");
    } else {
        utility::ArchiveUtil archive(openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression);
        std::vector<uint8_t> jsonBytes;
        const bool success = archive.readEntry("config.json", jsonBytes);
        DAI_CHECK(success, "Didn't find the config.json file inside the NNArchive.");
        maybeJson = nlohmann::json::parse(jsonBytes);
    }
    initConfig(maybeJson);
}

void NNArchiveVersionedConfig::initConfig(const std::optional<nlohmann::json>& maybeJson) {
    DAI_CHECK_IN(maybeJson);

    // TODO: Check what version of config this is - for now we will fall back to V1
    dai::nn_archive::v1::Config configV1;
    dai::nn_archive::v1::from_json(*maybeJson, configV1);
    config = configV1;

    // TODO: This should be loaded from the json
    version = NNArchiveConfigVersion::V1;
}

}  // namespace dai
