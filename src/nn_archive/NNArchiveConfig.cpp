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
#include "nn_archive/v1/helper.hpp"
#include "nn_archive/v1/Generators.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchiveConfig::Impl {
   public:
    // in the future we will have <nn_archive::v1::Config, nn_archive::v2::Config>
    std::variant<dai::nn_archive::v1::Config> mConfig;

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {}

    Impl(const Path& path, NNArchiveEntry::Compression compression) {
        const auto filepath = path.string();
        bool isJson = compression == NNArchiveEntry::Compression::RAW_FS;
        if(compression == NNArchiveEntry::Compression::AUTO) {
            const auto pointIndex = filepath.find_last_of('.');
            if(pointIndex != std::string::npos) {
                isJson = filepath.substr(filepath.find_last_of('.') + 1) == "json";
            }
        }
        std::optional<nlohmann::json> maybeJson;
        nlohmann::json json;
        if(isJson) {
            // std::ifstream jsonStream(path);
            // maybeJson = nlohmann::json::parse(jsonStream);
            daiCheck(false, "Raw json not implemented yet");
        } else {
            utility::ArchiveUtil archive(filepath, compression);
            std::vector<uint8_t> jsonBytes;
            const bool success = archive.readEntry("config.json", jsonBytes);
            daiCheckV(success, "Didn't find the config.json file inside the {} archive.", filepath);
            json = nlohmann::json::parse(jsonBytes);
        }
        dai::nn_archive::v1::Config config;
        dai::nn_archive::v1::from_json(json, config);
        mConfig = config;
    }

    template <typename T>
    std::optional<T> getConfig() const {
        if(const auto* configPtr(std::get_if<T>(&mConfig)); configPtr) {
            return *configPtr;
        }
        return std::nullopt;
    }

    const nn_archive::v1::Head& getHead() const {
        const auto& maybeConfig = getConfig<nn_archive::v1::Config>();
        daiCheckIn(maybeConfig);
        const auto& config = *maybeConfig;

        // TODO(jakgra) is NN Archive valid without this? why is this optional?
        daiCheck(config.model.heads, "Heads array is not defined in the NN Archive config file.");
        // TODO(jakgra) for now get info from heads[0] but in the future correctly support multiple outputs and mapped h  eads
        daiCheckV((*config.model.heads).size() == 1,
                  "There should be exactly one head per model in the NN Archive config file defined. Found {} heads.",
                  (*config.model.heads).size());
        return (*config.model.heads)[0];
    }

    std::string getBlobPath() const {
        const auto& maybeConfig = getConfig<nn_archive::v1::Config>();
        daiCheckIn(maybeConfig);
        return (*maybeConfig).model.metadata.path;
    }
};

NNArchiveConfig::NNArchiveConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchiveConfig::NNArchiveConfig(const Path& path, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(path, compression)) {}

std::optional<nn_archive::v1::Config> NNArchiveConfig::getConfigV1() const {
    return pimpl->getConfig<nn_archive::v1::Config>();
}

std::string NNArchiveConfig::getBlobPath() const {
    return pimpl->getBlobPath();
}

}  // namespace dai
