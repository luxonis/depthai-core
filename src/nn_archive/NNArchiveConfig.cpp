#include "depthai/nn_archive/NNArchiveConfig.hpp"

// C++ std
#include <optional>
#include <variant>

// libraries
#include <spimpl.h>

// internal public
#include "depthai/nn_archive/v1/Config.hpp"

// internal private
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchiveConfig::Impl {
   public:
    // in the future we will have <nn_archive::v1::Config, nn_archive::v2::Config>
    std::variant<dai::nn_archive::v1::Config> mConfig;

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {}

    Impl(const Path& path, NNArchiveEntry::Compression compression) {}

    template <typename T>
    std::optional<T> getConfig() const {
        const auto config = std::get_if<T>(mConfig);
        return config ? config : std::nullopt;
    }

    const nn_archive::v1::Head& getHead() const {
        const auto& maybeConfig = getConfig<nn_archive::v1::Config>();
        daiCheckIn(maybeConfig);
        const auto& config = *maybeConfig;

        // TODO(jakgra) is NN Archive valid without this? why is this optional?
        daiCheck(config.model.heads, "Heads array is not defined in the NN Archive config file.");
        // TODO(jakgra) for now get info from heads[0] but in the future correctly support multiple outputs and mapped h  eads
        daiCheckV((*config.model.heads).size() == 1,
                  "There should be exactly one head per model in the NN Archive config file define  d. Found {} heads.",
                  (*config.model.heads).size());
        return (*config.model.heads)[0];
    }

    const std::string& getBlobPath() const {
        const auto& maybeConfig = getConfig<nn_archive::v1::Config>();
        daiCheckIn(maybeConfig);
        const auto& config = *maybeConfig;
        return config.model.metadata.path;
    }
};

NNArchiveConfig::NNArchiveConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchiveConfig::NNArchiveConfig(const Path& path, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(path, compression)) {}

template <typename T>
std::optional<T> NNArchiveConfig::getConfig() const {
    return pimpl->getConfig<T>();
}

const std::string& NNArchiveConfig::getBlobPath() const {
    return pimpl->getBlobPath();
}

}  // namespace dai
