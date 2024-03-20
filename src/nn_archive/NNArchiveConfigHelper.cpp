#include "NNArchiveConfigHelper.hpp"

// internal private
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace nn_archive {

std::string NNArchiveConfigHelper::getBlobPath(const NNArchiveConfig& config) {
    const auto& maybeConfig = config.getConfigV1();
    DAI_CHECK_IN(maybeConfig);
    return (*maybeConfig).model.metadata.path;
}

}  // namespace nn_archive
}  // namespace dai
