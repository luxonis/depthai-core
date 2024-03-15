#include "depthai/nn_archive/NNArchive.hpp"

// C++ std
#include <utility>

// libraries
#include <spimpl.h>

// internal private
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchive::Impl {
   public:
    NNArchiveConfig mConfig;
    std::optional<NNArchiveBlob> mBlob;

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
        : mConfig(NNArchiveConfig(data, compression)), mBlob(NNArchiveBlob(data, compression)) {}

    Impl(NNArchiveConfig config, NNArchiveBlob blob) : mConfig(std::move(config)), mBlob(std::move(blob)){};

    Impl(const Path& path, NNArchiveEntry::Compression compression)
        : mConfig(NNArchiveConfig(path, compression)), mBlob(NNArchiveBlob(mConfig.getBlobPath(), path, compression)) {}

    const NNArchiveBlob& getBlob() const {
        daiCheckIn(mBlob);
        return *mBlob;
    }
};

NNArchive::NNArchive(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchive::NNArchive(const NNArchiveConfig& config, const NNArchiveBlob& blob) : pimpl(spimpl::make_impl<Impl>(config, blob)) {}

NNArchive::NNArchive(const Path& path, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(path, compression)) {}

const NNArchiveConfig& NNArchive::getConfig() const {
    return pimpl->mConfig;
}

const NNArchiveBlob& NNArchive::getBlob() const {
    return pimpl->getBlob();
}

}  // namespace dai
