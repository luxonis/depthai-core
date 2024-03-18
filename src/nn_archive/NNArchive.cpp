#include "depthai/nn_archive/NNArchive.hpp"

// C++ std
#include <utility>

// libraries
#include <spimpl.h>

// internal private
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchive::Impl {
   public:
    NNArchiveConfig mConfig;
    NNArchiveBlob mBlob;

    static NNArchiveBlob blobFromConfig(const NNArchiveConfig& config, const Path& path, NNArchiveEntry::Compression compression) {
        if((compression == NNArchiveEntry::Compression::AUTO && utility::ArchiveUtil::isJsonPath(path)) || compression == NNArchiveEntry::Compression::RAW_FS) {
            const auto filepath = path.string();
#if defined(_WIN32) && defined(_MSC_VER)
            const char separator = '\\';
#else
            const char separator = '/';
#endif
            const auto& configV1 = config.getConfigV1();
            daiCheckIn(configV1);
            const size_t lastSlashIndex = filepath.find_last_of(separator);
            std::string blobPath;
            if(std::string::npos == lastSlashIndex) {
                blobPath = (*configV1).model.metadata.path;
            } else {
                const auto basedir = filepath.substr(0, lastSlashIndex + 1);
                blobPath = basedir + separator + (*configV1).model.metadata.path;
            }
            return NNArchiveBlob(config, blobPath, NNArchiveEntry::Compression::RAW_FS);
        }
        return NNArchiveBlob(config, path, compression);
    }

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
        : mConfig(NNArchiveConfig(data, compression)), mBlob(NNArchiveBlob(mConfig, data, compression)) {}

    Impl(NNArchiveConfig config, NNArchiveBlob blob) : mConfig(std::move(config)), mBlob(std::move(blob)){};

    Impl(const Path& path, NNArchiveEntry::Compression compression)
        : mConfig(NNArchiveConfig(path, compression)), mBlob(blobFromConfig(mConfig, path, compression)) {}

    Impl(const std::function<int()>& openCallback,
         const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
         const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
         const std::function<int64_t(int64_t request)>& skipCallback,
         const std::function<int()>& closeCallback,
         NNArchiveEntry::Compression compression)
        : mConfig(NNArchiveConfig(openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression)),
          mBlob(NNArchiveBlob(mConfig, openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression)) {}

    const NNArchiveBlob& getBlob() const {
        return mBlob;
    }
};

NNArchive::NNArchive(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchive::NNArchive(const NNArchiveConfig& config, const NNArchiveBlob& blob) : pimpl(spimpl::make_impl<Impl>(config, blob)) {}

NNArchive::NNArchive(const Path& path, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(path, compression)) {}

NNArchive::NNArchive(const std::function<int()>& openCallback,
                     const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                     const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                     const std::function<int64_t(int64_t request)>& skipCallback,
                     const std::function<int()>& closeCallback,
                     NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(openCallback, readCallback, seekCallback, skipCallback, closeCallback, compression)) {}

const NNArchiveConfig& NNArchive::getConfig() const {
    return pimpl->mConfig;
}

const NNArchiveBlob& NNArchive::getBlob() const {
    return pimpl->getBlob();
}

}  // namespace dai
