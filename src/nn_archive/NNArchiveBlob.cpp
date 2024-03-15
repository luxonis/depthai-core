#include "depthai/nn_archive/NNArchiveBlob.hpp"

// C++ std
#include <optional>

// libraries
#include <spimpl.h>

// internal private
#include "nn_archive/NNArchiveConfigHelper.hpp"
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchiveBlob::Impl {
   public:
    std::optional<OpenVINO::Blob> mBlob;

    template <typename T>
    void init(const NNArchiveConfig& config, const T& dataOrPath, NNArchiveEntry::Compression compression) {
        if(compression == NNArchiveEntry::Compression::RAW_FS) {
            mBlob.emplace(OpenVINO::Blob(dataOrPath));
        } else {
            const auto& blobPath = nn_archive::NNArchiveConfigHelper::getBlobPath(config);
            utility::ArchiveUtil archive(dataOrPath, compression);
            std::vector<uint8_t> blobBytes;
            const bool success = archive.readEntry(blobPath, blobBytes);
            daiCheckV(success, "No blob {} found in NNArchive. Please check your NNArchive.", blobPath);
            mBlob.emplace(OpenVINO::Blob(blobBytes));
        }
    }

    Impl(const NNArchiveConfig& config, const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {
        init(config, data, compression);
    }

    Impl(const NNArchiveConfig& config, const Path& path, NNArchiveEntry::Compression compression) {
        init(config, path, compression);
    }

    std::optional<std::reference_wrapper<const OpenVINO::Blob>> getBlob() const {
        return mBlob ? std::make_optional(std::ref(*mBlob)) : std::nullopt;
    }
};

NNArchiveBlob::NNArchiveBlob(const NNArchiveConfig& config, const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(config, data, compression)){};

NNArchiveBlob::NNArchiveBlob(const NNArchiveConfig& config, const Path& path, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(config, path, compression)) {}

std::optional<std::reference_wrapper<const OpenVINO::Blob>> NNArchiveBlob::getOpenVINOBlob() const {
    return pimpl->getBlob();
}

}  // namespace dai
