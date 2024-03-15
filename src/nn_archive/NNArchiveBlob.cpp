#include "depthai/nn_archive/NNArchiveBlob.hpp"

// C++ std
#include <iostream>
#include <optional>

// libraries
#include <spimpl.h>

// internal
#include "utility/ArchiveUtil.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

class NNArchiveBlob::Impl {
   public:
    std::optional<OpenVINO::Blob> mBlob;

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {}

    explicit Impl(const std::string& blobPath, const Path& archivePath, NNArchiveEntry::Compression compression) {
        utility::ArchiveUtil archive(archivePath, compression);
        std::vector<uint8_t> blobBytes;
        const bool success = archive.readEntry(blobPath, blobBytes);
        daiCheckV(success, "No blob {} found in NNArchive at path {}. Please check your NNArchive.", blobPath, archivePath);
        mBlob.emplace(OpenVINO::Blob(blobBytes));
    }

    explicit Impl(const Path& path) : mBlob(OpenVINO::Blob(path)) {}

    std::optional<std::reference_wrapper<const OpenVINO::Blob>> getBlob() const {
        return mBlob ? std::make_optional(std::ref(*mBlob)) : std::nullopt;
    }
};

NNArchiveBlob::NNArchiveBlob(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchiveBlob::NNArchiveBlob(const Path& path) : pimpl(spimpl::make_impl<Impl>(path)) {}

NNArchiveBlob::NNArchiveBlob(const std::string& blobPath, const Path& archivePath, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(blobPath, archivePath, compression)) {}

std::optional<std::reference_wrapper<const OpenVINO::Blob>> NNArchiveBlob::getOpenVINOBlob() const {
    return pimpl->getBlob();
}

}  // namespace dai
