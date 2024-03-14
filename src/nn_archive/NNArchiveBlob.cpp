#include "depthai/nn_archive/NNArchiveBlob.hpp"

// C++ std
#include <optional>

// libraries
#include <spimpl.h>

namespace dai {

class NNArchiveBlob::Impl {
   public:
    std::optional<OpenVINO::Blob> mBlob;

    Impl(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) {}

    explicit Impl(const std::string& blobName, const Path& path, NNArchiveEntry::Compression compression) {}

    explicit Impl(const Path& path) : mBlob(OpenVINO::Blob(path)) {}

    const OpenVINO::Blob* getBlob() const {
        return mBlob ? &(*mBlob) : nullptr;
    }
};

NNArchiveBlob::NNArchiveBlob(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression) : pimpl(spimpl::make_impl<Impl>(data, compression)){};

NNArchiveBlob::NNArchiveBlob(const Path& path) : pimpl(spimpl::make_impl<Impl>(path)) {}

NNArchiveBlob::NNArchiveBlob(const std::string& blobName, const Path& path, NNArchiveEntry::Compression compression)
    : pimpl(spimpl::make_impl<Impl>(blobName, path, compression)) {}

const OpenVINO::Blob* NNArchiveBlob::getBlob() const {
    return pimpl->getBlob();
}

}  // namespace dai
