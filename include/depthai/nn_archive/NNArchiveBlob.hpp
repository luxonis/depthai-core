#pragma once

// C std
#include <cstdint>

// C++ std
#include <functional>
#include <optional>
#include <vector>

// libraries
#include <spimpl.h>

// internal
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchiveBlob {
   public:
    /**
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveBlob(const std::vector<uint8_t>& data, dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
    /**
     * @blobName Name of the blob file inside the archive. Can be found using NNArchiveConfig.getBlobPath().
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveBlob(const std::string& blobPath,
                           const dai::Path& archivePath,
                           dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
    /**
     * see NNArchive class for parameter explanation
     */
    NNArchiveBlob(const std::function<int()>& openCallback,
                  const std::function<void(std::vector<uint8_t>& buffer)>& readCallback,
                  const std::function<int64_t(int64_t offset, dai::NNArchiveEntry::Seek whence)>& seekCallback,
                  const std::function<int64_t(int64_t request)>& skipCallback,
                  const std::function<int()>& closeCallback,
                  dai::NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * @path filesystem path to valid OpenVINO blob file
     */
    explicit NNArchiveBlob(const dai::Path& path);

    /**
     * returns the blob or std::nullopt if the blob isn't loaded (yet) or the blob isn't an OpenVINO blob
     */
    std::optional<std::reference_wrapper<const OpenVINO::Blob>> getOpenVINOBlob() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
