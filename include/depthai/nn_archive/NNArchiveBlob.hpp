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
#include "depthai/nn_archive/NNArchiveConfig.hpp"
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchiveBlob {
   public:
    /**
     * @data Should point to just the blob if compression == RAW_FS or the whole NNArchive otherwise
     */
    explicit NNArchiveBlob(const NNArchiveConfig& config,
                           const std::vector<uint8_t>& data,
                           dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
    /**
     * @path Should point to just the blob if compression == RAW_FS or the whole NNArchive otherwise
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveBlob(const NNArchiveConfig& config,
                           const dai::Path& path,
                           dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
    /**
     *  Returned data should be just the blob if compression == RAW_FS or the whole NNArchive otherwise
     * see NNArchive class for parameter explanation
     */
    NNArchiveBlob(const NNArchiveConfig& config,
                  const std::function<int()>& openCallback,
                  const std::function<void(std::vector<uint8_t>& buffer)>& readCallback,
                  const std::function<int64_t(int64_t offset, dai::NNArchiveEntry::Seek whence)>& seekCallback,
                  const std::function<int64_t(int64_t request)>& skipCallback,
                  const std::function<int()>& closeCallback,
                  dai::NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    /**
     * returns the OpenVINO blob or std::nullopt if the blob is in another format
     */
    std::optional<std::reference_wrapper<const OpenVINO::Blob>> getOpenVINOBlob() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
