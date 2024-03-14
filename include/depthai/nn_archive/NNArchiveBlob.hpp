#pragma once

// C std
#include <cstdint>

// C++ std
#include <functional>
#include <variant>
#include <vector>

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
    explicit NNArchiveBlob(std::vector<uint8_t> data, dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
    /**
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveBlob(const dai::Path& path, dai::NNArchiveEntry::Compression compression = dai::NNArchiveEntry::Compression::AUTO);
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
     * returns the blob or nullptr if the blob isn't loaded (yet)
     */
    OpenVINO::Blob* getBlob();

   private:
    // TODO(jakgra) move to pimpl
    std::variant<OpenVINO, Path> blobOrPath;
};

}  // namespace dai
