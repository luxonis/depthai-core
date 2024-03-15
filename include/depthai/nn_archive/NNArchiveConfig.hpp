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
#include "depthai/nn_archive/v1/Config.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchiveConfig {
   public:
    /**
     * @data Should point to a whole compressed NNArchive read to memory if compression is not set to RAW_FS.
     * If compression is set to RAW_FS, then this should point to just the config.json file read to memory.
     */
    explicit NNArchiveConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * @path Should point to:
     * 1) if compression is set to RAW_FS: to just the config.json file.
     * 2) if compression is set to AUTO: to whole compressed NNArchive or just the config.json file which must end in .json .
     * 3) else: to whole compressed NNArchive.
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveConfig(const dai::Path& path, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * all parameters should point to a whole compressed NNArchive
     * see NNArchive class for parameter explanation
     */
    NNArchiveConfig(const std::function<int()>& openCallback,
                    const std::function<void(std::vector<uint8_t>& buffer)>& readCallback,
                    const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                    const std::function<int64_t(int64_t request)>& skipCallback,
                    const std::function<int()>& closeCallback,
                    NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    std::optional<dai::nn_archive::v1::Config> getConfigV1() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
