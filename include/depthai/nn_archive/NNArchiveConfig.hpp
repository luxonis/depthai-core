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
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchiveConfig {
   public:
    /**
     * Tries to parse the config from json data.
     * Throws an error if the config data is malformed / unsupported.
     */
    explicit NNArchiveConfig(const std::vector<uint8_t>& configData);

    /**
     * @data Should point to a whole compressed NNArchive read to memory
     * see NNArchive class for parameter explanation
     */
    explicit NNArchiveConfig(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * all parameters should point to a whole compressed NNArchive
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

    /**
     * type T can only be dai::nn_archive::v1::Config for now.
     * returns the config object if config is loaded and of type T, std::nullopt otherwise.
     */
    template <typename T>
    std::optional<T> getConfig() const;

    /**
     * returns the blobName from the config file.
     * Throws if the config is malformed.
     */
    const std::string& getBlobPath() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
