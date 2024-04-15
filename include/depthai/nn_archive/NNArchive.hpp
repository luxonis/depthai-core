#pragma once

// C std
#include <cstdint>

// C++ std
#include <functional>
#include <vector>

// libraries
#include <spimpl.h>

// internal
#include "depthai/nn_archive/NNArchiveBlob.hpp"
#include "depthai/nn_archive/NNArchiveConfig.hpp"
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchive {
   public:
    /**
     * @data Should point to a whole compressed NNArchive read to memory
     */
    explicit NNArchive(const std::vector<uint8_t>& data, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * @path Filesystem path to a whole compressed NNArchive
     */
    explicit NNArchive(const dai::Path& path, NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);
    /**
     * Parses the NNArchive on the fly via a custom read callback.
     * Useful for remote NNArchives as it can decode data on the fly while downloading and is more memory friendly.
     * If you need to only get the config, to check some fields and then optionally download the blob,
     * prefer the NNArchiveConfig(openCallback, ...)
     * optionally followed by NNArchive(const NNArchiveConfig& config, const NNArchiveBlob& blob)
     */
    NNArchive(
        /**
         * Setup any needed data structures before read callbacks are fired.
         * Return 0 on success.
         */
        const std::function<int()>& openCallback,
        /**
         * Returns the next block of data from the archive.
         * We will manage the memory to keep it alive, so you don't have to keep a reference.
         */
        const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
        /**
         * Seeks to specified location in the file and returns the position.
         * Whence values have same meaning as SEEK_SET, SEEK_CUR, SEEK_END from stdio.h.
         * Throw instance of std::runtime_error if the seek fails for any reason.
         */
        const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
        /**
         * Skips at most request bytes from archive and returns the skipped amount.
         * This may skip fewer bytes than requested; it may even skip zero bytes.
         * If you do skip fewer bytes than requested, we will invoke your
         * read callback and discard data as necessary to make up the full skip.
         */
        const std::function<int64_t(int64_t request)>& skipCallback,
        /**
         * Cleanup any data structures from openCallback and return 0 on success
         */
        const std::function<int()>& closeCallback,
        NNArchiveEntry::Compression compression = NNArchiveEntry::Compression::AUTO);

    NNArchive(const NNArchiveConfig& config, const NNArchiveBlob& blob);

    const NNArchiveConfig& getConfig() const;

    const NNArchiveBlob& getBlob() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
