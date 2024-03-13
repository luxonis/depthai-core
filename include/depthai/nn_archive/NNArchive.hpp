#pragma once

// C std
#include <cstdint>

// C++ std
#include <functional>
#include <optional>
#include <vector>

// internal
#include "depthai/nn_archive_v1/Config.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

class NNArchive {
   public:
    enum class Compression : uint8_t {
        /**
         * Try to guess the file format from the file extension.
         * .json -> RAW_FS
         * everything else use libarchive to guess the format
         * supported formats are: https://github.com/libarchive/libarchive?tab=readme-ov-file#supported-formats
         */
        AUTO = 0,
        /**
         * Raw host filesystem. Just pass the path to the json config file. The BLOB should be located in the same f  older.
         * Only usable in the constructor that takes a filesystem path.
         */
        RAW_FS,
        /**
         * Force libarchive to treat the file as .tar
         */
        TAR,
        /**
         * Force libarchive to treat the file as .tar.gz
         */
        TAR_GZ,
        /**
         * Force libarchive to treat the file as .tar.xz
         */
        TAR_XZ,
    };

    /**
     * Check stdio.h SEEK_SET, SEEK_CUR, SEEK_END for meaning.
     */
    enum class Seek : uint8_t {
        SET = 0,
        CUR,
        END,
    };

    NNArchive();
    explicit NNArchive(std::vector<uint8_t> data, Compression compression = Compression::AUTO);
    explicit NNArchive(const dai::Path& path, Compression compression = Compression::AUTO);
    NNArchive(
        // TODO(jakgra) Check if it's called for reading the archive or only for writting
        const std::function<int()>& openCallback,
        /**
         * Returns the next block of data from the archive.
         */
        const std::function<void(std::vector<uint8_t>& buffer)>& readCallback,
        /**
         * Seeks to specified location in the file and returns the position.
         * Whence values have same meaning as SEEK_SET, SEEK_CUR, SEEK_END from stdio.h.
         * Throw instance of std::error if the seek fails for any reason.
         */
        const std::function<int64_t(int64_t offset, Seek whence)>& seekCallback,
        /**
         * Skips at most request bytes from archive and returns the skipped amount.
         * This may skip fewer bytes than requested; it may even skip zero bytes.
         * If you do skip fewer bytes than requested, we will invoke your
         * read callback and discard data as necessary to make up the full skip.
         */
        const std::function<int64_t(int64_t request)>& skipCallback,
        // TODO(jakgra) Check if it's called for reading the archive or only for writting
        const std::function<int()>& closeCallback,
        Compression compression = Compression::AUTO);

    /**
     * type T can only be dai::nn_archive_v1::Config for now.
     * returns the config object if config is loaded and of type T, std::nullopt otherwise.
     */
    template <typename T>
    std::optional<T> getConfig() const;

    /**
     * type T can only be dai::nn_archive_v1::Config for now.
     * sets the config but doesn't load the blob.
     */
    template <typename T>
    void setConfig(T& config);

    /**
     * returns the blob or nullptr if the blob isn't loaded (yet)
     */
    OpenVINO::Blob* getBlob();

    void setBlob(const OpenVINO::Blob& blob);

   private:
    // TODO(jakgra) move to pimpl
    std::optional<dai::nn_archive_v1::Config> configV1;
    // in the future we will have
    // std::optional<dai::nn_archive_v1::Config> configV2;
};

}  // namespace dai
