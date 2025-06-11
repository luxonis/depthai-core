#pragma once

// C std
#include <cstdint>

namespace dai {

class NNArchiveEntry {
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
         * The entry isn't compressed. Access it directly on the filesystem.
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
};

}  // namespace dai
