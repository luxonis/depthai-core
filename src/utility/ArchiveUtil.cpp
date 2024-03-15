#include "ArchiveUtil.hpp"

#include "utility/ErrorMacros.hpp"
#include "utility/Logging.hpp"

namespace dai::utility {

ArchiveUtil::ArchiveUtil(const std::string& filepath, NNArchiveEntry::Compression format) {
    auto* archivePtr = archive_read_new();
    daiCheckIn(archivePtr);
    aPtr = archivePtr;
    using F = NNArchiveEntry::Compression;
    switch(format) {
        case F::AUTO:
            archive_read_support_filter_all(aPtr);
            archive_read_support_format_all(aPtr);
            break;
        case F::TAR:
            archive_read_support_filter_none(aPtr);
            archive_read_support_format_tar(aPtr);
            break;
        case F::TAR_GZ:
            archive_read_support_filter_gzip(aPtr);
            archive_read_support_format_tar(aPtr);
            break;
        case F::TAR_XZ:
            archive_read_support_filter_xz(aPtr);
            archive_read_support_format_tar(aPtr);
            break;
        case F::RAW_FS:
        default:
            daiCheckIn(false);
            break;
    }
    const auto res = archive_read_open_filename(aPtr, filepath.c_str(), 10240);
    daiCheckV(res == ARCHIVE_OK, "Error when decompressing {}.", filepath);
}

ArchiveUtil::ArchiveUtil(struct archive* archivePtr) {
    daiCheckIn(archivePtr);
    aPtr = archivePtr;
}

struct archive* ArchiveUtil::getA() {
    daiCheckIn(aPtr);
    return aPtr;
}

void ArchiveUtil::readEntry(struct archive_entry* entry, std::vector<uint8_t>& out) {
    daiCheckIn(aPtr);
    out.clear();

    // Read size, 16KiB
    auto readSize = static_cast<int64_t>(16 * 1024);
    if(archive_entry_size_is_set(entry) != 0) {
        // if size is specified, use that for read size
        readSize = archive_entry_size(entry);
    }

    // Record number of bytes actually read
    int64_t finalSize = 0;

    while(true) {
        // Current size, as a offset to write next data to
        auto currentSize = out.size();

        // Resize to accomodate for extra data
        out.resize(currentSize + readSize);
        int64_t size = archive_read_data(aPtr, &out[currentSize], readSize);

        // Check that no errors occurred
        daiCheck(size >= 0, "Errors occured when reading from archive using libarchive.");

        // Append number of bytes actually read to finalSize
        finalSize += size;

        // All bytes were read
        if(size == 0) {
            break;
        }
    }

    // Resize vector to actual read size
    out.resize(finalSize);
}

bool ArchiveUtil::readEntry(const std::string& entryName, std::vector<uint8_t>& out) {
    struct archive_entry* entry = nullptr;
    while(archive_read_next_header(getA(), &entry) == ARCHIVE_OK) {
        std::string curEntryName(archive_entry_pathname(entry));
        if(curEntryName == entryName) {
            readEntry(entry, out);
            return true;
        }
    }
    return false;
}

ArchiveUtil::~ArchiveUtil() {
    if(aPtr != nullptr) {
        const auto res = archive_read_free(aPtr);
        if(res != ARCHIVE_OK) {
            logger::warn("Couldn't free archive memory using libarchive.");
        }
        aPtr = nullptr;
    }
}

}  // namespace dai::utility
