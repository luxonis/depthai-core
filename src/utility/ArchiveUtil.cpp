#include "ArchiveUtil.hpp"

#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace utility {

ArchiveUtil::ArchiveUtil(std::string filepath, const dai::node::DetectionNetwork::NNArchiveFormat format) {
    const auto a = archive_read_new();
    daiCheckIn(a);
    aPtr = a;
    using F = dai::node::DetectionNetwork::NNArchiveFormat;
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
            daiCheckIn(false);
            break;
    }
    auto rc = archive_read_open_filename(aPtr, filepath.c_str(), 10240);
    daiCheckV(rc == ARCHIVE_OK, "Error when decompressing {}.", filepath);
}

ArchiveUtil::ArchiveUtil(struct archive* a) {
    daiCheckIn(a);
    aPtr = a;
}

struct archive* ArchiveUtil::getA() {
    daiCheckIn(aPtr);
    return aPtr;
}

std::vector<uint8_t> ArchiveUtil::readEntry(struct archive_entry* entry) {
    daiCheckIn(aPtr);
    std::vector<uint8_t> out;

    // Read size, 16KiB
    std::size_t readSize = 16 * 1024;
    if(archive_entry_size_is_set(entry)) {
        // if size is specified, use that for read size
        readSize = archive_entry_size(entry);
    }

    // Record number of bytes actually read
    long long finalSize = 0;

    while(true) {
        // Current size, as a offset to write next data to
        auto currentSize = out.size();

        // Resize to accomodate for extra data
        out.resize(currentSize + readSize);
        long long size = archive_read_data(aPtr, &out[currentSize], readSize);

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
    return out;
}

ArchiveUtil::~ArchiveUtil() {
    if(aPtr) {
        const auto rc = archive_read_free(aPtr);
        if(rc != ARCHIVE_OK) {
            logger::warn("Couldn't free archive memory using libarchive.");
        }
        aPtr = nullptr;
    }
}

}  // namespace utility
}  // namespace dai
