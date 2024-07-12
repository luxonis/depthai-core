#include "ArchiveUtil.hpp"

// c std
#include <cstdio>
#include <filesystem>
#include <optional>

#include "utility/ErrorMacros.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace utility {

void ArchiveUtil::init(NNArchiveEntry::Compression format) {
    auto* archivePtr = archive_read_new();
    DAI_CHECK_IN(archivePtr);
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
            DAI_CHECK_IN(false);
            break;
    }
}

void ArchiveUtil::unpackArchiveInDirectory(const std::filesystem::path directory) {
    struct archive* a = getA();
    struct archive_entry* entry = nullptr;
    std::filesystem::create_directories(directory);
    while(archive_read_next_header(a, &entry) == ARCHIVE_OK && entry != nullptr) {
        const auto entryPath = directory / archive_entry_pathname(entry);
        std::filesystem::remove(entryPath);
        if(archive_entry_filetype(entry) == AE_IFREG) {
            std::vector<uint8_t> data;
            readEntry(entry, data);
            std::ofstream out(entryPath, std::ios::binary);
            out.write(reinterpret_cast<const char*>(data.data()), data.size());
            out.close();
        } else {
            std::filesystem::create_directories(entryPath);
        }
        entry = nullptr;
    }
}

ArchiveUtil::ArchiveUtil(const std::vector<uint8_t>& data, NNArchiveEntry::Compression format) {
    init(format);
    const auto res = archive_read_open_memory(aPtr, data.data(), data.size());
    DAI_CHECK(res == ARCHIVE_OK, "Error when decompressing archive from memory.");
}

ArchiveUtil::ArchiveUtil(const dai::Path& filepath, NNArchiveEntry::Compression format) {
    init(format);
    const auto res = archive_read_open_filename(aPtr, filepath.string().c_str(), 10240);
    DAI_CHECK_V(res == ARCHIVE_OK, "Error when decompressing {}.", filepath);
}

la_ssize_t ArchiveUtil::readCb(struct archive*, void* context, const void** buffer) {
    DAI_CHECK_IN(context);
    auto* cSelf = static_cast<ArchiveUtil*>(context);
    DAI_CHECK_IN(cSelf);
    return cSelf->archiveRead(buffer);
}

int ArchiveUtil::openCb(struct archive*, void* context) {
    DAI_CHECK_IN(context);
    auto* cSelf = static_cast<ArchiveUtil*>(context);
    DAI_CHECK_IN(cSelf);
    return cSelf->archiveOpen();
}

int ArchiveUtil::closeCb(struct archive*, void* context) {
    DAI_CHECK_IN(context);
    auto* cSelf = static_cast<ArchiveUtil*>(context);
    DAI_CHECK_IN(cSelf);
    return cSelf->archiveClose();
}

int ArchiveUtil::archiveOpen() {
    DAI_CHECK_IN(userOpenCallback);
    return (*userOpenCallback)();
}

int ArchiveUtil::archiveClose() {
    DAI_CHECK_IN(userCloseCallback);
    return (*userCloseCallback)();
}

int64_t ArchiveUtil::seekCb(struct archive*, void* context, int64_t offset, int whence) {
    DAI_CHECK_IN(context);
    auto* cSelf = static_cast<ArchiveUtil*>(context);
    DAI_CHECK_IN(cSelf);
    return cSelf->archiveSeek(offset, whence);
}

int64_t ArchiveUtil::archiveSeek(int64_t offset,  // NOLINT(bugprone-easily-swappable-parameters)
                                 int whence       // NOLINT(bugprone-easily-swappable-parameters)
) {
    DAI_CHECK_IN(userSeekCallback);
    std::optional<NNArchiveEntry::Seek> whenceConverted;
    switch(whence) {
        case SEEK_SET:
            whenceConverted = NNArchiveEntry::Seek::SET;
            break;
        case SEEK_CUR:
            whenceConverted = NNArchiveEntry::Seek::CUR;
            break;
        case SEEK_END:
            whenceConverted = NNArchiveEntry::Seek::END;
            break;
        default:
            DAI_CHECK_IN(false);
            break;
    }
    return (*userSeekCallback)(offset, *whenceConverted);
}

int64_t ArchiveUtil::skipCb(struct archive*, void* context, int64_t request) {
    DAI_CHECK_IN(context);
    auto* cSelf = static_cast<ArchiveUtil*>(context);
    DAI_CHECK_IN(cSelf);
    return cSelf->archiveSkip(request);
}

int64_t ArchiveUtil::archiveSkip(int64_t request) {
    DAI_CHECK_IN(userSkipCallback);
    return (*userSkipCallback)(request);
}

ArchiveUtil::ArchiveUtil(const std::function<int()>& openCallback,
                         const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                         const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                         const std::function<int64_t(int64_t request)>& skipCallback,
                         const std::function<int()>& closeCallback,
                         NNArchiveEntry::Compression format)
    : userOpenCallback(openCallback),
      userReadCallback(readCallback),
      userSeekCallback(seekCallback),
      userSkipCallback(skipCallback),
      userCloseCallback(closeCallback) {
    init(format);
    auto res = archive_read_set_callback_data(aPtr, this);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_set_open_callback(aPtr, openCb);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_set_read_callback(aPtr, readCb);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_set_close_callback(aPtr, closeCb);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_set_seek_callback(aPtr, seekCb);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_set_skip_callback(aPtr, skipCb);
    DAI_CHECK_IN(res == ARCHIVE_OK);
    res = archive_read_open1(aPtr);
    DAI_CHECK(res == ARCHIVE_OK, "Couldn't open the archive. Did you provide the correct binary data to the read callback? Did your open callback return 0?");
}

ArchiveUtil::ArchiveUtil(struct archive* archivePtr) {
    DAI_CHECK_IN(archivePtr);
    aPtr = archivePtr;
}

struct archive* ArchiveUtil::getA() {
    DAI_CHECK_IN(aPtr);
    return aPtr;
}

int64_t ArchiveUtil::archiveRead(const void** buffer) {
    DAI_CHECK_IN(userReadCallback);
    userReadBuffer = (*userReadCallback)();
    DAI_CHECK(userReadBuffer, "Don't return nullptr from read callbacks.");
    *buffer = userReadBuffer->data();
    DAI_CHECK(userReadBuffer->size() <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()),
              "You can return at most int64_t MAX data->size() from read callbacks.");
    return static_cast<int64_t>(userReadBuffer->size());
}

void ArchiveUtil::readEntry(struct archive_entry* entry, std::vector<uint8_t>& out) {
    DAI_CHECK_IN(aPtr);
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
        DAI_CHECK(size >= 0, fmt::format("Errors occured when reading from archive using libarchive. Error - {}", size));

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

bool ArchiveUtil::isJsonPath(const Path& path) {
    const auto filepath = path.string();
    const auto pointIndex = filepath.find_last_of('.');
    if(pointIndex != std::string::npos) {
        return filepath.substr(filepath.find_last_of('.') + 1) == "json";
    }
    return false;
}

}  // namespace utility
}  // namespace dai
