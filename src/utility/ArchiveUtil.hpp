// C++ std
#include <functional>
#include <optional>
#include <string>
#include <vector>

// libraries
#include "archive.h"
#include "archive_entry.h"

// internal public
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/utility/Path.hpp"

namespace dai::utility {

// Wrapper C++ class for correct exception handling without memory leaks
class ArchiveUtil {
   public:
    explicit ArchiveUtil(struct archive* archivePtr);
    ArchiveUtil(const std::string& filepath, NNArchiveEntry::Compression format);
    ArchiveUtil(const std::vector<uint8_t>& data, NNArchiveEntry::Compression format);
    ArchiveUtil(const std::function<int()>& openCallback,
                const std::function<std::shared_ptr<std::vector<uint8_t>>()>& readCallback,
                const std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>& seekCallback,
                const std::function<int64_t(int64_t request)>& skipCallback,
                const std::function<int()>& closeCallback,
                NNArchiveEntry::Compression format);

    struct archive* getA();
    // Throws on error
    void readEntry(struct archive_entry* entry, std::vector<uint8_t>& out);
    // Reads entryName from archive to curEntry member.
    // Returns true if entry found, false otherwise.
    // Throws on other errors
    bool readEntry(const std::string& entryName, std::vector<uint8_t>& out);
    ArchiveUtil(const ArchiveUtil&) = delete;
    ArchiveUtil& operator=(const ArchiveUtil&) = delete;
    ArchiveUtil(ArchiveUtil&&) = delete;
    ArchiveUtil& operator=(ArchiveUtil&&) = delete;
    ~ArchiveUtil();

    static bool isJsonPath(const Path& path);

   private:
    struct archive* aPtr = nullptr;
    std::shared_ptr<std::vector<uint8_t>> userReadBuffer;
    std::optional<std::function<int()>> userOpenCallback;
    std::optional<std::function<std::shared_ptr<std::vector<uint8_t>>()>> userReadCallback;
    std::optional<std::function<int64_t(int64_t offset, NNArchiveEntry::Seek whence)>> userSeekCallback;
    std::optional<std::function<int64_t(int64_t request)>> userSkipCallback;
    std::optional<std::function<int()>> userCloseCallback;

    void init(NNArchiveEntry::Compression format);
    static int openCb(struct archive*, void* context);
    int archiveOpen();
    static int64_t readCb(struct archive*, void* context, const void** buffer);
    int64_t archiveRead(const void** buffer);
    static int64_t seekCb(struct archive*, void* context, int64_t offset, int whence);
    int64_t archiveSeek(int64_t offset, int whence);
    static int64_t skipCb(struct archive*, void* context, int64_t request);
    int64_t archiveSkip(int64_t request);
    static int closeCb(struct archive*, void* context);
    int archiveClose();
};

}  // namespace dai::utility
