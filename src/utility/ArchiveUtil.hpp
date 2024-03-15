// C++ std
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

    void init(NNArchiveEntry::Compression format);
};

}  // namespace dai::utility
