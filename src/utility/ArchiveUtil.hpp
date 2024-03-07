// C++ std
#include <vector>

// libraries
#include "archive.h"
#include "archive_entry.h"

// internal
#include "depthai/pipeline/node/DetectionNetwork.hpp"

namespace dai::utility {

// Wrapper C++ class for correct exception handling without memory leaks
class ArchiveUtil {
   public:
    explicit ArchiveUtil(struct archive* archivePtr);
    ArchiveUtil(const std::string& filepath, dai::node::DetectionNetwork::NNArchiveFormat format);
    struct archive* getA();
    std::vector<uint8_t> readEntry(struct archive_entry* entry);
    ArchiveUtil(const ArchiveUtil&) = delete;
    ArchiveUtil& operator=(const ArchiveUtil&) = delete;
    ArchiveUtil(ArchiveUtil&&) = delete;
    ArchiveUtil& operator=(ArchiveUtil&&) = delete;
    ~ArchiveUtil();

   private:
    struct archive* aPtr = nullptr;
};

}  // namespace dai::utility
