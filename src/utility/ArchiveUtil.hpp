// C++ std
#include <vector>

// libraries
#include "archive.h"
#include "archive_entry.h"

// internal
#include "depthai/pipeline/node/DetectionNetwork.hpp"

namespace dai {
namespace utility {

// Wrapper C++ class for correct exception handling without memory leaks
class ArchiveUtil {
   public:
    ArchiveUtil(struct archive* a);
    ArchiveUtil(std::string filepath, const dai::node::DetectionNetwork::NNArchiveFormat format);
    struct archive* getA();
    std::vector<uint8_t> readEntry(struct archive_entry*);
    ~ArchiveUtil();

   private:
    struct archive* aPtr = nullptr;
};

}  // namespace utility
}  // namespace dai
