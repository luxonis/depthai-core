// C++ std
#include <string>

// internal public
#include "depthai/nn_archive/NNArchiveConfig.hpp"

namespace dai {
namespace nn_archive {

class NNArchiveConfigHelper {
   public:
    static std::string getBlobPath(const NNArchiveConfig& config);
};

}  // namespace nn_archive
}  // namespace dai
