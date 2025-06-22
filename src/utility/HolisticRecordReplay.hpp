#include <filesystem>
#include <string>

#include "depthai/pipeline/Pipeline.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Record.hpp"
    #include "depthai/pipeline/node/host/Replay.hpp"
#endif

namespace dai {
namespace utility {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
bool setupHolisticRecord(Pipeline& pipeline,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy = false);
bool setupHolisticReplay(Pipeline& pipeline,
                         std::filesystem::path replayPath,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy = false);
#endif

}  // namespace utility
}  // namespace dai
