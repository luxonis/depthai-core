#include <string>

#include "depthai/pipeline/Pipeline.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Record.hpp"
    #include "depthai/pipeline/node/host/Replay.hpp"
#endif

namespace dai {
namespace utility {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
bool setupHolisticRecord(Pipeline& pipeline, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames);
bool setupHolisticReplay(Pipeline& pipeline,
                         std::string replayPath,
                         const std::string& mxId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::string>& outFilenames);
#endif

}  // namespace utility
}  // namespace dai
