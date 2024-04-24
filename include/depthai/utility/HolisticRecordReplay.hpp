#include "depthai/pipeline/Pipeline.hpp"
#include <string>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Record.hpp"
    #include "depthai/pipeline/node/host/Replay.hpp"
#endif

namespace dai {
namespace utility {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
bool setupHolisticRecord(Pipeline& pipeline, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames);
bool setupHolisticReplay(
    Pipeline& pipeline, std::string replayPath, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames);
#else
template <typename... T>
struct dependent_false {
    static constexpr bool value = false;
};
template <typename... T>
bool __attribute__((weak)) setupHolisticRecord(Pipeline& pipeline, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames) {
    // static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    throw std::runtime_error("Library not configured with OpenCV support");
    return false;
}
template <typename... T>
bool __attribute__((weak)) setupHolisticReplay(
    Pipeline& pipeline, std::string replayPath, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames) {
    // static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    throw std::runtime_error("Library not configured with OpenCV support");
    return false;
}
#endif

}  // namespace utility
}  // namespace dai
