#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkInHost.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "depthai/pipeline/node/internal/XLinkOutHost.hpp"

namespace dai {
namespace utility {

struct XLinkOutBridge {
    std::shared_ptr<node::internal::XLinkOut> xLinkOut;
    std::shared_ptr<node::internal::XLinkInHost> xLinkInHost;
};

struct XLinkInBridge {
    std::shared_ptr<node::internal::XLinkOutHost> xLinkOutHost;
    std::shared_ptr<node::internal::XLinkIn> xLinkIn;
};

class PipelineImplHelper {
    PipelineImpl* pipeline;

   public:
    PipelineImplHelper(PipelineImpl* pipeline) : pipeline(pipeline) {};
    void setupHolisticRecordAndReplay();
    void setupPipelineDebuggingPre();
    void setupPipelineDebuggingPost(std::unordered_map<dai::Node::Output*, XLinkOutBridge>&, std::unordered_map<dai::Node::Input*, XLinkInBridge>&);
};

}  // namespace utility
}  // namespace dai
