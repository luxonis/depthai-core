#if defined(__clang__)
    #include "depthai/pipeline/ThreadedHostNode.hpp"
#endif

namespace dai {
namespace node {

#if defined(__clang__)
ThreadedHostNode::~ThreadedHostNode() = default;
#endif

}  // namespace node
}  // namespace dai