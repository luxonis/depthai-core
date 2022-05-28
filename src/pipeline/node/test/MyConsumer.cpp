#include "depthai/pipeline/node/test/MyConsumer.hpp"

namespace dai {
namespace node {
namespace test {

MyConsumer::MyConsumer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : MyConsumer(par, nodeId, std::make_unique<MyConsumer::Properties>()) {}
MyConsumer::MyConsumer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<ThreadedNode, MyConsumer, XLinkOutProperties>(par, nodeId, std::move(props)) {
    setInputRefs(&input);
    hostNode = true;
}

}  // namespace test
}  // namespace node
}  // namespace dai
