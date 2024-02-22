#include "depthai/pipeline/node/MessageDemux.hpp"

namespace dai {
namespace node {

MessageDemux::MessageDemux(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : MessageDemux(par, nodeId, std::make_unique<MessageDemux::Properties>()) {}
MessageDemux::MessageDemux(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, MessageDemux, MessageDemuxProperties>(par, nodeId, std::move(props)),
      outputs("outputs", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})) {
    setInputRefs({&input});
    setOutputMapRefs(&outputs);
}

}  // namespace node
}  // namespace dai
