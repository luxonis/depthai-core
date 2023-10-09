#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace node {

Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Sync(par, nodeId, std::make_unique<Sync::Properties>()) {}
Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Sync, SyncProperties>(par, nodeId, std::move(props)),
      inputs("io", Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}})) {
    setInputMapRefs(&inputs);
    setOutputRefs({&out});
}

}  // namespace node
}  // namespace dai
