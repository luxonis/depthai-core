#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace node {

Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Sync(par, nodeId, std::make_unique<Sync::Properties>()) {}
Sync::Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Sync, SyncProperties>(par, nodeId, std::move(props)),
      inputs("inputs", Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}})) {
    setInputMapRefs(&inputs);
    setOutputRefs({&out});
}

void Sync::setSyncThresholdMs(unsigned int syncIntervalMs) {
    properties.syncIntervalMs = syncIntervalMs;
}

void Sync::setSyncAttempts(int syncAttempts) {
    properties.syncAttempts = syncAttempts;
}

void Sync::setNumFramesPool(unsigned int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

unsigned int Sync::getSyncThresholdMs() const {
    return properties.syncIntervalMs;
}

int Sync::getSyncAttempts() const {
    return properties.syncAttempts;
}

unsigned int Sync::getNumFramesPool() const {
    return properties.numFramesPool;
}

}  // namespace node
}  // namespace dai
