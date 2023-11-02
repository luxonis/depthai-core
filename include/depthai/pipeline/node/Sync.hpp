#pragma once

#include "depthai-shared/properties/SyncProperties.hpp"
#include "depthai/pipeline/Node.hpp"

namespace dai {
namespace node {

class Sync : public NodeCRTP<Node, Sync, SyncProperties> {
   public:
    constexpr static const char* NAME = "Sync";
    Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * TODO
     */
    InputMap inputs;

    /**
     * TODO
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::MessageGroup, false}}};

    /**
     * TODO
     */
    void setSyncIntervalMs(unsigned int syncIntervalMs);

    /**
     * TODO
     */
    void setSyncAttempts(unsigned int syncAttempts);

    /**
     * TODO
     */
    void setNumFramesPool(unsigned int numFramesPool);

    /**
     * TODO
     */
    unsigned int getSyncIntervalMs() const;

    /**
     * TODO
     */
    unsigned int getSyncAttempts() const;

    /**
     * TODO
     */
    unsigned int getNumFramesPool() const;
};

}  // namespace node
}  // namespace dai
