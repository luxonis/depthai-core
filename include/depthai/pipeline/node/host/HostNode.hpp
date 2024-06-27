#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/node/Sync.hpp>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace node {
class HostNode : public ThreadedHostNode {
   private:
    std::optional<bool> syncOnHost;
    bool sendProcessToPipeline = false;
    Subnode<dai::node::Sync> sync{*this, "sync"};
    // Input input{*this, "in", Input::Type::SReceiver, true, 3, {{DatatypeEnum::MessageGroup, true}}};
    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::MessageGroup, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

   protected:
    void buildStage1() override;
    void run() override;

   public:
    InputMap& inputs = sync->inputs;
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    virtual std::shared_ptr<Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) = 0;

    void sendProcessingToPipeline(bool send) {
        sendProcessToPipeline = send;
    }

    void runSyncingOnHost() {
        syncOnHost = true;
    }
    void runSyncingOnDevice() {
        syncOnHost = false;
    }
};

template <typename T>
using CustomNode = NodeCRTP<HostNode, T>;
}  // namespace node
}  // namespace dai
