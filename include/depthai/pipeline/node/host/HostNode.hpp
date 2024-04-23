#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/node/Sync.hpp>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace node {
class HostNode : public ThreadedHostNode {
   private:
    std::optional<bool> syncOnHost;
    Subnode<dai::node::Sync> sync{*this, "sync"};
    // Input input{*this, "in", Input::Type::SReceiver, true, 3, {{DatatypeEnum::MessageGroup, true}}};
    Input input{*this, {.name = "in", .types = {{DatatypeEnum::MessageGroup, true}}}};

   protected:
    void buildStage1() override;
    void run() override;
   public:
    InputMap& inputs = sync->inputs;
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};
    virtual std::shared_ptr<Buffer> _process(std::shared_ptr<dai::MessageGroup> in) = 0;

    void runSyncingOnHost() {
        syncOnHost = true;
    }
    void runSyncingOnDevice() {
        syncOnHost = false;
    }
};
}  // namespace node
}  // namespace dai
