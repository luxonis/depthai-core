#pragma once

#include <depthai/pipeline/HostNode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace node {
class SyncedNode : public dai::HostNode {
   private:
    Subnode<dai::node::Sync> sync{*this, "sync"};  // TODO(Morato) - this should optionally run on host OR device
    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::MessageGroup, true}}};

   protected:
    void buildStage1() override;
    void run() override;
   public:
    InputMap& inputs = sync->inputs;
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    virtual std::shared_ptr<Buffer> runOnce(std::shared_ptr<dai::MessageGroup> in) = 0;
};
}  // namespace node
}  // namespace dai