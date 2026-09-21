#pragma once

#include "depthai/pipeline/DeviceNode.hpp"

namespace dai {
namespace beta {

class BetaNode : public DeviceNode, public HostRunnable {
   public:
    virtual ~BetaNode() = default;

    virtual void setRunOnHost(bool runOnHost) = 0;

   protected:
    using DeviceNode::DeviceNode;

    void buildStage1() override;
};

}  // namespace beta
}  // namespace dai
