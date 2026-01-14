#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node {
class Display : public dai::NodeCRTP<ThreadedHostNode, Display> {
   private:
    std::string name;

   public:
    /**
     * Construct a display node with an optional window name.
     */
    explicit Display(std::string name = "Display");
    Input input{*this, {}};
    void run() override;
};
}  // namespace node
}  // namespace dai
