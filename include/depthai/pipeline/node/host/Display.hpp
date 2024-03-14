#pragma once

#include <depthai/pipeline/HostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node{
class Display : public dai::NodeCRTP<dai::HostNode, Display> {
   public:
    Input input{*this};

    void run() override;
};
}  // namespace node
}  // namespace dai