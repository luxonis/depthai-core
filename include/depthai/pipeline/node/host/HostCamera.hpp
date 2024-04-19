#pragma once

#include <depthai/pipeline/HostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node {
class HostCamera : public dai::NodeCRTP<dai::HostNode, HostCamera> {
   public:
    Output out{*this, {}};
    void run() override;
};
}  // namespace node
}  // namespace dai