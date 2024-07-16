#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {
namespace node {
class HostCamera : public dai::NodeCRTP<ThreadedHostNode, HostCamera> {
   public:
    Output out{*this, {DEFAULT_NAME, DEFAULT_GROUP, DEFAULT_TYPES}};
    void run() override;
};
}  // namespace node
}  // namespace dai