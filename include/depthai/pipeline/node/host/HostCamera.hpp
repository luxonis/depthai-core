#pragma once

#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

#include "depthai/utility/export.hpp"

namespace dai {
namespace node {
class DEPTHAI_API HostCamera : public dai::NodeCRTP<ThreadedHostNode, HostCamera> {
   public:
    HostCamera() = default;
    HostCamera(std::unique_ptr<Properties> props) {}

    Output out{this, {DEFAULT_NAME, DEFAULT_GROUP, DEFAULT_TYPES}};
    void run() override;
};
}  // namespace node
}  // namespace dai