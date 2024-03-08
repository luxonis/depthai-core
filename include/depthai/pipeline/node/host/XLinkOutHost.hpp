#pragma once

#include "depthai/pipeline/HostNode.hpp"
namespace dai {
class XLinkOutHost : dai::HostNode {
   public:
    Input in{*this, "in", Input::Type::SReceiver, true, 4, {{DatatypeEnum::Buffer, true}}};
    XLinkOutHost(std::shared_ptr<XLinkConnection> conn, const std::string& streamName);
    void run() override;
    void stop() override;
};
}  // namespace dai