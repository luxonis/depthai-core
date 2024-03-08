#pragma once

#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/Node.hpp"
namespace dai {
class XLinkInHost : NodeCRTP<HostNode, XLinkInHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;

   public:
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName);
    void run() override;
};
}  // namespace dai