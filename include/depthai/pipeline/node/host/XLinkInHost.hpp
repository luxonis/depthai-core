#pragma once

#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/Node.hpp"

namespace dai {
namespace node {
class XLinkInHost : public NodeCRTP<HostNode, XLinkInHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;

   public:
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    // XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName);
    void setStreamName(const std::string& name);
    void setConnection(std::shared_ptr<XLinkConnection> conn);
    void run() override;
};
}  // namespace node
}  // namespace dai