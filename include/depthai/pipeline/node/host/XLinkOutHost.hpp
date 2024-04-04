#pragma once

#include "depthai/pipeline/HostNode.hpp"
#include "pipeline/Node.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
namespace dai {
namespace node {
class XLinkOutHost : public NodeCRTP<HostNode, XLinkOutHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;

   public:
    constexpr static const char* NAME = "XLinkInHost";
    // Input in{*this, "in", Input::Type::SReceiver, true, 4, {{DatatypeEnum::Buffer, true}}};
    Input in{*this, {.name = "in", .types = {{DatatypeEnum::Buffer, true}}}};
    // XLinkOutHost(std::shared_ptr<XLinkConnection> conn, const std::string& streamName);
    void setStreamName(const std::string& name);
    void setConnection(std::shared_ptr<XLinkConnection> conn);
    void run() override;
};
}  // namespace node
}  // namespace dai