#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {
namespace node {
class XLinkInHost : public NodeCRTP<ThreadedHostNode, XLinkInHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;

   public:
    constexpr static const char* NAME = "XLinkInHost";
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::Buffer, true}}}};
    // XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName);
    void setStreamName(const std::string& name);
    void setConnection(std::shared_ptr<XLinkConnection> conn);
    void run() override;
};
}  // namespace node
}  // namespace dai