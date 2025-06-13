#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {
namespace node {
namespace internal {

class XLinkInHost : public NodeCRTP<ThreadedHostNode, XLinkInHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;
    std::condition_variable isWaitingForReconnect;
    std::mutex mtx;
    bool isDisconnected = false;

   public:
    constexpr static const char* NAME = "XLinkInHost";
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    // XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName);
    void setStreamName(const std::string& name);
    void setConnection(std::shared_ptr<XLinkConnection> conn);
    void disconnect();
    void run() override;
};

}  // namespace internal
}  // namespace node
}  // namespace dai