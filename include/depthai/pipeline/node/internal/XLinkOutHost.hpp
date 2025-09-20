#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#include "depthai/utility/export.hpp"

namespace dai {
namespace node {
namespace internal {

class DEPTHAI_API XLinkOutHost : public NodeCRTP<ThreadedHostNode, XLinkOutHost> {
   private:
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;
    std::condition_variable isWaitingForReconnect;
    std::mutex mtx;
    bool isDisconnected = false;
    bool allowResize = false;

   public:
    using NodeCRTP::NodeCRTP;

    XLinkOutHost() = default;
    XLinkOutHost(std::unique_ptr<Properties> props) {}

    constexpr static const char* NAME = "XLinkOutHost";
    // Input in{this, "in", Input::Type::SReceiver, true, 4, {{DatatypeEnum::Buffer, true}}};
    Input in{this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    // XLinkOutHost(std::shared_ptr<XLinkConnection> conn, const std::string& streamName);
    void setStreamName(const std::string& name);
    void setConnection(std::shared_ptr<XLinkConnection> conn);
    void allowStreamResize(bool allow);
    void disconnect();
    void run() override;
};

}  // namespace internal
}  // namespace node
}  // namespace dai
