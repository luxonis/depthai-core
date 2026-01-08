#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {
struct PacketizedData;
namespace node {
namespace internal {

class XLinkInHostTestable;

class XLinkInHost : public NodeCRTP<ThreadedHostNode, XLinkInHost> {
   private:
    friend XLinkInHostTestable;
    std::shared_ptr<XLinkConnection> conn;
    std::string streamName;
    std::condition_variable isWaitingForReconnect;
    std::mutex mtx;
    bool isDisconnected = false;
    std::unique_ptr<XLinkStream> stream;

    virtual StreamPacketDesc readStreamMessage() const;

    std::shared_ptr<ADatatype> readData() const;
    std::shared_ptr<ADatatype> parsePacketizedData(const std::shared_ptr<PacketizedData>& packetizedData) const;
    void parseMessageGroup(const std::shared_ptr<MessageGroup>& messageGroup) const;

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
