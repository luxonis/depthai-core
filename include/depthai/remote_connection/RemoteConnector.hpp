#pragma once

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <memory.h>

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
namespace dai {
class RemoteConnector {
   public:
    // Constructor and Destructor
    RemoteConnector() = default;
    ~RemoteConnector() = default;

    // Initializes the WebSocket server
    void initServer(const std::string& address = "0.0.0.0", uint16_t port = 8765);

   private:
    std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> server;
};
}  // namespace dai