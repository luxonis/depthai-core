#include "depthai/remote_connection/RemoteConnector.hpp"

#include <iostream>
namespace dai {
void RemoteConnector::initServer(const std::string& address, uint16_t port) {
    // Create the WebSocket server with a simple log handler
    const auto logHandler = [](foxglove::WebSocketLogLevel, const char* msg) { std::cout << msg << std::endl; };
    foxglove::ServerOptions serverOptions;

    // Instantiate the server using the factory
    server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("DepthAI RemoteConnector", logHandler, serverOptions);

    // Start the server at the specified address and port
    try {
        server->start(address, port);
        std::cout << "Server started at " << address << ":" << port << std::endl;
    } catch(const std::exception& ex) {
        std::cerr << "Failed to start server: " << ex.what() << std::endl;
    }
}
}  // namespace dai
