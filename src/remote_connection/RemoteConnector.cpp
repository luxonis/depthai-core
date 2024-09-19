#include "depthai/remote_connection/RemoteConnector.hpp"

#include <iostream>
#include <websocketpp/common/connection_hdl.hpp>

#include "utility/Resources.hpp"

namespace dai {
void RemoteConnector::initWebsocketServer(const std::string& address, uint16_t port) {
    // Create the WebSocket server with a simple log handler
    const auto logHandler = [](foxglove::WebSocketLogLevel, const char* msg) { std::cout << msg << std::endl; };
    foxglove::ServerOptions serverOptions;
    serverOptions.sendBufferLimitBytes = 100 * 1024 * 1024;  // 100 MB

    // Instantiate the server using the factory
    server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("DepthAI RemoteConnector", logHandler, serverOptions);

    // Add handlers for the server
    foxglove::ServerHandlers<websocketpp::connection_hdl> hdlrs;
    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " subscribed to " << chanId << std::endl;
    };
    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " unsubscribed from " << chanId << std::endl;
    };

    server->setHandlers(std::move(hdlrs));
    // Start the server at the specified address and port
    try {
        server->start(address, port);
        std::cout << "Server started at " << address << ":" << port << std::endl;
    } catch(const std::exception& ex) {
        std::cerr << "Failed to start server: " << ex.what() << std::endl;
    }
}

std::string getMimeType(const std::string& path) {
    static std::map<std::string, std::string> mimeTypes = {{".html", "text/html"},
                                                           {".htm", "text/html"},
                                                           {".css", "text/css"},
                                                           {".js", "application/javascript"},
                                                           {".png", "image/png"},
                                                           {".jpg", "image/jpeg"},
                                                           {".jpeg", "image/jpeg"},
                                                           {".gif", "image/gif"},
                                                           {".svg", "image/svg+xml"},
                                                           {".json", "application/json"},
                                                           {".txt", "text/plain"}};
    auto ext = path.substr(path.find_last_of("."));
    auto it = mimeTypes.find(ext);
    if(it != mimeTypes.end()) {
        return it->second;
    }
    return "application/octet-stream";  // Default binary type if no match
};

void RemoteConnector::initHttpServer(const std::string& address, uint16_t port) {
    auto visualizerFs = Resources::getInstance().getEmbeddedVisualizer();
    httpServer = std::make_unique<httplib::Server>();
    // httpServer->set_mount_point("/", "/home/matevz/Downloads/viewer-fe-2");
    httpServer->Get("/(.*)", [visualizerFs](const httplib::Request& req, httplib::Response& res) {
        std::string requestedPath = req.matches[1];

        if(requestedPath.empty() || requestedPath == "/") {
            requestedPath = "index.html";  // Default file
        }
        requestedPath = "index/" + requestedPath;
        auto fileData = visualizerFs.getFile(requestedPath);
        if(fileData) {
            res.set_content(std::string(fileData.value().begin(), fileData.value().end()), getMimeType(requestedPath));
        } else {
            res.status = 404;
            res.set_content("File not found", "text/plain");
        }
    });
    std::cout << "To connect to the DepthAI visualizer, open http://" << address << ":" << port << " in your browser" << std::endl;
    // Run the server in a separate thread
    httpServerThread = std::make_unique<std::thread>([this, address, port]() { httpServer->listen(address, port); });
}
}  // namespace dai
