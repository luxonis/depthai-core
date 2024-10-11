#include "depthai/remote_connection/RemoteConnection.hpp"

#include <iostream>
#include <websocketpp/common/connection_hdl.hpp>

#include "depthai/pipeline/MessageQueue.hpp"
#include "foxglove/websocket/common.hpp"
#include "utility/Logging.hpp"
#include "utility/Resources.hpp"

namespace dai {

inline static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

RemoteConnection::RemoteConnection(const std::string& address, uint16_t port) {
    initWebsocketServer(address, port);
    initHttpServer(address, 8000);

    // Expose services
    exposeKeyPressedService();
    exposeTopicGroupsService();
}

RemoteConnection::~RemoteConnection() {
    server->stop();
    httpServer->stop();
    httpServerThread->join();
}

int RemoteConnection::waitKey(int delayMs) {
    // Implemented based on opencv's waitKey method
    // https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga5628525ad33f52eab17feebcfba38bd7

    constexpr int NEUTRAL_VALUE = -1;

    std::unique_lock<std::mutex> lock(keyMutex);
    int ret = keyPressed;
    keyPressed = NEUTRAL_VALUE;

    // If key has been pressed already, just return
    if(ret != NEUTRAL_VALUE) {
        return ret;
    }

    // Wait indefinitely if delay_ms is non-positive
    if(delayMs <= 0) {
        keyCv.wait(lock);
    } else {
        keyCv.wait_for(lock, std::chrono::milliseconds(delayMs));
    }

    ret = keyPressed;
    keyPressed = NEUTRAL_VALUE;
    return ret;
}

void RemoteConnection::keyPressedCallback(int key) {
    std::unique_lock<std::mutex> lock(keyMutex);
    keyPressed = key;
    keyCv.notify_all();
}

void RemoteConnection::initWebsocketServer(const std::string& address, uint16_t port) {
    // Create the WebSocket server with a simple log handler
    const auto logHandler = [](foxglove::WebSocketLogLevel, const char* msg) { logger::info(msg); };
    foxglove::ServerOptions serverOptions;
    serverOptions.sendBufferLimitBytes = 100 * 1024 * 1024;  // 100 MB
    serverOptions.capabilities.emplace_back("services");
    serverOptions.supportedEncodings.emplace_back("json");
    // Instantiate the server using the factory
    server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("DepthAI RemoteConnection", logHandler, serverOptions);

    // Add handlers for the server
    foxglove::ServerHandlers<websocketpp::connection_hdl> hdlrs;
    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        logger::info("Client {} subscribed to {}", clientStr, chanId);
    };
    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        logger::info("Client {} unsubscribed from {}", clientStr, chanId);
    };

    hdlrs.serviceRequestHandler = [&](const foxglove::ServiceRequest& request, foxglove::ConnHandle clientHandle) {
        logger::info("Received service request from client {}", server->remoteEndpointString(clientHandle));
        // Check that the service handler is registered
        auto it = serviceMap.find(request.serviceId);
        if(it == serviceMap.end()) {
            server->sendServiceFailure(clientHandle, request.serviceId, request.callId, "Service not found");
            return;
        }
        // Call the service handler
        auto response = it->second(request);
        response.callId = request.callId;
        response.serviceId = request.serviceId;
        server->sendServiceResponse(clientHandle, response);
    };

    server->setHandlers(std::move(hdlrs));
    // Start the server at the specified address and port
    try {
        server->start(address, port);
        logger::info("Server started at {}:{}", address, port);
    } catch(const std::exception& ex) {
        logger::error("Failed to start server: {}", ex.what());
    }
}

void RemoteConnection::addPublishThread(const std::string& topicName, const std::shared_ptr<MessageQueue>& outputQueue, const std::string& group) {
    std::thread([this, topicName, outputQueue, group]() {
        bool isRunning = true;
        // Wait for the first message to extract schema
        auto firstMessage = outputQueue->get();
        if(!firstMessage) {
            logger::error("No message received from the output for topic: {}", topicName);
            return;
        }

        auto serializableMessage = std::dynamic_pointer_cast<utility::ProtoSerializable>(firstMessage);
        if(!serializableMessage) {
            std::cerr << "First message is not a ProtoSerializable message for topic: " << topicName << std::endl;
            return;
        }
        // Assuming the first message is a protobuf message
        const auto descriptor = serializableMessage->serializeSchema();

        // Add the topic to the server
        auto channelId = server->addChannels({{.topic = topicName,
                                               .encoding = "protobuf",
                                               .schemaName = descriptor.schemaName,
                                               .schema = foxglove::base64Encode(descriptor.schema),
                                               .schemaEncoding = std::nullopt}})[0];

        // Store the group information
        if(topicGroups.find(topicName) != topicGroups.end()) {
            logger::error("Topic named {} is already present", topicName);
            return;
        }
        topicGroups[topicName] = group;

        // Start the message forwarding loop
        while(isRunning) {
            std::shared_ptr<ADatatype> message;
            try {
                message = outputQueue->get();
            } catch(const dai::MessageQueue::QueueException& ex) {
                logger::error("Error while getting message from output queue for topic: {}", topicName);
                isRunning = false;
                continue;
            } catch(const std::exception& ex) {
                logger::error("Error while getting message from output queue for topic: {}", topicName);
                isRunning = false;
                continue;
            }
            if(!message) continue;

            auto serializableMessage = std::dynamic_pointer_cast<utility::ProtoSerializable>(message);
            if(!serializableMessage) {
                logger::error("Message is not a ProtoSerializable message for topic: {}", topicName);
                continue;
            }

            auto serializedMsg = serializableMessage->serializeProto();
            server->broadcastMessage(channelId, nanosecondsSinceEpoch(), static_cast<const uint8_t*>(serializedMsg.data()), serializedMsg.size());
        }
    }).detach();  // Detach the thread to run independently
}

void RemoteConnection::addTopic(const std::string& topicName, Node::Output& output, const std::string& group) {
    auto outputQueue = output.createOutputQueue();
    // Start a thread to handle the schema extraction and message forwarding
    addPublishThread(topicName, outputQueue, group);
}

std::shared_ptr<MessageQueue> RemoteConnection::addTopic(const std::string& topicName, const std::string& group, unsigned int maxSize, bool blocking) {
    auto outputQueue = std::make_shared<MessageQueue>(maxSize, blocking);
    addPublishThread(topicName, outputQueue, group);
    return outputQueue;
}

void RemoteConnection::registerPipeline(const Pipeline& pipeline) {
    exposePipelineService(pipeline);
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

void RemoteConnection::initHttpServer(const std::string& address, uint16_t port) {
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
    logger::info("To connect to the DepthAI visualizer, open http://localhost:{} in your browser", port);
    logger::info("In case of a different client, replace 'localhost' with the correct hostname");
    // Run the server in a separate thread
    httpServerThread = std::make_unique<std::thread>([this, address, port]() { httpServer->listen(address, port); });
}

void RemoteConnection::exposeTopicGroupsService() {
    std::vector<foxglove::ServiceWithoutId> services;
    auto topicGroupsService = foxglove::ServiceWithoutId();
    topicGroupsService.name = "topicGroups";
    auto request = foxglove::ServiceRequestDefinition();
    request.schemaName = "topicGroups";
    request.schema = "";
    request.encoding = "json";
    topicGroupsService.request = request;
    topicGroupsService.response = request;
    topicGroupsService.type = "json";
    services.push_back(topicGroupsService);
    auto ids = server->addServices(services);
    assert(ids.size() == 1);
    auto id = ids[0];

    // Add the handler
    serviceMap[id] = [this](foxglove::ServiceResponse request) {
        (void)request;  // Nothing to do with the request
        auto response = foxglove::ServiceResponse();
        nlohmann::json topicGroupsJson = this->topicGroups;
        std::string serializedTopicGroups = topicGroupsJson.dump();
        response.data = std::vector<uint8_t>(serializedTopicGroups.begin(), serializedTopicGroups.end());
        return response;
    };
}

void RemoteConnection::exposeKeyPressedService() {
    std::string serviceName = "keyPressed";
    foxglove::ServiceWithoutId service;
    service.name = serviceName;
    service.type = "json";

    foxglove::ServiceRequestDefinition requestDef;
    requestDef.schemaName = serviceName + "Request";
    requestDef.encoding = "json";
    service.request = requestDef;

    foxglove::ServiceRequestDefinition responseDef;
    responseDef.schemaName = serviceName + "Response";
    responseDef.encoding = "json";
    service.response = responseDef;

    auto ids = server->addServices({service});
    assert(ids.size() == 1);
    auto id = ids[0];
    serviceMap[id] = [this](foxglove::ServiceResponse request) {
        // Create a json object from the request data
        nlohmann::json jsonRequest = nlohmann::json::parse(request.data);
        auto keyPressed = jsonRequest["key"].get<std::string>();
        // Check that the keyPressed is a single character
        if(keyPressed.size() != 1) {
            std::string errorMsg = R"({"error": "Invalid key pressed. Expected a single character"})";
            return foxglove::ServiceResponse{.data = std::vector<uint8_t>(errorMsg.begin(), errorMsg.end())};
        }

        this->keyPressedCallback(static_cast<int>(keyPressed[0]));
        std::string responseMsg = R"({"status": "Received key"})";
        return foxglove::ServiceResponse{.data = std::vector<uint8_t>(responseMsg.begin(), responseMsg.end())};
    };
}

void RemoteConnection::exposePipelineService(const Pipeline& pipeline) {
    // Add the service
    std::vector<foxglove::ServiceWithoutId> services;
    auto pipelineService = foxglove::ServiceWithoutId();
    pipelineService.name = "pipelineSchema";
    auto request = foxglove::ServiceRequestDefinition();
    request.schemaName = "pipelineSchema";
    request.schema = "";
    request.encoding = "json";
    pipelineService.request = request;
    pipelineService.response = request;
    pipelineService.type = "json";
    services.push_back(pipelineService);
    auto ids = server->addServices(services);
    assert(ids.size() == 1);
    auto id = ids[0];

    // Add the handler
    auto serializedPipeline = pipeline.serializeToJson();
    auto serializedPipelineStr = serializedPipeline.dump();
    serviceMap[id] = [serializedPipelineStr](foxglove::ServiceResponse request) {
        (void)request;  // Nothing to do with the request
        auto response = foxglove::ServiceResponse();
        response.data = std::vector<uint8_t>(serializedPipelineStr.begin(), serializedPipelineStr.end());
        return response;
    };
}

}  // namespace dai
