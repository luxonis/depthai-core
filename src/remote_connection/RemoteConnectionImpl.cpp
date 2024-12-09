#include "RemoteConnectionImpl.hpp"

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <iostream>
#include <variant>
#include <websocketpp/common/connection_hdl.hpp>

#include "depthai/pipeline/MessageQueue.hpp"
#include "foxglove/websocket/common.hpp"
#include "pipeline/datatype/Buffer.hpp"
#include "pipeline/datatype/ImgAnnotations.hpp"
#include "utility/Logging.hpp"
#include "utility/ProtoSerializable.hpp"
#include "utility/Resources.hpp"

namespace dai {

inline static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

static std::shared_ptr<Buffer> getVisualizableMessage(const std::shared_ptr<Buffer>& message) {
    dai::VisualizeType visualizableMessage = message->getVisualizationMessage();
    // Default behavior is to return a visualizable message, if available, otherwise return the message itself back
    if(auto* imgFrame = std::get_if<std::shared_ptr<ImgFrame>>(&visualizableMessage)) {
        return *imgFrame;
    }
    if(auto* imgAnnotations = std::get_if<std::shared_ptr<ImgAnnotations>>(&visualizableMessage)) {
        return *imgAnnotations;
    }
    return message;
}

RemoteConnectionImpl::RemoteConnectionImpl(const std::string& address, uint16_t webSocketPort, bool serveFrontend, uint16_t httpPort) {
    auto success = initWebsocketServer(address, webSocketPort);
    if(!success) {
        throw std::runtime_error("Failed to initialize websocket server");
    }
    if(serveFrontend) {
        success = initHttpServer(address, httpPort);
        if(!success) {
            throw std::runtime_error("Failed to initialize http server");
        }
    }

    // Expose services
    exposeKeyPressedService();
    exposeTopicGroupsService();
}

RemoteConnectionImpl::~RemoteConnectionImpl() {
    server->stop();
    for(auto& thread : publishThreads) {
        thread->join();
    }
    if(httpServer) {
        httpServer->stop();
    }
    if(httpServerThread && httpServerThread->joinable()) {
        httpServerThread->join();
    }
}

int RemoteConnectionImpl::waitKey(int delayMs) {
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

void RemoteConnectionImpl::keyPressedCallback(int key) {
    std::unique_lock<std::mutex> lock(keyMutex);
    keyPressed = key;
    keyCv.notify_all();
}

bool RemoteConnectionImpl::initWebsocketServer(const std::string& address, uint16_t port) {
    const auto logHandler = [](foxglove::WebSocketLogLevel level, const char* msg) {
        constexpr auto LOGGER_PREFIX = "WebSocketServer";
        auto msgStr = fmt::format("{}: {}", LOGGER_PREFIX, msg);
        switch(level) {
            case foxglove::WebSocketLogLevel::Debug:
                logger::debug(msgStr);
                break;
            case foxglove::WebSocketLogLevel::Info:
                // TODO(Morato) - revisit and potentially pass the levels correctly
                // Right now, the happy path produces errors when the client disconnects
            case foxglove::WebSocketLogLevel::Warn:
            case foxglove::WebSocketLogLevel::Error:
            case foxglove::WebSocketLogLevel::Critical:
            default:
                logger::info(msgStr);
        }
    };
    foxglove::ServerOptions serverOptions;
    serverOptions.sendBufferLimitBytes = 100 * 1024 * 1024;  // 100 MB
    serverOptions.capabilities.emplace_back("services");
    serverOptions.supportedEncodings.emplace_back("json");

    server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("DepthAI RemoteConnection", logHandler, serverOptions);
    if(!server) {
        logger::error("Failed to create server");
        return false;
    }
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
        auto it = serviceMap.find(request.serviceId);
        if(it == serviceMap.end()) {
            server->sendServiceFailure(clientHandle, request.serviceId, request.callId, "Service not found");
            return;
        }
        auto response = it->second(request);
        response.callId = request.callId;
        response.serviceId = request.serviceId;
        server->sendServiceResponse(clientHandle, response);
    };

    server->setHandlers(std::move(hdlrs));
    try {
        server->start(address, port);
        logger::info("Server started at {}:{}", address, port);
        return true;
    } catch(const std::exception& ex) {
        logger::error("Failed to start server: {}", ex.what());
        return false;
    }
}

void RemoteConnectionImpl::addPublishThread(const std::string& topicName,
                                            const std::shared_ptr<MessageQueue>& outputQueue,
                                            const std::string& group,
                                            bool useVisualizationIfAvailable) {
    auto thread = std::make_unique<std::thread>([this, topicName, outputQueue, group, useVisualizationIfAvailable]() {
        bool isRunning = true;
        auto firstMessage = outputQueue->get();
        if(!firstMessage) {
            logger::error("No message received from the output for topic: {}", topicName);
            return;
        }
        if(!std::dynamic_pointer_cast<Buffer>(firstMessage)) {
            logger::error("First message is not a Buffer message for topic: {}", topicName);
            return;
        }
        if(useVisualizationIfAvailable) {
            firstMessage = getVisualizableMessage(std::dynamic_pointer_cast<Buffer>(firstMessage));
        }
        auto serializableMessage = std::dynamic_pointer_cast<ProtoSerializable>(firstMessage);
        if(!serializableMessage) {
            logger::error("First message is not a ProtoSerializable message for topic: {}", topicName);
            return;
        }
        const auto descriptor = serializableMessage->serializeSchema();

        auto channelId = server->addChannels({{topicName, "protobuf", descriptor.schemaName, foxglove::base64Encode(descriptor.schema), std::nullopt}})[0];

        if(topicGroups.find(topicName) != topicGroups.end()) {
            logger::error("Topic named {} is already present", topicName);
            return;
        }
        topicGroups[topicName] = group;

        while(isRunning) {
            std::shared_ptr<ADatatype> message;
            try {
                message = outputQueue->get();
            } catch(const dai::MessageQueue::QueueException& ex) {
                isRunning = false;
                continue;
            } catch(const std::exception& ex) {
                logger::error("Error while getting message from output queue for topic: {}", topicName);
                isRunning = false;
                continue;
            }
            if(!message) continue;
            if(!std::dynamic_pointer_cast<Buffer>(message)) {
                logger::error("First message is not a Buffer message for topic: {}", topicName);
                return;
            }
            if(useVisualizationIfAvailable) {
                message = getVisualizableMessage(std::dynamic_pointer_cast<Buffer>(message));
            }
            auto serializableMessage = std::dynamic_pointer_cast<ProtoSerializable>(message);
            if(!serializableMessage) {
                logger::error("Message is not a ProtoSerializable message for topic: {}", topicName);
                continue;
            }

            auto serializedMsg = serializableMessage->serializeProto();
            server->broadcastMessage(channelId, nanosecondsSinceEpoch(), static_cast<const uint8_t*>(serializedMsg.data()), serializedMsg.size());
        }
    });
    publishThreads.push_back(std::move(thread));
}

void RemoteConnectionImpl::addTopic(const std::string& topicName, Node::Output& output, const std::string& group, bool useVisualizationIfAvailable) {
    auto outputQueue = output.createOutputQueue();
    addPublishThread(topicName, outputQueue, group, useVisualizationIfAvailable);
}

std::shared_ptr<MessageQueue> RemoteConnectionImpl::addTopic(
    const std::string& topicName, const std::string& group, unsigned int maxSize, bool blocking, bool useVisualizationIfAvailable) {
    auto outputQueue = std::make_shared<MessageQueue>(maxSize, blocking);
    addPublishThread(topicName, outputQueue, group, useVisualizationIfAvailable);
    return outputQueue;
}

void RemoteConnectionImpl::registerPipeline(const Pipeline& pipeline) {
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

bool RemoteConnectionImpl::initHttpServer(const std::string& address, uint16_t port) {
    auto visualizerFs = Resources::getInstance().getEmbeddedVisualizer();
    httpServer = std::make_unique<httplib::Server>();
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

    // Bind the server to the address and port
    auto success = httpServer->bind_to_port(address, port);
    if(!success) {
        logger::error("Failed to bind the http server to port {}", port);
        return false;
    }

    std::cout << "To connect to the DepthAI visualizer, open http://localhost:" << port << " in your browser" << std::endl;
    std::cout << "In case of a different client, replace 'localhost' with the correct hostname" << std::endl;
    httpServerThread = std::make_unique<std::thread>([this]() { httpServer->listen_after_bind(); });
    return true;
}

void RemoteConnectionImpl::exposeTopicGroupsService() {
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

    serviceMap[id] = [this](foxglove::ServiceResponse request) {
        (void)request;
        auto response = foxglove::ServiceResponse();
        nlohmann::json topicGroupsJson = this->topicGroups;
        std::string serializedTopicGroups = topicGroupsJson.dump();
        response.data = std::vector<uint8_t>(serializedTopicGroups.begin(), serializedTopicGroups.end());
        return response;
    };
}

void RemoteConnectionImpl::exposeKeyPressedService() {
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
        nlohmann::json jsonRequest = nlohmann::json::parse(request.data);
        auto keyPressed = jsonRequest["key"].get<std::string>();
        if(keyPressed.size() != 1) {
            std::string errorMsg = R"({"error": "Invalid key pressed. Expected a single character"})";
            foxglove::ServiceResponse ret;
            ret.data = std::vector<uint8_t>(errorMsg.begin(), errorMsg.end());
            return ret;
        }

        this->keyPressedCallback(static_cast<int>(keyPressed[0]));
        std::string responseMsg = R"({"status": "Received key"})";
        foxglove::ServiceResponse ret;
        ret.data = std::vector<uint8_t>(responseMsg.begin(), responseMsg.end());
        return ret;
    };
}

void RemoteConnectionImpl::exposePipelineService(const Pipeline& pipeline) {
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

    auto serializedPipeline = pipeline.serializeToJson();
    auto serializedPipelineStr = serializedPipeline.dump();
    serviceMap[id] = [serializedPipelineStr](foxglove::ServiceResponse request) {
        (void)request;
        auto response = foxglove::ServiceResponse();
        response.data = std::vector<uint8_t>(serializedPipelineStr.begin(), serializedPipelineStr.end());
        return response;
    };
}

void RemoteConnectionImpl::registerService(const std::string& serviceName, std::function<nlohmann::json(const nlohmann::json&)> callback) {
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
    serviceMap[id] = [callback](foxglove::ServiceResponse request) {
        nlohmann::json jsonRequest = nlohmann::json::parse(request.data);
        auto response = callback(jsonRequest);
        auto responseStr = response.dump();
        foxglove::ServiceResponse ret;
        ret.data = std::vector<uint8_t>(responseStr.begin(), responseStr.end());
        ret.callId = request.callId;
        ret.serviceId = request.serviceId;
        return ret;
    };
}

}  // namespace dai
