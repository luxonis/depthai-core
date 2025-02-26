#pragma once

#include <memory>
#include <string>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {

class RemoteConnectionImpl;
class RemoteConnection {
   public:
    static constexpr auto DEFAULT_WEBSOCKET_PORT = 8765;
    static constexpr auto DEFAULT_HTTP_PORT = 8080;
    static constexpr auto DEFAULT_ADDRESS = "0.0.0.0";
    explicit RemoteConnection(const std::string& address = DEFAULT_ADDRESS,
                              uint16_t webSocketPort = DEFAULT_WEBSOCKET_PORT,
                              bool serveFrontend = true,
                              uint16_t httpPort = DEFAULT_HTTP_PORT);
    ~RemoteConnection();

    void addTopic(const std::string& topicName, Node::Output& output, const std::string& group = "", bool useVisualizationIfAvailable = true);
    std::shared_ptr<MessageQueue> addTopic(
        const std::string& topicName, const std::string& group = "", unsigned int maxSize = 16, bool blocking = false, bool useVisualizationIfAvailable = true);
    bool removeTopic(const std::string& topicName);
    void registerPipeline(const Pipeline& pipeline);
    int waitKey(int delayMs);
    void registerService(const std::string& serviceName, std::function<nlohmann::json(const nlohmann::json&)> callback);

   private:
    Pimpl<RemoteConnectionImpl> impl;
};

}  // namespace dai
