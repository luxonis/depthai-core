#pragma once

#include <httplib.h>

#include <condition_variable>
#include <depthai/pipeline/Node.hpp>
#include <depthai/utility/Pimpl.hpp>
#include <depthai/utility/ProtoSerializable.hpp>
#include <foxglove/websocket/server_interface.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utility/PimplImpl.hpp"

namespace dai {

class RemoteConnectionImpl {
   public:
    RemoteConnectionImpl(const std::string& address, uint16_t webSocketPort, bool serveFrontend, uint16_t httpPort);
    ~RemoteConnectionImpl();

    void addTopic(const std::string& topicName, Node::Output& output, const std::string& group);
    std::shared_ptr<MessageQueue> addTopic(const std::string& topicName, const std::string& group, unsigned int maxSize, bool blocking);
    void registerPipeline(const Pipeline& pipeline);
    int waitKey(int delayMs);

   private:
     /**
      * @brief Initialize websocket server
      * @param address Address to bind to
      * @param port Port to bind to
      * @return True if successful, false otherwise
      *
     */
    bool initWebsocketServer(const std::string& address, uint16_t port);

    /**
     * @brief Initialize HTTP server
     * @param address Address to bind to
     * @param port Port to bind to
     * @return True if successful, false otherwise
     */
    bool initHttpServer(const std::string& address, uint16_t port);

    void addPublishThread(const std::string& topicName, const std::shared_ptr<MessageQueue>& outputQueue, const std::string& group);
    void exposeTopicGroupsService();
    void exposeKeyPressedService();
    void exposePipelineService(const Pipeline& pipeline);
    void keyPressedCallback(int key);

    std::mutex keyMutex;
    std::condition_variable keyCv;
    int keyPressed = -1;

    std::unordered_map<std::string, std::string> topicGroups;
    std::vector<std::shared_ptr<InputQueue>> inputQueues;
    std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> server;
    std::unique_ptr<httplib::Server> httpServer;
    std::unique_ptr<std::thread> httpServerThread;
    std::vector<std::unique_ptr<std::thread>> publishThreads;
    std::map<foxglove::ServiceId, std::function<foxglove::ServiceResponse(foxglove::ServiceResponse)>> serviceMap;
};

}  // namespace dai
