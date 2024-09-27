#pragma once

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

// clang-format off
#include <depthai/pipeline/Pipeline.hpp> // Pipeline has to be included before httplib for some reason
// clang-format on

#include <httplib.h>
#include <memory.h>

#include <condition_variable>
#include <depthai/pipeline/Node.hpp>
#include <depthai/utility/ProtoSerializable.hpp>
#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "foxglove/websocket/common.hpp"
#include "foxglove/websocket/server_interface.hpp"

namespace dai {
class RemoteConnector {
   public:
    // Constructor
    explicit RemoteConnector(const std::string& address = "0.0.0.0", uint16_t port = 8765);

    // Destructor
    ~RemoteConnector();

    void addTopic(const std::string& topicName, Node::Output& output, const std::string& group = "");

    void registerPipeline(const Pipeline& pipeline);

    int waitKey(int delay_ms);

    // TODO
    // add a function for better low level API
    // Subscribe to events
    // Expose the pipeline
   private:
    // Initializes the WebSocket server
    void initWebsocketServer(const std::string& address, uint16_t port);
    void initHttpServer(const std::string& address, uint16_t port);

    // Expose services
    void exposeTopicGroupsService();
    void exposeKeyPressedService();
    void exposePipelineService(const Pipeline& pipeline);

    // waitKey related
    std::mutex keyMutex;
    std::condition_variable keyCv;
    int keyPressed = -1;
    void keyPressedCallback(int key);

    // Map storing added topics their corresponding groups
    std::unordered_map<std::string, std::string> topicGroups;

    // Foxglove and http servers
    std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> server;
    std::unique_ptr<httplib::Server> httpServer;  // For the frontend
    std::unique_ptr<std::thread> httpServerThread;

    // Add a serviceId - function map
    std::map<foxglove::ServiceId, std::function<foxglove::ServiceResponse(foxglove::ServiceResponse)>> serviceMap;
};

}  // namespace dai