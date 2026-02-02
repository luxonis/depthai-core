#pragma once

#include <functional>
#include <memory>
#include <string>

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {

class RemoteConnectionImpl;

/**
 * @class RemoteConnection
 * @brief Runs a websocket server exposing DepthAI messages as well as a static frontend UI
 */
class RemoteConnection {
   public:
    static constexpr auto DEFAULT_WEBSOCKET_PORT = 8765;  ///< Default WebSocket port.
    static constexpr auto DEFAULT_HTTP_PORT = 8082;       ///< Default HTTP port.
    static constexpr auto DEFAULT_ADDRESS = "0.0.0.0";    ///< Default address to bind.

    /**
     * @brief Constructs a RemoteConnection instance.
     *
     * @param address The address to bind the connection to.
     * @param webSocketPort The port for WebSocket communication.
     * @param serveFrontend Whether to serve a frontend UI.
     * @param httpPort The port for HTTP communication.
     */
    explicit RemoteConnection(const std::string& address = DEFAULT_ADDRESS,
                              uint16_t webSocketPort = DEFAULT_WEBSOCKET_PORT,
                              bool serveFrontend = true,
                              uint16_t httpPort = DEFAULT_HTTP_PORT);

    /**
     * @brief Destroys the RemoteConnection instance.
     */
    ~RemoteConnection();

    /**
     * @brief Adds a topic to the remote connection.
     *
     * @param topicName The name of the topic.
     * @param output The output to link to the topic.
     * @param group An optional group name for the topic.
     * @param useVisualizationIfAvailable Whether to enable visualization on the message if available or send message as is.
     */
    void addTopic(const std::string& topicName, Node::Output& output, const std::string& group = "", bool useVisualizationIfAvailable = true);

    /**
     * @brief Adds a topic with a message queue.
     *
     * @param topicName The name of the topic.
     * @param group An optional group name for the topic.
     * @param maxSize The maximum queue size.
     * @param blocking Whether the queue is blocking or non-blocking.
     * @param useVisualizationIfAvailable Whether to enable visualization on the message if available or send message as is.
     * @return A shared pointer to the created message queue.
     */
    std::shared_ptr<MessageQueue> addTopic(
        const std::string& topicName, const std::string& group = "", unsigned int maxSize = 16, bool blocking = false, bool useVisualizationIfAvailable = true);

    /**
     * @brief Removes a topic from the remote connection.
     *
     * @param topicName The name of the topic to remove.
     * @note After removing a topic any messages sent to it will cause an exception to be called on the sender, since this closes the queue.
     * @return True if the topic was successfully removed, false otherwise.
     */
    bool removeTopic(const std::string& topicName);

    /**
     * @brief Registers a pipeline with the remote connection.
     *
     * @param pipeline The pipeline to register.
     */
    void registerPipeline(const Pipeline& pipeline);

    /**
     * @brief Waits for a key event.
     *
     * @param delayMs The delay in milliseconds to wait for a key press.
     * @return The key code of the pressed key.
     */
    int waitKey(int delayMs);

    /**
     * @brief Registers a service with a callback function.
     *
     * @param serviceName The name of the service.
     * @param callback The callback function to handle requests.
     */
    void registerService(const std::string& serviceName, std::function<nlohmann::json(const nlohmann::json&)> callback);

    /**
     * @brief Registers a binary service with a callback function.
     *
     * @param serviceName The name of the service.
     * @param callback The callback function to handle requests.
     */
    void registerBinaryService(const std::string& serviceName, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> callback);

   private:
    Pimpl<RemoteConnectionImpl> impl;  ///< PIMPL idiom for implementation hiding.
};

}  // namespace dai
