#include "depthai/remote_connection/RemoteConnection.hpp"

#include "RemoteConnectionImpl.hpp"
#include "utility/PimplImpl.hpp"
namespace dai {

RemoteConnection::RemoteConnection(const std::string& address, uint16_t webSocketPort, bool serveFrontend, uint16_t httpPort)
    : impl(address, webSocketPort, serveFrontend, httpPort) {}

RemoteConnection::~RemoteConnection() = default;

void RemoteConnection::addTopic(const std::string& topicName, Node::Output& output, const std::string& group, bool useVisualizationIfAvailable) {
    impl->addTopic(topicName, output, group, useVisualizationIfAvailable);
}

std::shared_ptr<MessageQueue> RemoteConnection::addTopic(
    const std::string& topicName, const std::string& group, unsigned int maxSize, bool blocking, bool useVisualizationIfAvailable) {
    return impl->addTopic(topicName, group, maxSize, blocking, useVisualizationIfAvailable);
}

bool RemoteConnection::removeTopic(const std::string& topicName) {
    return impl->removeTopic(topicName);
}

void RemoteConnection::registerPipeline(const Pipeline& pipeline) {
    impl->registerPipeline(pipeline);
}

int RemoteConnection::waitKey(int delayMs) {
    return impl->waitKey(delayMs);
}

void RemoteConnection::registerService(const std::string& serviceName, std::function<nlohmann::json(const nlohmann::json&)> callback) {
    impl->registerService(serviceName, std::move(callback));
}

void RemoteConnection::registerBinaryService(const std::string& serviceName, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> callback) {
    impl->registerBinaryService(serviceName, std::move(callback));
}

}  // namespace dai