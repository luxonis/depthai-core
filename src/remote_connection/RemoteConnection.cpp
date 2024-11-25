#include "depthai/remote_connection/RemoteConnection.hpp"

#include "RemoteConnectionImpl.hpp"
#include "utility/PimplImpl.hpp"
namespace dai {

RemoteConnection::RemoteConnection(const std::string& address, uint16_t webSocketPort, bool serveFrontend, uint16_t httpPort)
    : impl(address, webSocketPort, serveFrontend, httpPort) {}

RemoteConnection::~RemoteConnection() = default;

void RemoteConnection::addTopic(const std::string& topicName, Node::Output& output, const std::string& group) {
    impl->addTopic(topicName, output, group);
}

std::shared_ptr<MessageQueue> RemoteConnection::addTopic(const std::string& topicName, const std::string& group, unsigned int maxSize, bool blocking) {
    return impl->addTopic(topicName, group, maxSize, blocking);
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

}  // namespace dai