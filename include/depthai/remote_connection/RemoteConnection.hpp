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
    explicit RemoteConnection(const std::string& address = "0.0.0.0", uint16_t port = 8765);
    ~RemoteConnection();

    void addTopic(const std::string& topicName, Node::Output& output, const std::string& group = "");
    std::shared_ptr<MessageQueue> addTopic(const std::string& topicName, const std::string& group = "", unsigned int maxSize = 16, bool blocking = false);
    void registerPipeline(const Pipeline& pipeline);
    int waitKey(int delayMs);

   private:
    Pimpl<RemoteConnectionImpl> impl;
};

}  // namespace dai
