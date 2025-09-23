#pragma once

#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
namespace utility {

class PipelineEventDispatcherInterface {
public:
    virtual ~PipelineEventDispatcherInterface() = default;
    virtual void setNodeId(int64_t id) = 0;
    virtual void addEvent(const std::string& source, PipelineEvent::EventType type) = 0;
    virtual void startEvent(const std::string& source, std::optional<Buffer> metadata = std::nullopt) = 0;  // Start event with a start and an end
    virtual void endEvent(const std::string& source) = 0;                                                   // Stop event with a start and an end
    virtual void pingEvent(const std::string& source) = 0;                                                  // Event where stop and start are the same (eg. loop)
};

}  // namespace utility
}  // namespace dai
