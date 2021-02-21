#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/XLinkOutProperties.hpp>

namespace dai {
namespace node {
class XLinkOut : public Node {
    dai::XLinkOutProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Input for any type of messages to be transfered over XLink stream
     * Default queue is blocking with size 8
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 8, {{DatatypeEnum::Buffer, true}}};

    void setStreamName(const std::string& name);
    void setFpsLimit(float fps);

    /**
     * Specify whether transfer data or only object attributes (metadata)
     */
    void setMetadataOnly(bool metadataOnly);

    std::string getStreamName() const;
    float getFpsLimit() const;

    /**
     * Get whether transfer data or only object attributes (metadata)
     */
    bool getMetadataOnly() const;
};

}  // namespace node
}  // namespace dai
