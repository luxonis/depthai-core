#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/XLinkOutProperties.hpp>

namespace dai
{
namespace node
{
    class XLinkOut : public Node {
        dai::XLinkOutProperties properties;
    
        std::string getName() override;
        std::vector<Input> getInputs() override;
        std::vector<Output> getOutputs() override;
        nlohmann::json getProperties() override;
        std::shared_ptr<Node> clone() override;

    public:
        XLinkOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

        Input in{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::RawBuffer, true}} };

        void setStreamName(std::string name);
        void setFpsLimit(double fps);

    };

} // namespace node
} // namespace dai
