#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/XLinkInProperties.hpp>

namespace dai
{
namespace node
{
    class XLinkIn : public Node {
        dai::XLinkInProperties properties;
    
        std::string getName() override;
        std::vector<Input> getInputs() override;
        std::vector<Output> getOutputs() override;
        nlohmann::json getProperties() override;
        std::shared_ptr<Node> clone() override;
    
    public:
        XLinkIn(const std::shared_ptr<PipelineImpl>& par);
        
        Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::RawBuffer, true}} };

        void setStreamName(std::string name);
        void setMaxDataSize(std::uint32_t maxDataSize);
        void setNumFrames(std::uint32_t numFrames);

    };

} // namespace node
} // namespace dai
