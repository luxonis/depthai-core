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
        
            std::string getName(){
                return "XLinkIn";
            }

        
            std::vector<Input> getInputs(){
                return {};
            }
            
            std::vector<Output> getOutputs(){
                return {out};
            }

            nlohmann::json getProperties(){
                nlohmann::json j;
                nlohmann::to_json(j, properties);
                return j;
            }

            std::shared_ptr<Node> clone(){
                return std::make_shared<std::decay<decltype(*this)>::type>(*this);
            }
        
        public:
            XLinkIn(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}
            Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::RawBuffer, true}} };

            void setStreamName(std::string name){
                properties.streamName = name;
            }

            void setMaxDataSize(std::uint32_t maxDataSize){
                properties.maxDataSize = maxDataSize;
            }

            void setNumFrames(std::uint32_t numFrames){
                properties.numFrames = numFrames;
            }

        };

    } // namespace node
} // namespace dai
