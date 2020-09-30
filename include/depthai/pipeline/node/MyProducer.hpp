#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/MyProducerProperties.hpp>

namespace dai
{
    namespace node
    {
        class MyProducer : public Node {
            dai::MyProducerProperties properties;
        
            std::string getName(){
                return "MyProducer";
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
            MyProducer(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}
            Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::RawBuffer, true}} };

            void setMessage(std::string m){
                properties.message = m;
            }

            void setProcessor(ProcessorType proc){
                properties.processorPlacement = proc;
            }

        };

    } // namespace node
} // namespace dai
