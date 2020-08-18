#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/generated/ColorCameraProperties.hpp>
#include <depthai-shared/generated/Generators.hpp>



namespace dai
{
    namespace node
    {
        class NeuralNetwork : public Node {
            dai::gen::NeuralNetworkProperties properties;
        
            std::string getName(){
                return "NeuralNetwork";
            }

            std::vector<Output> getOutputs(){
                return {out};
            }

            std::vector<Input> getInputs(){
                return {in};
            }

            nlohmann::json getProperties(){
                nlohmann::json j;
                nlohmann::to_json(j, properties);
                return j;
            }

            std::shared_ptr<Node> clone(){
                return std::make_shared<NeuralNetwork>(*this);
            }


            std::string blobPath;

        public:
            NeuralNetwork(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}

            Input in{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::RawBuffer, true}} };
            Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNTensor, false}} };



            // Specify local filesystem path to load the blob
            void setBlobPath(std::string path){
                blobPath = path;
            }

            void setNumPoolFrames(int numFrames){
                properties.numFrames = std::make_shared<int64_t>(numFrames);
            }

        };

    } // namespace node
} // namespace dai
