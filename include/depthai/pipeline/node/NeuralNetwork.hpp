#pragma once

#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

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

            // Assets
            void loadAssets(AssetManager& assetManager){
                // Load blob in blobPath into asset
                // And mark in properties where to look for it

                std::ifstream blobStream(blobPath, std::ios::binary);
                if(!blobStream.is_open()) throw std::runtime_error("NeuralNetwork node | Blob at path: " + blobPath + " doesn't exist");

                std::vector<std::uint8_t> blob;
                            
                // get its size
                std::streampos fileSize;

                blobStream.seekg(0, std::ios::end);
                fileSize = blobStream.tellg();
                blobStream.seekg(0, std::ios::beg);

                // reserve
                blob.reserve(fileSize);

                // copy data
                blob.insert(blob.begin(), std::istream_iterator<std::uint8_t>(blobStream), std::istream_iterator<std::uint8_t>());

                // Create an asset (alignment 64)
                Asset blobAsset(blob.data(), blob.size(), 64);

                // Create asset key
                std::string assetKey = std::to_string(id)+"/blob";

                // Add asset
                assetManager.add(assetKey, blobAsset);

                // Set properties URI to asset:id/blob 
                properties.blobUri = std::string("asset:") + assetKey;
                properties.blobSize = std::make_shared<int64_t>(blob.size());
            }

            std::string blobPath;

        public:
            NeuralNetwork(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}

            Input in{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::RawBuffer, true}} };
            Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNTensor, false}} };


            // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
            void setBlobPath(std::string path){
                blobPath = path;
            }

            void setNumPoolFrames(int numFrames){
                properties.numFrames = std::make_shared<int64_t>(numFrames);
            }

        };

    } // namespace node
} // namespace dai
