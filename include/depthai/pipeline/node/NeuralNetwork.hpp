#pragma once

#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/pb/properties/NeuralNetworkProperties.hpp>


namespace dai
{
    namespace node
    {
        class NeuralNetwork : public Node {
            dai::NeuralNetworkProperties properties;
        
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

            void loadBlob(std::string path){
                // Get pipelines asset manager
                AssetManager& assetManager = getParentPipeline().getAssetManager();

                // Load blob in blobPath into asset
                // And mark in properties where to look for it

                std::ifstream blobStream(blobPath, std::ios::in | std::ios::binary);
                if(!blobStream.is_open()) throw std::runtime_error("NeuralNetwork node | Blob at path: " + blobPath + " doesn't exist");

                std::vector<std::uint8_t> blob(std::istreambuf_iterator<char>(blobStream), {});

                /*
                // Get file size
                std::streampos fileSize;
                blobStream.seekg(0, std::ios::end);
                fileSize = blobStream.tellg();
                blobStream.seekg(0, std::ios::beg);

                // Read to vector
                std::vector<std::uint8_t> blob(fileSize);
                blobStream.read()
                */

                // Create an asset (alignment 64)
                Asset blobAsset;
                blobAsset.alignment = 64;
                blobAsset.data = std::move(blob);

                // Create asset key
                std::string assetKey = std::to_string(id)+"/blob";

                // set asset (replaces previous asset without throwing)
                assetManager.set(assetKey, blobAsset);

                // Set properties URI to asset:id/blob 
                properties.blobUri = std::string("asset:") + assetKey;
                properties.blobSize = blob.size();
            }


            std::string blobPath;

        public:
            NeuralNetwork(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}

            Input in{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::RawBuffer, true}} };
            Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNTensor, false}} };


            // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
            void setBlobPath(std::string path){
                blobPath = path;
                loadBlob(path);
            }

            void setNumPoolFrames(int numFrames){
                properties.numFrames = numFrames;
            }

        };

    } // namespace node
} // namespace dai
