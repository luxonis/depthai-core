#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/pb/properties/ColorCameraProperties.hpp>


namespace dai
{
    namespace node
    {
        class ColorCamera : public Node{
            dai::ColorCameraProperties properties;
        
            std::string getName(){
                return "ColorCamera";
            }

            std::vector<Output> getOutputs(){
                return {video, preview, still};
            }

            std::vector<Input> getInputs(){
                return {};
            }

            nlohmann::json getProperties(){
                nlohmann::json j;
                nlohmann::to_json(j, properties);
                return j;
            }

            std::shared_ptr<Node> clone(){
                return std::make_shared<ColorCamera>(*this);
            }

        public:
            ColorCamera(const std::shared_ptr<PipelineImpl>& par) : Node(par) {
                properties.camId = 0;
                properties.colorOrder = ColorCameraProperties::ColorOrder::BGR;
                properties.interleaved = true;
                properties.previewHeight = 300;
                properties.previewWidth = 300;
                properties.resolution = ColorCameraProperties::SensorResolution::THE_1080_P;
            }

//            Output(Node& par, std::string n, Type t, std::vector< DatatypeHierarchy > types) : parent(par), type(t), name(n), possibleDatatypes(types) {}

            Output video{*this, "video", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}} };
            Output preview{*this, "preview", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}} };
            Output still{*this, "still", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}} };
            
            // Set which color camera to use
            void setCamId(int64_t id){
                properties.camId = id;
            }
            // Get which color camera to use
            int64_t getCamId(){
                return properties.camId;
            }


            // setColorOrder - RGB or BGR
            void setColorOrder(ColorCameraProperties::ColorOrder colorOrder){
                properties.colorOrder = colorOrder;
            }
            
            // getColorOrder - returns color order 
            ColorCameraProperties::ColorOrder getColorOrder(){
                return properties.colorOrder;
            }

            // setInterleaved
            void setInterleaved(bool interleaved){
                properties.interleaved = interleaved;
            }

            // set preview output size
            void setPreviewSize(int width, int height){
                properties.previewWidth = width;
                properties.previewHeight = height;
            }

            void setResolution(ColorCameraProperties::SensorResolution resolution){
                properties.resolution = resolution;
            }      


        };

    } // namespace node
} // namespace dai
