#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/pb/properties/ColorCameraProperties.hpp>

namespace dai {
namespace node {
    class ColorCamera : public Node {
        dai::ColorCameraProperties properties;

        std::string getName() override;
        std::vector<Output> getOutputs() override;
        std::vector<Input> getInputs() override;
        nlohmann::json getProperties() override;
        std::shared_ptr<Node> clone() override;

       public:
        ColorCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

        Output video{*this, "video", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output preview{*this, "preview", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output still{*this, "still", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

        // Set which color camera to use
        void setCamId(int64_t id);

        // Get which color camera to use
        int64_t getCamId() const;

        // setColorOrder - RGB or BGR
        void setColorOrder(ColorCameraProperties::ColorOrder colorOrder);

        // getColorOrder - returns color order
        ColorCameraProperties::ColorOrder getColorOrder() const;

        // setInterleaved
        void setInterleaved(bool interleaved);

        // set preview output size
        void setPreviewSize(int width, int height);

        void setResolution(ColorCameraProperties::SensorResolution resolution);
    };

}  // namespace node
}  // namespace dai
