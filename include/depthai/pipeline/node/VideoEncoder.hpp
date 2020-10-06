#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/VideoEncoderProperties.hpp>

namespace dai {
namespace node {
    class VideoEncoder : public Node {
        dai::VideoEncoderProperties properties;

        std::string getName() override;
        std::vector<Input> getInputs() override;
        std::vector<Output> getOutputs() override;
        nlohmann::json getProperties() override;
        std::shared_ptr<Node> clone() override;

       public:
        VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

        Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, true}}};
        Output bitstream{*this, "bitstream", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

        void setDefaultProfilePreset(int width, int height, VideoEncoderProperties::Profile profile);
    };

}  // namespace node
}  // namespace dai
