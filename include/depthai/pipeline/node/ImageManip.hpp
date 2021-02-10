#pragma once

#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>

// shared
#include <depthai-shared/properties/ImageManipProperties.hpp>

namespace dai {
namespace node {

class ImageManip : public Node {
    dai::ImageManipProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawImageManipConfig> rawConfig;
    ImageManipConfig config;

   public:
    ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, {{DatatypeEnum::ImageManipConfig, true}}};
    Input inputImage{*this, "inputImage", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, true}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};

    // Functions to set ImageManipConfig
    void setCropRect(float xmin, float ymin, float xmax, float ymax);
    void setCenterCrop(float ratio, float whRatio = 1.0f);
    void setResize(int w, int h);
    void setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);
    void setFrameType(dai::RawImgFrame::Type name);
    void setHorizontalFlip(bool flip);
    void setKeepAspectRatio(bool keep);

    // Functions to set properties
    void setWaitForConfigInput(bool wait);
    void setNumFramesPool(int numFramesPool);
    void setMaxOutputFrameSize(int maxFrameSize);
};

}  // namespace node
}  // namespace dai
