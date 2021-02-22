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

   public:
    ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    ImageManipConfig initialConfig;

    /**
     * Input ImageManipConfig message with ability to modify parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImageManipConfig, true}}};

    /**
     * Input image to be modified
     * Default queue is blocking with size 8
     */
    Input inputImage{*this, "inputImage", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};

    // Functions to set ImageManipConfig - deprecated
    [[deprecated("Use 'initialConfig.setCropRect()' instead")]] void setCropRect(float xmin, float ymin, float xmax, float ymax);
    [[deprecated("Use 'initialConfig.setCenterCrop()' instead")]] void setCenterCrop(float ratio, float whRatio = 1.0f);
    [[deprecated("Use 'initialConfig.setResize()' instead")]] void setResize(int w, int h);
    [[deprecated("Use 'initialConfig.setResizeThumbnail()' instead")]] void setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);
    [[deprecated("Use 'initialConfig.setFrameType()' instead")]] void setFrameType(dai::RawImgFrame::Type name);
    [[deprecated("Use 'initialConfig.setHorizontalFlip()' instead")]] void setHorizontalFlip(bool flip);
    void setKeepAspectRatio(bool keep);

    // Functions to set properties
    void setWaitForConfigInput(bool wait);
    void setNumFramesPool(int numFramesPool);
    void setMaxOutputFrameSize(int maxFrameSize);
};

}  // namespace node
}  // namespace dai
