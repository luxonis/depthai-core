#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"
#include "depthai/properties/ImageManipPropertiesV2.hpp"

namespace dai {
namespace node {

class ImageManipHost : public NodeCRTP<ThreadedHostNode, ImageManipHost /*, ImageManipPropertiesV2*/> {
   public:
    constexpr static const char* NAME = "ImageManipHost";

    /*using DeviceNodeCRTP::DeviceNodeCRTP;*/
    /*std::shared_ptr<ImageManipHost> build() {*/
    /*    return std::static_pointer_cast<ImageManipHost>(shared_from_this());*/
    /*}*/
    ImageManipPropertiesV2 properties;
    ImageManipConfigV2 initialConfig;

    Input inputConfig{*this, {.name = "inputConfig", .types = {{DatatypeEnum::ImageManipConfigV2, true}}, .waitForMessage = false}};
    Input inputImage{*this, {.name = "inputImage", .queueSize = 15, .types = {{DatatypeEnum::ImgFrame, false}}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::ImgFrame, false}}}};

    void run() override;

    ImageManipHost& setMaxOutputFrameSize(size_t size) {
        properties.outputFrameSize = size;
        return *this;
    }

    std::string getConfigString(const size_t inWidth, const size_t inHeight);
};
}  // namespace node

}  // namespace dai
