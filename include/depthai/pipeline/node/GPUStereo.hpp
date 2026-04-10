#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/GPUStereoConfig.hpp>
#include <depthai/pipeline/node/MessageDemux.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>

namespace dai {
namespace node {

class GPUStereo : public DeviceNodeCRTP<DeviceNode, GPUStereo, GPUStereoProperties> {
   protected:
    using DeviceNodeCRTP::DeviceNodeCRTP;
    Properties& getProperties() override;
    GPUStereo(std::unique_ptr<Properties> props);

   public:
    constexpr static const char* NAME = "GPUStereo";
    GPUStereo() = default;

    std::shared_ptr<GPUStereoConfig> initialConfig = std::make_shared<GPUStereoConfig>();

    std::shared_ptr<GPUStereo> build(Output& left, Output& right);
    GPUStereo& setRectification(bool enable);

    Subnode<Sync> sync{*this, "sync"};
    Subnode<MessageDemux> messageDemux{*this, "messageDemux"};
    Subnode<Rectification> rectification{*this, "rectification"};

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    Input& left{sync->inputs["left"]};
    Input& right{sync->inputs["right"]};

    Output& rectifiedLeft{rectification->output1};
    Output& rectifiedRight{rectification->output2};
#endif

    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::GPUStereoConfig, false}}}}};

    Input leftInternal{*this, {"leftFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};
    Input rightInternal{*this, {"rightFrameInternal", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output disparity{*this, {"disparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output debugPyramid{*this, {"debugPyramid", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output debugPyramidDisparity{*this, {"debugPyramidDisparity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output debugZnccCurve{*this, {"debugZnccCurve", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
