#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/properties/FocusedDepthCropProperties.hpp>

namespace dai {
namespace node {

class FocusedDepth;

/**
 * Host-side helper: consumes a MessageGroup with keys ``left``, ``right``, ``roi`` and emits
 * symmetrically cropped left/right frames suitable for StereoDepth / NeuralDepth.
 */
class FocusedDepthCrop : public DeviceNodeCRTP<DeviceNode, FocusedDepthCrop, FocusedDepthCropProperties>, public HostRunnable {
    friend class FocusedDepth;

   private:
    bool runOnHostVar = true;

   protected:
    Properties& getProperties() override;
    FocusedDepthCrop(std::unique_ptr<Properties> props);

   public:
    constexpr static const char* NAME = "FocusedDepthCrop";
    FocusedDepthCrop();

    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::MessageGroup, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output leftOut{*this, {"left", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output rightOut{*this, {"right", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    FocusedDepthCrop& setRunOnHost(bool runOnHost = true);
    bool runOnHost() const override;

    void run() override;
};

}  // namespace node
}  // namespace dai
