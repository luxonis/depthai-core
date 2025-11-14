#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Sync.hpp"

// shared
#include <depthai/properties/RectificationProperties.hpp>

namespace dai {
namespace node {

class Rectification : public DeviceNodeCRTP<DeviceNode, Rectification, RectificationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "Rectification";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Input images to be rectified
     */
    Input input1{*this, {"input1", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, true}}}};
    Input input2{*this, {"input2", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough1{*this, {"passthrough1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output passthrough2{*this, {"passthrough2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Send outputs
     */
    Output output1{*this, {"output1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output output2{*this, {"output2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Set output size
     */
    Rectification& setOutputSize(uint32_t width, uint32_t height);

    /**
     * Set output size
     */
    Rectification& setOutputSize(std::pair<uint32_t, uint32_t> size) {
        return setOutputSize(size.first, size.second);
    }

    /**
     * Enable or disable rectification (useful for minimal changes during debugging)
     */
    Rectification& enableRectification(bool enable) {
        properties.enableRectification = enable;
        return *this;
    }
    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    virtual CalibrationHandler getCalibrationData() const;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
