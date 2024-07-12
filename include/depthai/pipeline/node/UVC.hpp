#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/UVCProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief UVC (USB Video Class) node
 */
class UVC : public DeviceNodeCRTP<DeviceNode, UVC, UVCProperties> {
   public:
    constexpr static const char* NAME = "UVC";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    std::shared_ptr<UVC> build() {
        return std::static_pointer_cast<UVC>(shared_from_this());
    }

   public:
    UVC() = default;
    UVC(std::unique_ptr<Properties> props);

    /**
     * Input for image frames to be streamed over UVC
     * Default queue is blocking with size 8
     */
    Input input{*this, {"in", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, true}};

    /// Set GPIO list <gpio_number, value> for GPIOs to set (on/off) at init
    void setGpiosOnInit(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is enabled
    void setGpiosOnStreamOn(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is disabled
    void setGpiosOnStreamOff(std::unordered_map<int, int> list);
};

}  // namespace node
}  // namespace dai
