#pragma once

// C++ std
#include <cstdint>
#include <optional>

// libraries
#include <spimpl.h>

// depthai public
#include <depthai/capabilities/Capability.hpp>
#include <depthai/capabilities/CapabilityRange.hpp>
#include <depthai/common/optional.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/utility/Serialization.hpp>

namespace dai {

enum struct ImgResizeMode {
    /**
     * Keeps aspect ratio.
     * Crops the image to get the correct output aspect ratio.
     * Crops some FOV to match the required FOV, then scale. No potential NN accuracy decrease.
     */
    CROP,
    /**
     * Doesn't keep aspect ratio.
     * Squishes or streches the image to fill the required pixel area.
     * Preserves full FOV, but frames are stretched to match the FOV, which might decrease NN accuracy.
     */
    STRETCH,
    /**
     * Keeps aspect ratio.
     * Envelop the image with a background color to get the corect output aspect ratio.
     * Preserves full FOV by padding/letterboxing, but smaller frame means less features which might decrease NN accuracy.
     */
    LETTERBOX,
};

class ImgFrameCapability : public CapabilityCRTP<Capability, ImgFrameCapability> {
   public:
    constexpr static const char* NAME = "dai/img-frame";
    // Capability getIntersection(const Capability& other) override;

    CapabilityRange<std::pair<uint32_t, uint32_t>> size;
    CapabilityRange<uint32_t> fps;
    std::optional<ImgFrame::Type> encoding;
    ImgResizeMode resizeMode{ImgResizeMode::CROP};
    // TODO(jakgra) add optional CapabilityRange fov / max-min horiz. / vertical crop;

    DEPTHAI_SERIALIZE(ImgFrameCapability, size, fps, encoding, resizeMode);

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
