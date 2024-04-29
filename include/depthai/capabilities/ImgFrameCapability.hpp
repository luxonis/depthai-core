// C++ std
// #include <string>

// libraries
#include <spimpl.h>

// depthai public
#include <depthai/capabilities/Capability.hpp>
#include <depthai/capabilities/CapabilityRange.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {

enum struct ImgResizeMode {
    /**
     * Doesn't keep aspect ratio.
     * Squishes or streches the image to fill the required pixel area.
     */
    FILL,
    /**
     * Keeps aspect ratio.
     * Envelops the image with a background color to get the corect output aspect ratio.
     */
    CONTAIN,
    /**
     * Keeps aspect ratio.
     * Crops the image to get the correct output aspect ratio.
     */
    COVER
};

class ImgFrameCapability : public CapabilityCRTP<Capability, ImgFrameCapability> {
   public:
    constexpr static const char* NAME = "dai/img-frame";
    // Capability getIntersection(const Capability& other) override;

    CapabilityRange<std::tuple<uint32_t, uint32_t>> size;
    CapabilityRange<uint32_t> fps;
    std::optional<ImgFrame::Type> encoding;
    ImgResizeMode resizeMode{ImgResizeMode::COVER};
    // TODO(jakgra) add optional CapabilityRange fov / max-min horiz. / vertical crop;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
