// C++ std
#include <cstdint>
#include <optional>
#include <tuple>
// #include <string>

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
     * Doesn't keep aspect ratio.
     * Squishes or streches the image to fill the required pixel area.
     */
    FILL,
    /**
     * Keeps aspect ratio.
     * Envelop the image with a background color to get the corect output aspect ratio.
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

    DEPTHAI_SERIALIZE(ImgFrameCapability, size, fps, encoding, resizeMode);

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
