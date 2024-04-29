// C++ std
// #include <string>

// libraries
#include <spimpl.h>

// depthai public
#include <depthai/capabilities/Capability.hpp>
#include <depthai/capabilities/CapabilityRange.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>

namespace dai {

class ImgFrameCapability : public CapabilityCRTP<Capability, ImgFrameCapability> {
   public:
    constexpr static const char* NAME = "dai/img-frame";
    // Capability getIntersection(const Capability& other) override;

    CapabilityRange<std::tuple<uint32_t, uint32_t>> size;
    CapabilityRange<uint32_t> fps;
    std::optional<ImgFrame::Type> encoding;
    // envelope / background around
    // TODO(jakgra) add optional CapabilityRange fov / max-min horiz. / vertical crop;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
