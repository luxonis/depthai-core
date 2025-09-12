
#if defined(__clang__)
    #include "depthai/capabilities/Capability.hpp"
    #include "depthai/capabilities/ImgFrameCapability.hpp"
#endif

namespace dai {

#if defined(__clang__)
Capability::~Capability() = default;
#endif

#if defined(__clang__)
ImgFrameCapability::~ImgFrameCapability() = default;
#endif

}  // namespace dai