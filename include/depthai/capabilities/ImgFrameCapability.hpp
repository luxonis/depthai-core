// C++ std
#include <string>

// libraries
#include <spimpl.h>

// depthai public
#include <depthai/capabilities/Capability.hpp>

namespace dai {

class ImgFrameCapability : Capability {
   public:
    const std::string& getName() override;
    Capability getIntersection(const Capability& other) override;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
