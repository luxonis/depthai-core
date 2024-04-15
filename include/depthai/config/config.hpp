#pragma once

// Build specific settings overwrite
#ifdef DEPTHAI_TARGET_CORE
    #ifndef DEPTHAI_TARGET_OPENCV
        #undef DEPTHAI_HAVE_OPENCV_SUPPORT
    #endif
#endif

namespace dai {
namespace build {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
constexpr static bool HAVE_OPENCV_SUPPORT = true;
#else
constexpr static bool HAVE_OPENCV_SUPPORT = false;
#endif

#ifdef DEPTHAI_HAVE_LIBUSB_SUPPORT
constexpr static bool HAVE_LIBUSB_SUPPORT = true;
#else
constexpr static bool HAVE_LIBUSB_SUPPORT = false;
#endif

}  // namespace build
}  // namespace dai