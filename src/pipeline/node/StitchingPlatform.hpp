#pragma once

#include "depthai/device/Platform.hpp"

namespace dai {
namespace node {
namespace stitching {

constexpr bool isDevicePlatformSupported(Platform platform) {
    return platform == Platform::RVC4;
}

}  // namespace stitching
}  // namespace node
}  // namespace dai
