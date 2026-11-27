#pragma once

namespace dai::utils {

bool hasGPU();  // True if the device has a usable GPU (RVC4 only); cached for the process lifetime

}  // namespace dai::utils
