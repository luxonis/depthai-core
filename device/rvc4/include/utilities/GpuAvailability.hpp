#pragma once

namespace dai::utils {

// Returns true if the device has a usable GPU (RVC4 only).
// The result is cached for the lifetime of the process.
bool hasGPU();

}  // namespace dai::utils
