#pragma once

// std
#include <cstdint>
#include <string>
#include <vector>

namespace dai::gatewire {

/**
 * sha256 of data as 64 lowercase hex digits, the form the gate keys its firmware package cache by.
 * Empty when the build has no sha256 implementation, which makes a create upload the package as before.
 */
std::string sha256Hex(const std::vector<uint8_t>& data);

}  // namespace dai::gatewire
