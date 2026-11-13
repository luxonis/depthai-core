#pragma once

// std
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dai::gatewire {

/**
 * sha256 of data as 64 lowercase hex digits, the form the gate keys its firmware package cache by.
 * Empty when the build has no sha256 implementation, which makes a create upload the package as before.
 */
std::string sha256Hex(const std::vector<uint8_t>& data);

/**
 * Splits the payload of a GET_FILE_RAW answer: u32 little-endian filename length, the filename, the file bytes.
 * Returns the file bytes and sets filename, or returns nullopt and leaves filename alone when the lengths do not fit the payload.
 */
std::optional<std::vector<uint8_t>> parseRawFile(std::vector<uint8_t> payload, std::string& filename);

}  // namespace dai::gatewire
