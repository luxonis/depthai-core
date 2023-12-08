#pragma once

#include <cstdint>
#include <string>

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
std::string getTempPath();

}  // namespace platform
}  // namespace dai
