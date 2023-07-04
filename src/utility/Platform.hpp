#pragma once

#include <string>
#include <cstdint>
#include "depthai/utility/JoiningThread.hpp"

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
void setThreadName(JoiningThread& thread, const std::string& name);

}
}
