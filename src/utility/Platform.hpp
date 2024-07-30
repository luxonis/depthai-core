#pragma once

#include <string>
#include <cstdint>
#include "depthai/utility/JoiningThread.hpp"

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

void setThreadPriority(JoiningThread& thread, int priority);
int getMinThreadPriority(JoiningThread& thread);
int getMaxThreadPriority(JoiningThread& thread);

// Get the path to the temporary directory // TODO change this to std::filesystem::path if the project switches to C++ 17
std::string getTempPath();
}
}
