#pragma once

#include <cstdint>
#include "depthai/utility/JoiningThread.hpp"
#include <string>

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
std::string getTempPath();
bool checkPathExists(const std::string& path, bool directory=false);
bool checkWritePermissions(const std::string& path);
std::string joinPaths(const std::string& path1, const std::string& path2);
std::string getDirFromPath(const std::string& path);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

}  // namespace platform
}  // namespace dai
