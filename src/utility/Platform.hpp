#pragma once

#include <cstdint>

#include "depthai/utility/JoiningThread.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
dai::Path getTempPath();
bool checkPathExists(const dai::Path& path, bool directory = false);
bool checkWritePermissions(const dai::Path& path);
dai::Path joinPaths(const dai::Path& path1, const dai::Path& path2);
dai::Path getDirFromPath(const dai::Path& path);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

}  // namespace platform
}  // namespace dai
