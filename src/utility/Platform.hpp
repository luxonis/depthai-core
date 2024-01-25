#pragma once

#include <cstdint>
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

}  // namespace platform
}  // namespace dai
