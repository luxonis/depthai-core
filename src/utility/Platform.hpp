#pragma once

#include <cstdint>
#include <filesystem>

#include "depthai/utility/JoiningThread.hpp"

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
std::filesystem::path getTempPath();
bool checkPathExists(const std::filesystem::path& path, bool directory = false);
bool checkWritePermissions(const std::filesystem::path& path);
std::filesystem::path joinPaths(const std::filesystem::path& path1, const std::filesystem::path& path2);
std::filesystem::path getDirFromPath(const std::filesystem::path& path);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

}  // namespace platform
}  // namespace dai
