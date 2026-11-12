#include "utilities/AuxConnector.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <system_error>

namespace dai {
namespace aux {

namespace {

std::optional<std::string> readDeviceTreeString(const std::filesystem::path& property) {
    std::ifstream file(property, std::ios::binary);
    if(!file) {
        return std::nullopt;
    }
    std::string value{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    // Device tree strings end with a NUL
    return std::string(value.c_str());
}

}  // namespace

bool fsyncLinkedToConnector(const std::filesystem::path& deviceTree) {
    auto node = readDeviceTreeString(deviceTree / "aliases/stm_fsync");
    if(!node || node->empty()) {
        return false;
    }
    std::error_code ec;
    if(!std::filesystem::exists(deviceTree / std::filesystem::path(*node).relative_path() / "luxonis,aux-connector", ec)) {
        return false;
    }
    auto status = readDeviceTreeString(deviceTree / "aux-connector/status");
    return !status || *status == "okay" || *status == "ok";
}

bool connectorBound(const std::filesystem::path& device) {
    int fd = open(device.c_str(), O_RDONLY | O_CLOEXEC);
    if(fd < 0) {
        return false;
    }
    close(fd);
    return true;
}

}  // namespace aux
}  // namespace dai
