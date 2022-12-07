#include "device/DeviceGate.hpp"

// std
#include <fstream>

// project
#include "build/version.hpp"
#include "device/Device.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Resources.hpp"

// libraries
#include "httplib.h"
#include "nlohmann/json.hpp"
#include "spdlog/fmt/fmt.h"
#include "spdlog/spdlog.h"

namespace dai {

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceGate::getFirstAvailableDevice() {
    return XLinkConnection::getFirstDevice(X_LINK_GATE);
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> DeviceGate::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices(X_LINK_GATE);
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

const std::string API_ROOT{"/api/v1"};
const int DEFAULT_PORT{11492};

class DeviceGate::Impl {
   public:
    Impl() = default;

    // Default Gate connection
    std::unique_ptr<httplib::Client> cli;
};
DeviceGate::~DeviceGate() {}

DeviceGate::DeviceGate(const DeviceInfo& deviceInfo) : deviceInfo(deviceInfo) {
    if(deviceInfo.state != X_LINK_GATE) {
        throw std::invalid_argument("Device is not in Gate state");
    }

    // Discover and connect
    pimpl->cli = std::make_unique<httplib::Client>(deviceInfo.name, DEFAULT_PORT);
    // pimpl->cli->set_connection_timeout(2);
}

bool DeviceGate::isOkay() {
    if(auto res = pimpl->cli->Get("/api/v1/status")) {
        return nlohmann::json::parse(res->body)["status"].get<bool>();
    }
    return false;
}

Version DeviceGate::getVersion() {
    httplib::Result res = pimpl->cli->Get("/api/v1/version");
    if(res && res->status == 200) {
        auto versionStr = nlohmann::json::parse(res->body)["version_gate"].get<std::string>();
        return Version{versionStr};
    }
    return Version{0, 0, 0};
}

DeviceGate::VersionInfo DeviceGate::getAllVersion() {
    httplib::Result res = pimpl->cli->Get("/api/v1/version");
    if(res && res->status == 200) {
        auto result = nlohmann::json::parse(res->body);

        VersionInfo info;
        info.gate = result.value("version_gate", "");
        info.os = result.value("version_os", "");
        return info;
    }
    return {};
}

bool DeviceGate::createSession() {
    const auto sessionsEndpoint = API_ROOT + "/sessions";

    nlohmann::json createSessionBody = {{"name", "depthai_session"},
                                        // {"fwp_checksum", fwpChecksum},
                                        {"fwp_version", DEPTHAI_DEVICE_RVC3_VERSION},
                                        {"library_version", build::VERSION},
                                        {"protected", false}};

    spdlog::debug("DeviceGate createSession: {}", createSessionBody.dump());

    if(auto res = pimpl->cli->Post(sessionsEndpoint.c_str(), createSessionBody.dump(), "application/json")) {
        // Parse response
        if(res->status != 200) {
            spdlog::warn("DeviceGate createSession not successful - status: {}, error: {}", res->status, res->body);
            return false;
        }
        auto resp = nlohmann::json::parse(res->body);
        spdlog::debug("DeviceGate createSession response: {}", resp.dump());

        // Retrieve sessionId
        sessionId = resp["id"];
        bool fwpExists = resp["fwp_exists"].get<bool>();

        if(!fwpExists) {
            std::vector<uint8_t> package;
            std::string path;
            if(!path.empty()) {
                std::ifstream fwStream(path, std::ios::binary);
                if(!fwStream.is_open()) throw std::runtime_error(fmt::format("Cannot flash bootloader, binary at path: {} doesn't exist", path));
                package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
            } else {
                package = Resources::getInstance().getDeviceKbFwp();
            }

            // TODO(themarpe) - Inefficient
            httplib::MultipartFormDataItems items = {
                {"file", std::string(package.begin(), package.end()), "depthai-device-kb-fwp.tar.xz", "application/octet-stream"},
            };

            std::string url = fmt::format("{}/{}/fwp", sessionsEndpoint, sessionId);
            if(auto res = pimpl->cli->Post(url.c_str(), items)) {
                if(res.value().status == 200) {
                    spdlog::debug("DeviceGate upload fwp successful");
                    return true;
                } else {
                    spdlog::warn("DeviceGate upload fwp not successful - status: {}, error: {}", res->status, res->body);
                    return false;
                }

            } else {
                spdlog::warn("DeviceGate upload fwp not successful - got no response");
                return false;
            }
        }
        return true;
    } else {
        spdlog::warn("DeviceGate createSession not successful - got no response");
    }
    return false;
}

bool DeviceGate::startSession() {
    const auto sessionsEndpoint = API_ROOT + "/sessions";

    std::string url = fmt::format("{}/{}/start", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Post(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate start fwp not successful - status: {}, error: {}", res->status, res->body);
            return false;
        }
        spdlog::debug("DeviceGate start fwp successful");
        return true;
    } else {
        spdlog::debug("DeviceGate start fwp not successful - got no response");
    }

    return false;
}

}  // namespace dai
