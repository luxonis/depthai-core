#include "device/DeviceGate.hpp"
#include <XLink/XLinkPublicDefines.h>

// std
#include <fstream>

// project
#include "build/version.hpp"
#include "device/Device.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Platform.hpp"
#include "utility/Resources.hpp"
#include "utility/Environment.hpp"

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
const auto sessionsEndpoint = API_ROOT + "/sessions";
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
    if(deviceInfo.platform != X_LINK_RVC3 && deviceInfo.platform != X_LINK_RVC4) {
        throw std::invalid_argument("Gate only supports RVC3 and RVC4 platforms");
    }
    this->platform = deviceInfo.platform;
    if(deviceInfo.platform == X_LINK_RVC3) {
        version = DEPTHAI_DEVICE_RVC3_VERSION;
    } else if(deviceInfo.platform == X_LINK_RVC4) {
        version = DEPTHAI_DEVICE_RVC4_VERSION;
    } else {
        throw std::runtime_error("Unknown platform"); // Should never happen
    }
    // Discover and connect
    pimpl->cli = std::make_unique<httplib::Client>(deviceInfo.name, DEFAULT_PORT);
    pimpl->cli->set_read_timeout(60);  // 60 seconds timeout to allow for compressing the crash dumps without async
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

bool DeviceGate::createSession(bool exclusive) {
    nlohmann::json createSessionBody = {{"name", "depthai_session"},
                                        // {"fwp_checksum", fwpChecksum},
                                        {"fwp_version", version},
                                        {"library_version", build::VERSION},
                                        {"protected", exclusive}};

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
                package = platform == X_LINK_RVC3 ? Resources::getInstance().getDeviceRVC3Fwp() : Resources::getInstance().getDeviceRVC4Fwp();
            }

            // TODO(themarpe) - Inefficient
            httplib::MultipartFormDataItems items = {
                {"file", std::string(package.begin(), package.end()), "depthai-device-kb-fwp.tar.xz", "application/octet-stream"},
            };

            std::string url = fmt::format("{}/{}/fwp", sessionsEndpoint, sessionId);
            if(auto res = pimpl->cli->Post(url.c_str(), items)) {
                if(res.value().status == 200) {
                    spdlog::debug("DeviceGate upload fwp successful");
                    sessionCreated = true;
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
        sessionCreated = true;
        return true;
    } else {
        spdlog::warn("DeviceGate createSession not successful - got no response");
    }
    return false;
}

bool DeviceGate::startSession() {
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

bool DeviceGate::stopSession() {
    auto sessionState = getState();
    if(sessionState == SessionState::STOPPED || sessionState == SessionState::DESTROYED) {
        spdlog::warn("DeviceGate trying to stop already stopped session");
        return true;
    }

    if(sessionState == SessionState::NOT_CREATED) {
        spdlog::debug("No need to stop a session that wasn't created.");
        return true;
    }

    const auto sessionsEndpoint = API_ROOT + "/sessions";

    std::string url = fmt::format("{}/{}/stop", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Post(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate stopSession not successful - status: {}, error: {}", res->status, res->body);
            return false;
        }
        spdlog::debug("DeviceGate stopSession successful");
        return true;
    } else {
        spdlog::error("DeviceGate stopSession not successful - got no response");
    }

    return false;
}

bool DeviceGate::destroySession() {
    if(getState() == SessionState::DESTROYED) {
        spdlog::warn("DeviceGate trying to destroy already destroyed session");
        return true;
    }

    if(getState() == SessionState::NOT_CREATED) {
        spdlog::debug("No need to destroy a session that wasn't created.");
        return true;
    }

    std::string url = fmt::format("{}/{}/destroy", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Post(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate destroySession not successful - status: {}, error: {}", res->status, res->body);
            return false;
        }
        spdlog::debug("DeviceGate destroySession successful");
        return true;
    } else {
        spdlog::error("DeviceGate destroySession not successful - got no response");
    }
    return false;
}

bool DeviceGate::deleteSession() {
    if(getState() == SessionState::NOT_CREATED) {
        spdlog::debug("No need to delete a session that wasn't created.");
        return true;
    }

    std::string url = fmt::format("{}/{}", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Delete(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate deleteSession not successful - status: {}, error: {}", res->status, res->body);
            return false;
        }
        spdlog::debug("DeviceGate deleteSession successful");
        return true;
    } else {
        spdlog::error("DeviceGate deleteSession not successful - got no response");
    }
    return false;
}

DeviceGate::SessionState DeviceGate::getState() {
    if(!sessionCreated) {
        spdlog::debug("Session not yet created - can't get the session state from gate");
        return SessionState::NOT_CREATED;
    }
    auto sessionState = SessionState::CREATED;
    std::string url = fmt::format("{}/{}", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Get(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate getState not successful - status: {}, error: {}", res->status, res->body);
            return SessionState::ERROR_STATE;
        }
        auto resp = nlohmann::json::parse(res->body);
        spdlog::trace("DeviceGate getState response: {}", resp.dump());

        std::string sessionStateStr = resp["state"];
        if(sessionStateStr == "CREATED") {
            sessionState = SessionState::CREATED;
        } else if(sessionStateStr == "RUNNING") {
            sessionState = SessionState::RUNNING;
        } else if(sessionStateStr == "STOPPED") {
            sessionState = SessionState::STOPPED;
        } else if(sessionStateStr == "STOPPING") {
            sessionState = SessionState::STOPPING;
        } else if(sessionStateStr == "CRASHED") {
            sessionState = SessionState::CRASHED;
        } else if(sessionStateStr == "DESTROYED") {
            sessionState = SessionState::DESTROYED;
        } else {
            spdlog::warn("DeviceGate getState not successful - unknown session state: {}", sessionStateStr);
            sessionState = SessionState::ERROR_STATE;
        }
        return sessionState;
    } else {
        spdlog::warn("DeviceGate getState not successful - got no response");
    }
    return SessionState::ERROR_STATE;
}

tl::optional<std::string> DeviceGate::saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename, std::string directoryPath) {
    std::string tmpdir;
    if(directoryPath.empty()) {
        tmpdir = platform::getTempPath();
    } else {
        tmpdir = directoryPath;
    }
    std::string path = std::string(tmpdir) + filename;

    std::ofstream file(path, std::ios::binary);
    if(!file.is_open()) {
        spdlog::error("Couldn't open file {} for writing", path);
        return tl::nullopt;
    }

    file.write(reinterpret_cast<char*>(data.data()), data.size());
    file.close();
    if(!file.good()) {
        spdlog::error("Couldn't write to file {}", path);
        return tl::nullopt;
    }
    spdlog::debug("Saved file {} to {}", filename, path);
    return std::string(path);
}

tl::optional<std::vector<uint8_t>> DeviceGate::getFile(const std::string& fileUrl, std::string& filename) {
    // Send a GET request to the server
    if(auto res = pimpl->cli->Get(fileUrl.c_str())) {
        if(res->status == 200) {
            filename = res->get_header_value("X-Filename");
            // Convert the response body to a vector of uint8_t
            std::vector<uint8_t> fileData(res->body.begin(), res->body.end());

            spdlog::debug("File download successful. Filename: {}", filename);
            return fileData;
        } else {
            spdlog::warn("File download not successful - status: {}, error: {}", res->status, res->body);
            return tl::nullopt;
        }
    } else {
        spdlog::warn("File download not successful - got no response");
        return tl::nullopt;
    }
}

void DeviceGate::waitForSessionEnd() {
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto sessionState = getState();
        if(sessionState == SessionState::ERROR_STATE) {
            spdlog::error("DeviceGate session state is in error state - exiting");
            return;
        }
        switch(sessionState) {
            case SessionState::NOT_CREATED:
            case SessionState::CREATED:
            case SessionState::RUNNING:
            case SessionState::STOPPING:
                break;  // Nothing to do
            case SessionState::ERROR_STATE:
                spdlog::error("DeviceGate session state is in error state - exiting");
                return;
            case SessionState::STOPPED:
                return;  // Session stopped - stop the thread
            case SessionState::CRASHED:
            case SessionState::DESTROYED:
                auto crashDumpPathStr = utility::getEnv("DEPTHAI_CRASHDUMP");
                if(crashDumpPathStr.empty()) {
                    spdlog::warn("Firmware crashed but the environment variable DEPTHAI_CRASHDUMP is not set, the crash dump will not be saved.");
                    return;
                }

                auto currentVersion = getVersion();
                auto requiredVersion = Version(0, 0, 14);
                if(currentVersion < requiredVersion) {
                    spdlog::warn("FW crashed but the gate version does not support transfering over the crash dump. Current version {}, required is {}",
                                 currentVersion.toString(),
                                 requiredVersion.toString());
                    return;
                }
                spdlog::warn("FW crashed - trying to get out the crash dump");
                std::this_thread::sleep_for(std::chrono::seconds(3));  // Allow for the generation of the crash dump and the log file
                std::string crashDumpName;
                spdlog::warn("Getting the crash dump out - this can take up to a minute, because it first needs to be compressed.");
                auto crashDump = getCrashDump(crashDumpName);
                if(crashDump) {
                    spdlog::warn("Crash dump found - trying to save it");
                    if(crashDumpName.empty()) {
                        crashDumpName = "depthai_gate_crash_dump.tar.gz";
                    }
                    std::string fullName = deviceInfo.getMxId() + "-" + crashDumpName;
                    if(auto path = saveFileToTemporaryDirectory(*crashDump, fullName, crashDumpPathStr)) {
                        spdlog::warn("Crash dump saved to {} - please report to developers", *path);
                    } else {
                        spdlog::error("Couldn't save crash dump");
                    }
                } else {
                    spdlog::warn("Crash dump not found");
                }
                return;
        }
    }
}

tl::optional<std::vector<uint8_t>> DeviceGate::getCrashDump(std::string& filename) {
    std::string url = fmt::format("{}/{}/core_dump", sessionsEndpoint, sessionId);
    return DeviceGate::getFile(url, filename);
}

// TODO(themarpe) - get all sessions, check if only one and not protected
bool DeviceGate::isBootedNonExclusive() {
    return true;
}

}  // namespace dai
