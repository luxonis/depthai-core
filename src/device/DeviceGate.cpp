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
const auto sessionsEndpoint = API_ROOT + "/sessions";
const int DEFAULT_PORT{11492};

class DeviceGate::Impl {
   public:
    Impl() = default;

    // Default Gate connection
    std::unique_ptr<httplib::Client> cli;
};
DeviceGate::~DeviceGate() {
    if(!stopSession()) {
        spdlog::warn("DeviceGate stopSession not successful");
    }
    if(stateMonitoringThread.joinable()) {
        stateMonitoringThread.join();
    }
    if(!destroySession()) {
        spdlog::warn("DeviceGate destroySession not successful");
    }
    if(!deleteSession()) {
        spdlog::warn("DeviceGate deleteSession not successful");
    }
}

DeviceGate::DeviceGate(const DeviceInfo& deviceInfo) : deviceInfo(deviceInfo) {
    if(deviceInfo.state != X_LINK_GATE) {
        throw std::invalid_argument("Device is not in Gate state");
    }

    // Discover and connect
    pimpl->cli = std::make_unique<httplib::Client>(deviceInfo.name, DEFAULT_PORT);
    // pimpl->cli->set_connection_timeout(2);
    stateMonitoringThread = std::thread(&DeviceGate::threadedStateMonitoring, this);
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
                                        {"fwp_version", DEPTHAI_DEVICE_RVC3_VERSION},
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
                    sessionState = SessionState::CREATED;
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
        sessionState = SessionState::CREATED;
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
    if(sessionState == SessionState::DESTROYED) {
        spdlog::warn("DeviceGate trying to destroy already destroyed session");
        return true;
    }

    if(sessionState == SessionState::NOT_CREATED) {
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
    if(sessionState == SessionState::NOT_CREATED) {
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

DeviceGate::SessionState DeviceGate::updateState() {
    if(sessionState == SessionState::NOT_CREATED) {
        spdlog::debug("Session not yet created - can't get the session state");
        return sessionState;
    }
    std::string url = fmt::format("{}/{}", sessionsEndpoint, sessionId);
    if(auto res = pimpl->cli->Get(url.c_str())) {
        if(res->status != 200) {
            spdlog::warn("DeviceGate updateState not successful - status: {}, error: {}", res->status, res->body);
            return SessionState::ERROR;
        }
        auto resp = nlohmann::json::parse(res->body);
        spdlog::trace("DeviceGate updateState response: {}", resp.dump());

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
            spdlog::warn("DeviceGate updateState not successful - unknown session state: {}", sessionStateStr);
            sessionState = SessionState::ERROR;
        }
        return sessionState;
    } else {
        spdlog::warn("DeviceGate updateState not successful - got no response");
    }
    return SessionState::ERROR;
}

tl::optional<std::string> DeviceGate::saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename) {
    std::string path = "/tmp/" + filename;  // Make this OS agnostic
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

tl::optional<std::vector<uint8_t>> DeviceGate::getFile(const std::string& fileUrl) {
    // Send a GET request to the server
    if (auto res = pimpl->cli->Get(fileUrl.c_str())) {
        // Check if the request was successful
        if (res->status == 200) {
            // Convert the response body to a vector of uint8_t
            std::vector<uint8_t> fileData(res->body.begin(), res->body.end());
            spdlog::debug("File download successful");
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

void DeviceGate::threadedStateMonitoring() {
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto sessionState = updateState();
        if(sessionState == SessionState::ERROR) {
            spdlog::error("DeviceGate session state is in error state - stopping the monitoring thread");
            return;
        }
        switch(sessionState) {
            case SessionState::NOT_CREATED:
            case SessionState::CREATED:
            case SessionState::RUNNING:
            case SessionState::STOPPING:
                break;  // Nothing to do
            case SessionState::ERROR:
                spdlog::error("DeviceGate session state is in error state - stopping the monitoring thread");
                return;
            case SessionState::STOPPED:
                return;  // Session stopped - stop the thread
            case SessionState::CRASHED:
            case SessionState::DESTROYED:
                spdlog::warn("FW crashed - trying to get out the logs and the core dump");
                auto logFile = getLogFile();
                if(logFile) {
                    std::string logFileName = "depthai_gate.log";  // TODO make this OS independent
                    spdlog::warn("Log file found - trying to save it");
                    if(auto path = saveFileToTemporaryDirectory(*logFile, logFileName)) {
                        spdlog::warn("Log file saved to {} - please report to developers", *path);
                    } else {
                        spdlog::error("Couldn't save log file");
                    }
                } else {
                    spdlog::warn("Log file not found");
                }
                auto coreDump = getCoreDump();
                if(coreDump) {
                    std::string coreDumpName = "depthai_gate.core";  // TODO make this OS independent
                    spdlog::warn("Core dump found - trying to save it");
                    if(auto path = saveFileToTemporaryDirectory(*coreDump, coreDumpName)) {
                        spdlog::warn("Core dump saved to {} - please report to developers", *path);
                    } else {
                        spdlog::error("Couldn't save core dump");
                    }
                } else {
                    spdlog::warn("Core dump not found");
                }
                return;
        }
    }
}

tl::optional<std::vector<uint8_t>> DeviceGate::getLogFile(){
    std::string url = fmt::format("{}/{}/log_file", sessionsEndpoint, sessionId);
    return DeviceGate::getFile(url);
}

tl::optional<std::vector<uint8_t>> DeviceGate::getCoreDump(){
    std::string url = fmt::format("{}/{}/core_dump", sessionsEndpoint, sessionId);
    return DeviceGate::getFile(url);
}

// TODO(themarpe) - get all sessions, check if only one and not protected
bool DeviceGate::isBootedNonExclusive() {
    return true;
}

}  // namespace dai
