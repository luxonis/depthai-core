#include "device/DeviceGate.hpp"

#include <XLink/XLink.h>
#include <XLink/XLinkPublicDefines.h>

// std
#include <fstream>
#include <optional>
#include <string>

// project
#include "build/version.hpp"
#include "device/Device.hpp"
#include "utility/Environment.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Platform.hpp"
#include "utility/Resources.hpp"

// libraries
#include "httplib.h"
#include "nlohmann/json.hpp"
#include "spdlog/fmt/fmt.h"
#include "spdlog/spdlog.h"

namespace dai {

const std::string API_ROOT{"/api/v1"};
const auto sessionsEndpoint = API_ROOT + "/sessions";
const int DEFAULT_PORT{11492};
const int gateTimeout = 2000;

enum USBRequestID_t {
    RESPONSE_OK,
    RESPONSE_ERROR,
    CREATE_SESSION,
    START_SESSION,
    UPLOAD_FWP,
    STOP_SESSION,
    DESTROY_SESSION,
    DELETE_SESSION,
    IS_OKAY,
    GET_VERSION,
    GET_STATE,
    GET_FILE,
};

struct USBRequest_t {
    uint32_t RequestNum;
    uint32_t RequestSize;
};

class DeviceGate::HTTPImpl::Impl {
   public:
    Impl() = default;

    // Default Gate connection
    std::unique_ptr<httplib::Client> cli;
};
DeviceGate::~DeviceGate() {}

DeviceGate::DeviceGate(const DeviceInfo& deviceInfo) : deviceInfo(deviceInfo) {
    if((deviceInfo.state != X_LINK_GATE) && (deviceInfo.state != X_LINK_GATE_SETUP)) {
        throw std::invalid_argument(
            "Device is already used by another application/process. Make sure to close all applications/processes using the device before starting a new one.");
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
        throw std::runtime_error("Unknown platform");  // Should never happen
    }

    if(deviceInfo.protocol == X_LINK_USB_EP) {
        impl = std::make_shared<USBImpl>(gateTimeout);
    } else {
        impl = std::make_shared<HTTPImpl>(deviceInfo);
    }
}

DeviceGate::HTTPImpl::HTTPImpl(DeviceInfo deviceInfo) {
    // Discover and connect
    pimpl->cli = std::make_unique<httplib::Client>(deviceInfo.name, DEFAULT_PORT);
    pimpl->cli->set_read_timeout(60);  // 60 seconds timeout to allow for compressing the crash dumps without async
    // pimpl->cli->set_connection_timeout(2);
}

DeviceGate::USBImpl::USBImpl(int timeout) : timeout(timeout) {}

bool DeviceGate::isOkay() {
    return impl->isOkay();
}

bool DeviceGate::HTTPImpl::isOkay() {
    auto res = pimpl->cli->Get("/api/v1/status");
    return nlohmann::json::parse(res->body)["status"].get<bool>();
}

bool DeviceGate::USBImpl::isOkay() {
    USBRequest_t request;
    request.RequestNum = IS_OKAY;
    request.RequestSize = 0;
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(request.RequestNum == RESPONSE_ERROR) {
        return false;
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return {};
    }
    respBuffer[request.RequestSize] = '\0';

    bool result = nlohmann::json::parse(respBuffer)["status"].get<bool>();

    return result;
}

Version DeviceGate::getVersion() {
    return impl->getVersion();
}

Version DeviceGate::HTTPImpl::getVersion() {
    httplib::Result res = pimpl->cli->Get("/api/v1/version");
    if(res && res->status == 200) {
        auto versionStr = nlohmann::json::parse(res->body)["version_gate"].get<std::string>();
        return Version{versionStr};
    } else {
        return Version{0, 0, 0};
    }
}

Version DeviceGate::USBImpl::getVersion() {
    USBRequest_t request;
    request.RequestNum = GET_VERSION;
    request.RequestSize = 0;
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return {0, 0, 0};
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return {0, 0, 0};
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        return Version{0, 0, 0};
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return {0, 0, 0};
    }
    respBuffer[request.RequestSize] = '\0';

    auto result = nlohmann::json::parse(respBuffer)["version_gate"].get<std::string>();

    return Version{result};
}

DeviceGate::VersionInfo DeviceGate::getAllVersion() {
    return impl->getAllVersion();
}

DeviceGate::VersionInfo DeviceGate::HTTPImpl::getAllVersion() {
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

DeviceGate::VersionInfo DeviceGate::USBImpl::getAllVersion() {
    USBRequest_t request;
    request.RequestNum = GET_VERSION;
    request.RequestSize = 0;
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return {};
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return {};
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        return {};
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return {};
    }
    respBuffer[request.RequestSize] = '\0';
    auto result = nlohmann::json::parse(respBuffer);

    VersionInfo info;
    info.gate = result.value("version_gate", "");
    info.os = result.value("version_os", "");
    return info;
}

bool DeviceGate::createSession(bool exclusive) {
    return impl->createSession(version, exclusive, platform, sessionId, sessionCreated);
}

bool DeviceGate::HTTPImpl::createSession(
    std::string version, bool exclusive, XLinkPlatform_t platform, std::string& sessionId, std::atomic_bool& sessionCreated) {
    nlohmann::json createSessionBody = {{"name", "depthai_session"},
                                        // {"fwp_checksum", fwpChecksum},
                                        {"fwp_version", version},
                                        {"library_version", build::VERSION},
                                        {"protected", exclusive}};

    spdlog::debug("DeviceGate createSession: {}", createSessionBody.dump());

    auto res = pimpl->cli->Post(sessionsEndpoint.c_str(), createSessionBody.dump(), "application/json");
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
}

bool DeviceGate::USBImpl::createSession(
    std::string version, bool exclusive, XLinkPlatform_t platform, std::string& sessionId, std::atomic_bool& sessionCreated) {
    nlohmann::json createSessionBody = {{"name", "depthai_session"},
                                        // {"fwp_checksum", fwpChecksum},
                                        {"fwp_version", version},
                                        {"library_version", build::VERSION},
                                        {"protected", exclusive}};

    spdlog::debug("DeviceGate createSession: {}", createSessionBody.dump());
    USBRequest_t request;
    request.RequestNum = CREATE_SESSION;
    request.RequestSize = createSessionBody.dump().size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(XLinkGateWrite((void*)createSessionBody.dump().c_str(), createSessionBody.dump().size(), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::warn("DeviceGate createSession not successful - got no response");
        return false;
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return false;
    }
    respBuffer[request.RequestSize] = '\0';
    auto resp = nlohmann::json::parse(respBuffer);
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

        nlohmann::json uploadFwpBody = {{"sessionId", sessionId}, {"file", package}};
        request.RequestNum = UPLOAD_FWP;
        request.RequestSize = uploadFwpBody.dump().size();
        if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
            return false;
        }
        if(XLinkGateWrite((void*)uploadFwpBody.dump().c_str(), uploadFwpBody.dump().size(), timeout) == X_LINK_ERROR) {
            return false;
        }

        if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
            return false;
        }
        if(request.RequestNum == RESPONSE_ERROR) {
            spdlog::warn("DeviceGate upload fwp not successful - got no response");
            return false;
        }
    }
    sessionCreated = true;
    return true;
}

bool DeviceGate::startSession() {
    return impl->startSession(sessionId);
}

bool DeviceGate::HTTPImpl::startSession(std::string sessionId) {
    std::string url = fmt::format("{}/{}/start", sessionsEndpoint, sessionId);
    auto res = pimpl->cli->Post(url.c_str());
    if(res->status != 200) {
        spdlog::warn("DeviceGate start fwp not successful - status: {}, error: {}", res->status, res->body);
        return false;
    }
    spdlog::debug("DeviceGate start fwp successful");
    return true;
}

bool DeviceGate::USBImpl::startSession(std::string sessionId) {
    USBRequest_t request;
    request.RequestNum = START_SESSION;
    request.RequestSize = sessionId.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(XLinkGateWrite((void*)sessionId.c_str(), sessionId.size(), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::debug("DeviceGate start fwp not successful - got no response");
        return false;
    }
    spdlog::debug("DeviceGate start fwp successful");
    return true;
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
    return impl->stopSession(sessionId);
}

bool DeviceGate::HTTPImpl::stopSession(std::string sessionId) {
    std::string url = fmt::format("{}/{}/stop", sessionsEndpoint, sessionId);
    auto res = pimpl->cli->Post(url.c_str());
    if(res->status != 200) {
        spdlog::warn("DeviceGate stopSession not successful - status: {}, error: {}", res->status, res->body);
        return false;
    }
    spdlog::debug("DeviceGate stopSession successful");
    return true;
}

bool DeviceGate::USBImpl::stopSession(std::string sessionId) {
    USBRequest_t request;
    request.RequestNum = STOP_SESSION;
    request.RequestSize = sessionId.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(XLinkGateWrite((void*)sessionId.c_str(), sessionId.size(), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::error("DeviceGate stopSession not successful - got no response");
        return false;
    }
    spdlog::debug("DeviceGate stopSession successful");
    return true;
}

bool DeviceGate::destroySession() {
    if(getState() == SessionState::DESTROYED) {
        spdlog::debug("DeviceGate trying to destroy already destroyed session");
        return true;
    }

    if(getState() == SessionState::NOT_CREATED) {
        spdlog::debug("No need to destroy a session that wasn't created.");
        return true;
    }
    return impl->destroySession(sessionId);
}

bool DeviceGate::HTTPImpl::destroySession(std::string sessionId) {
    std::string url = fmt::format("{}/{}/destroy", sessionsEndpoint, sessionId);
    auto res = pimpl->cli->Post(url.c_str());
    if(res->status != 200) {
        spdlog::warn("DeviceGate destroySession not successful - status: {}, error: {}", res->status, res->body);
        return false;
    }
    spdlog::debug("DeviceGate destroySession successful");
    return true;
}

bool DeviceGate::USBImpl::destroySession(std::string sessionId) {
    USBRequest_t request;
    request.RequestNum = DESTROY_SESSION;
    request.RequestSize = sessionId.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(XLinkGateWrite((void*)sessionId.c_str(), sessionId.size(), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::error("DeviceGate destroySession not successful - got no response");
        return false;
    }
    spdlog::debug("DeviceGate destroySession successful");
    return true;
}

bool DeviceGate::deleteSession() {
    if(getState() == SessionState::NOT_CREATED) {
        spdlog::debug("No need to delete a session that wasn't created.");
        return true;
    }
    return impl->deleteSession(sessionId);
}

bool DeviceGate::HTTPImpl::deleteSession(std::string sessionId) {
    std::string url = fmt::format("{}/{}", sessionsEndpoint, sessionId);
    auto res = pimpl->cli->Delete(url.c_str());
    if(res->status != 200) {
        spdlog::warn("DeviceGate deleteSession not successful - status: {}, error: {}", res->status, res->body);
        return false;
    }
    spdlog::debug("DeviceGate deleteSession successful");
    return true;
}

bool DeviceGate::USBImpl::deleteSession(std::string sessionId) {
    USBRequest_t request;
    request.RequestNum = DELETE_SESSION;
    request.RequestSize = sessionId.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(XLinkGateWrite((void*)sessionId.c_str(), sessionId.size(), timeout) == X_LINK_ERROR) {
        return false;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return false;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::error("DeviceGate deleteSession not successful - got no response");
        return false;
    }
    spdlog::debug("DeviceGate deleteSession successful");
    return true;
}

DeviceGate::SessionState DeviceGate::getState() {
    if(!sessionCreated) {
        spdlog::debug("Session not yet created - can't get the session state from gate");
        return SessionState::NOT_CREATED;
    }
    return impl->getState(sessionId);
}

DeviceGate::SessionState DeviceGate::HTTPImpl::getState(std::string sessionId) {
    auto sessionState = SessionState::CREATED;
    std::string url = fmt::format("{}/{}", sessionsEndpoint, sessionId);

    auto res = pimpl->cli->Get(url.c_str());

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
}

DeviceGate::SessionState DeviceGate::USBImpl::getState(std::string sessionId) {
    auto sessionState = SessionState::CREATED;
    USBRequest_t request;
    request.RequestNum = GET_STATE;
    request.RequestSize = sessionId.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return SessionState::ERROR_STATE;
    }
    if(XLinkGateWrite((void*)sessionId.c_str(), sessionId.size(), timeout) == X_LINK_ERROR) {
        return SessionState::ERROR_STATE;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return SessionState::ERROR_STATE;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::warn("DeviceGate getState not successful - got no response");
        return SessionState::ERROR_STATE;
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return SessionState::ERROR_STATE;
    }
    respBuffer[request.RequestSize] = '\0';
    auto resp = nlohmann::json::parse(respBuffer);
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
}

std::optional<std::vector<uint8_t>> DeviceGate::getFile(const std::string& fileUrl, std::string& filename) {
    return impl->getFile(fileUrl, filename);
}

std::optional<std::vector<uint8_t>> DeviceGate::HTTPImpl::getFile(const std::string& fileUrl, std::string& filename) {
    auto res = pimpl->cli->Get(fileUrl.c_str());
    if(res->status == 200) {
        filename = res->get_header_value("X-Filename");
        // Convert the response body to a vector of uint8_t
        std::vector<uint8_t> fileData(res->body.begin(), res->body.end());

        spdlog::debug("File download successful. Filename: {}", filename);
        return fileData;
    } else {
        spdlog::warn("File download not successful - status: {}, error: {}", res->status, res->body);
        return std::nullopt;
    }
}

std::optional<std::vector<uint8_t>> DeviceGate::USBImpl::getFile(const std::string& fileUrl, std::string& filename) {
    USBRequest_t request;
    request.RequestNum = GET_FILE;
    request.RequestSize = fileUrl.size();
    if(XLinkGateWrite(&request, sizeof(USBRequest_t), timeout) == X_LINK_ERROR) {
        return std::nullopt;
    }
    if(XLinkGateWrite((void*)fileUrl.c_str(), fileUrl.size(), timeout) == X_LINK_ERROR) {
        return std::nullopt;
    }

    if(XLinkGateRead(&request, sizeof(request), timeout) == X_LINK_ERROR) {
        return std::nullopt;
    }
    if(request.RequestNum == RESPONSE_ERROR) {
        spdlog::warn("File download not successful - got no response");
        return std::nullopt;
    }

    std::vector<uint8_t> respBuffer;
    respBuffer.resize(request.RequestSize + 1);
    if(XLinkGateRead(&respBuffer[0], request.RequestSize, timeout) == X_LINK_ERROR) {
        return std::nullopt;
    }
    respBuffer[request.RequestSize] = '\0';
    auto resp = nlohmann::json::parse(respBuffer);

    filename = resp["filename"].get<std::string>();
    std::vector<uint8_t> fileData(resp["data"].get<std::vector<uint8_t>>());

    spdlog::debug("File download successful. Filename: {}", filename);
    return fileData;
}

std::optional<DeviceGate::CrashDump> DeviceGate::waitForSessionEnd() {
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto sessionState = getState();
        if(sessionState == SessionState::ERROR_STATE) {
            spdlog::error("DeviceGate session state is in error state - exiting");
            return std::nullopt;
        }
        switch(sessionState) {
            case SessionState::NOT_CREATED:
            case SessionState::CREATED:
            case SessionState::RUNNING:
            case SessionState::STOPPING:
                break;  // Nothing to do
            case SessionState::ERROR_STATE:
                spdlog::error("DeviceGate session state is in error state - exiting");
                return std::nullopt;
            case SessionState::STOPPED:
                return std::nullopt;
            case SessionState::CRASHED:
            case SessionState::DESTROYED:
                auto crashDumpPathStr = utility::getEnvAs<std::string>("DEPTHAI_CRASHDUMP", "");
                if(crashDumpPathStr == "0") {
                    spdlog::warn("Firmware crashed but DEPTHAI_CRASHDUMP is set to 0, the crash dump will not be saved.");
                    return std::nullopt;
                }

                auto currentVersion = getVersion();
                auto requiredVersion = Version(0, 0, 14);
                if(currentVersion < requiredVersion) {
                    spdlog::warn("FW crashed but the gate version does not support transfering over the crash dump. Current version {}, required is {}",
                                 currentVersion.toString(),
                                 requiredVersion.toString());
                    return std::nullopt;
                }
                spdlog::warn("FW crashed - trying to get out the crash dump");
                std::this_thread::sleep_for(std::chrono::seconds(3));  // Allow for the generation of the crash dump and the log file
                std::string crashDumpName;
                spdlog::warn("Getting the crash dump out - this can take up to a minute, because it first needs to be compressed.");
                return getCrashDump();
        }
    }
}

std::optional<DeviceGate::CrashDump> DeviceGate::getCrashDump() {
    std::string url = fmt::format("{}/{}/core_dump", sessionsEndpoint, sessionId);
    std::string filename;
    auto fileData = DeviceGate::getFile(url, filename);
    if(fileData) {
        return CrashDump{std::move(*fileData), filename};
    }
    return std::nullopt;
}

// TODO(themarpe) - get all sessions, check if only one and not protected
bool DeviceGate::isBootedNonExclusive() {
    return true;
}

}  // namespace dai
