#include "LogCollection.hpp"

#include <XLink/XLinkPublicDefines.h>
#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/cpr.h>
#endif

#include <ghc/filesystem.hpp>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <system_error>

#include "build/version.hpp"
#include "sha1.hpp"
#include "utility/Environment.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace logCollection {

constexpr auto LOG_ENDPOINT = "https://logs.luxonis.com/logs";

struct FileWithSHA1 {
    std::string content;
    std::string sha1Hash;
};

std::string platformToString(XLinkPlatform_t platform) {
    switch(platform) {
        case X_LINK_ANY_PLATFORM:
            return "X_LINK_ANY_PLATFORM";
        case X_LINK_MYRIAD_X:
            return "X_LINK_MYRIAD_X";
        case X_LINK_MYRIAD_2:
            return "X_LINK_MYRIAD_2";
        default:
            return "INVALID_ENUM_VALUE";
    }
}

std::string protocolToString(XLinkProtocol_t protocol) {
    switch(protocol) {
        case X_LINK_USB_VSC:
            return "X_LINK_USB_VSC";
        case X_LINK_USB_CDC:
            return "X_LINK_USB_CDC";
        case X_LINK_PCIE:
            return "X_LINK_PCIE";
        case X_LINK_IPC:
            return "X_LINK_IPC";
        case X_LINK_TCP_IP:
            return "X_LINK_TCP_IP";
        case X_LINK_NMB_OF_PROTOCOLS:
            return "X_LINK_NMB_OF_PROTOCOLS";
        case X_LINK_ANY_PROTOCOL:
            return "X_LINK_ANY_PROTOCOL";
        default:
            return "INVALID_ENUM_VALUE";
    }
}

std::string getOSPlatform() {
#ifdef _WIN32
    return "Windows";
#elif __APPLE__
    return "MacOS";
#elif __linux__
    return "Linux";
#else
    return "Other";
#endif
}

std::string calculateSHA1(const std::string& input) {
    // We could also use SHA1 from OpenSSL and SChannel
    SHA1 checksum;
    checksum.update(input);
    return checksum.final();
}

#ifdef DEPTHAI_ENABLE_CURL
bool sendLogsToServer(const tl::optional<FileWithSHA1>& pipelineData, const tl::optional<FileWithSHA1>& crashDumpData, const dai::DeviceInfo& deviceInfo) {
    (void)deviceInfo;  // Unused for now
    // At least one of the files must be present
    if(!pipelineData && !crashDumpData) {
        logger::error("Incorrect usage of sendLogsToServer, at least one of the files must be present");
        return false;
    }
    cpr::Multipart multipart{};
    if(pipelineData) {
        cpr::Buffer pipelineBuffer(pipelineData->content.begin(), pipelineData->content.end(), "pipeline.json");
        multipart.parts.emplace_back("pipelineFile", pipelineBuffer);
        multipart.parts.emplace_back("pipelineId", pipelineData->sha1Hash);
    }

    if(crashDumpData) {
        cpr::Buffer crashDumpBuffer(crashDumpData->content.begin(), crashDumpData->content.end(), "crash_dump.json");
        multipart.parts.emplace_back("crashDumpFile", crashDumpBuffer);
        multipart.parts.emplace_back("crashDumpId", crashDumpData->sha1Hash);
    }

    multipart.parts.emplace_back("platform", platformToString(deviceInfo.platform));
    multipart.parts.emplace_back("connectionType", protocolToString(deviceInfo.protocol));
    multipart.parts.emplace_back("osPlatform", getOSPlatform());
    std::string daiVersion = fmt::format("{}-{}", build::VERSION, build::COMMIT);
    multipart.parts.emplace_back("depthAiVersion", std::move(daiVersion));
    multipart.parts.emplace_back("productId", deviceInfo.getMxId());
    auto response = cpr::Post(cpr::Url{LOG_ENDPOINT}, multipart);
    if(response.status_code != 200) {
        logger::info("Failed to send logs, status code: {}", response.status_code);
        return false;
    }

    logger::info("Logs sent successfully");
    return true;
}
#else
bool sendLogsToServer(const tl::optional<FileWithSHA1>&, const tl::optional<FileWithSHA1>&, const dai::DeviceInfo&) {
    logger::info("Not sending the logs to the server, as CURL support is disabled");
    return false;
}
#endif

void logPipeline(const PipelineSchema& pipelineSchema, const dai::DeviceInfo& deviceInfo) {
    // Check if compiled without CURL support and exit early if so
#ifndef DEPTHAI_ENABLE_CURL
    (void)pipelineSchema;
    (void)deviceInfo;
    logger::info("Compiled without CURL support, not logging pipeline.");
#else
    namespace fs = ghc::filesystem;
    // Check if logging is explicitly disabled
    auto loggingEnabled = utility::getEnv("DEPTHAI_ENABLE_FEEDBACK_PIPELINE");
    if(loggingEnabled.empty()) {
        logger::info("Logging disabled");
        return;
    }

    auto pipelineJson = nlohmann::json(pipelineSchema);
    std::string pipelineJsonStr = pipelineJson.dump();
    std::string pipelineSHA1 = calculateSHA1(pipelineJsonStr);

    fs::path pipelineDir = fs::current_path() / ".cache" / "depthai" / "pipelines";
    fs::path pipelinePath = pipelineDir / pipelineSHA1 / "pipeline.json";

    if(fs::exists(pipelinePath)) {
        logger::info("Pipeline already logged");
        return;
    }

    logger::info("Pipeline not logged yet, logging...");
    std::error_code ec;
    fs::create_directories(pipelinePath.parent_path(), ec);
    if(ec) {
        logger::error("Failed to create log directory: {}", ec.message());
        return;
    }

    std::ofstream pipelineFile(pipelinePath);
    pipelineFile << pipelineJsonStr;
    pipelineFile.close();

    FileWithSHA1 pipelineData;
    pipelineData.content = std::move(pipelineJsonStr);
    pipelineData.sha1Hash = std::move(pipelineSHA1);
    auto success = sendLogsToServer(pipelineData, tl::nullopt, deviceInfo);
    if(!success) {
        // Keep at info level to not spam in case of no internet connection
        logger::info("Failed to send pipeline logs to server");
    } else {
        logger::info("Pipeline logs sent to server");
    }
#endif
}

void logCrashDump(const tl::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump, const dai::DeviceInfo& deviceInfo) {
    namespace fs = ghc::filesystem;
    std::string crashDumpJson = crashDump.serializeToJson().dump();
    std::string crashDumpHash = calculateSHA1(crashDumpJson);
    fs::path logDir = fs::current_path() / ".cache" / "depthai" / "crashdumps";
    auto crashDumpPath = utility::getEnv("DEPTHAI_CRASHDUMP");
    fs::path crashDumpPathLocal;
    if(crashDumpPath.empty()) {
        crashDumpPathLocal = logDir / crashDumpHash / "crash_dump.json";
    } else {
        crashDumpPathLocal = crashDumpPath;
    }
    auto errorString = fmt::format(
        "Device with id {} has crashed. Crash dump logs are stored in: {} - please report to developers.", deviceInfo.getMxId(), crashDumpPathLocal.string());

    std::error_code ec;
    fs::create_directories(crashDumpPathLocal.parent_path(), ec);
    if(ec) {
        logger::error("Failed to create log directory: {}", ec.message());
        return;
    }

    std::ofstream crashDumpFile(crashDumpPathLocal);
    crashDumpFile << crashDumpJson;
    crashDumpFile.close();
    logger::error(errorString);
    // Send logs to the server if possible
#ifdef DEPTHAI_ENABLE_CURL

    FileWithSHA1 crashDumpData;
    crashDumpData.content = std::move(crashDumpJson);
    crashDumpData.sha1Hash = calculateSHA1(crashDumpJson);

    tl::optional<FileWithSHA1> pipelineData;
    if(pipelineSchema) {
        pipelineData = FileWithSHA1{};
        std::string pipelineJson = nlohmann::json(*pipelineSchema).dump();
        std::string pipelineSHA1 = calculateSHA1(pipelineJson);
        pipelineData->content = std::move(pipelineJson);
        pipelineData->sha1Hash = std::move(pipelineSHA1);
    }

    // Check if logging is explicitly disabled
    auto loggingEnabled = utility::getEnv("DEPTHAI_ENABLE_FEEDBACK_CRASHDUMP");
    if(!loggingEnabled.empty()) {
        logger::info("Logging enabled");
        auto success = sendLogsToServer(pipelineData, crashDumpData, deviceInfo);
        if(!success) {
            // Keep at info level to not spam in case of no internet connection
            logger::warn("Failed to send crash dump logs to the server.");
        }
    } else {
        logger::info("Logging disabled");
    }
#else
    (void)pipelineSchema;
#endif
}

}  // namespace logCollection
}  // namespace dai
