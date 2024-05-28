#include "LogCollection.hpp"

#include <cpr/cpr.h>
#include <cryptopp/filters.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <system_error>

#include "utility/Environment.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace logCollection {

constexpr auto LOG_ENDPOINT = "https://logs.luxonis.com/logs";

struct FileWithSHA1 {
    std::string content;
    std::string sha1Hash;
};

std::string calculateSHA1(const std::string& input) {
    CryptoPP::SHA1 sha1;
    std::string digest;

    CryptoPP::StringSource ss(input, true, new CryptoPP::HashFilter(sha1, new CryptoPP::HexEncoder(new CryptoPP::StringSink(digest), false)));

    return digest;
}

bool sendLogsToServer(const std::optional<FileWithSHA1>& pipelineData, const std::optional<FileWithSHA1>& crashDumpData) {
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

    auto response = cpr::Post(cpr::Url{LOG_ENDPOINT}, multipart);
    if(response.status_code != 200) {
        logger::error("Failed to send logs, status code: {}", response.status_code);
        return false;
    }

    logger::info("Logs sent successfully");
    return true;
}


void logPipeline(const Pipeline& pipeline) {
    logPipeline(pipeline.getPipelineSchema());
}

void logPipeline(const PipelineSchema& pipelineSchema) {
    // Check if logging is explicitly disabled
    auto loggingEnabled = utility::getEnv("DEPTHAI_DISABLE_FEEDBACK");
    if(!loggingEnabled.empty()) {
        logger::info("Logging disabled");
        return;
    }
    auto pipelineJson = nlohmann::json(pipelineSchema);
    std::string pipelineJsonStr = pipelineJson.dump();
    std::string pipelineSHA1 = calculateSHA1(pipelineJsonStr);

    std::filesystem::path pipelineDir = std::filesystem::current_path() / ".cache" / "depthai" / "pipelines";
    std::filesystem::path pipelinePath = pipelineDir / pipelineSHA1 / "pipeline.json";

    if(std::filesystem::exists(pipelinePath)) {
        logger::info("Pipeline already logged");
        return;
    }

    logger::info("Pipeline not logged yet, logging...");
    std::error_code ec;
    std::filesystem::create_directories(pipelinePath.parent_path(), ec);
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
    auto success = sendLogsToServer(pipelineData, std::nullopt);
    if(!success) {
        // Keep at info level to not spam in case of no internet connection
        logger::info("Failed to send pipeline logs to server");
    } else {
        logger::info("Pipeline logs sent to server");
    }
}

void logCrashDump(const std::optional<PipelineSchema>& pipelineSchema, const CrashDump& crashDump) {
    // Check if logging is explicitly disabled
    auto loggingEnabled = utility::getEnv("DEPTHAI_DISABLE_FEEDBACK");
    if(!loggingEnabled.empty()) {
        logger::info("Logging disabled");
        return;
    }

    std::string crashDumpJson = crashDump.serializeToJson().dump();
    std::string crashDumpSHA1 = calculateSHA1(crashDumpJson);
    std::filesystem::path logDir = std::filesystem::current_path() / ".cache" / "depthai" / "crashdumps";
    auto crashDumpPath = utility::getEnv("DEPTHAI_CRASHDUMP");
    std::filesystem::path crashDumpPathLocal;
    if(crashDumpPath.empty()) {
        crashDumpPathLocal = logDir / crashDumpSHA1 / "crash_dump.json";
    } else {
        crashDumpPathLocal = crashDumpPath;
    }

    std::error_code ec;
    std::filesystem::create_directories(crashDumpPathLocal.parent_path(), ec);
    if(ec) {
        logger::error("Failed to create log directory: {}", ec.message());
        return;
    }

    std::ofstream crashDumpFile(crashDumpPathLocal);
    crashDumpFile << crashDumpJson;
    crashDumpFile.close();
    logger::error("Logged to local file: {}", crashDumpPathLocal.string());

    FileWithSHA1 crashDumpData;
    crashDumpData.content = std::move(crashDumpJson);
    crashDumpData.sha1Hash = std::move(crashDumpSHA1);

    std::optional<FileWithSHA1> pipelineData;
    if(pipelineSchema) {
        pipelineData = FileWithSHA1{};
        std::string pipelineJson = nlohmann::json(*pipelineSchema).dump();
        std::string pipelineSHA1 = calculateSHA1(pipelineJson);
        pipelineData->content = std::move(pipelineJson);
        pipelineData->sha1Hash = std::move(pipelineSHA1);
    }
    auto success = sendLogsToServer(pipelineData, crashDumpData);
    if(!success) {
        // Keep at info level to not spam in case of no internet connection
        logger::error("Failed to send crash dump logs to the server.");
        logger::error("Crash logs are stored in: {} - please report to developers.", crashDumpPathLocal.string());
    } else {
        logger::warn("Device has crashed. Crash dump logs sent to server.");
    }
}

}  // namespace logCollection
}  // namespace dai
