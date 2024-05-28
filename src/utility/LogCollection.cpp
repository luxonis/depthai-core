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

bool sendLogsToServer(const FileWithSHA1& pipelineData, const std::optional<FileWithSHA1>& crashDumpData) {
    cpr::Buffer pipelineBuffer(pipelineData.content.begin(), pipelineData.content.end(), "pipeline.json");

    cpr::Multipart multipart = {{"pipelineFile", pipelineBuffer}, {"pipelineId", pipelineData.sha1Hash}};

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

    std::filesystem::path logDir = std::filesystem::current_path() / ".cache" / "depthai_logs";
    std::filesystem::path pipelinePath = logDir / pipelineSHA1 / "pipeline.json";

    if(std::filesystem::exists(pipelinePath)) {
        logger::info("Pipeline already logged");
        return;
    }

    logger::info("Pipeline not logged yet, logging...");
    std::error_code ec;
    std::filesystem::create_directories(logDir / pipelineSHA1, ec);
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
        logger::error("Failed to send pipeline logs to server");
    } else {
        logger::info("Pipeline logs sent to server");
    }
}

}  // namespace logCollection
}  // namespace dai
