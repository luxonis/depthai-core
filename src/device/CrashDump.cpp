#include "depthai/device/CrashDump.hpp"

// std
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>
#include <thread>

// project
#include "depthai/utility/Compression.hpp"

// third party
#include <nlohmann/json.hpp>

namespace dai {

// Crashdump constants for tar file structure
static constexpr const char* METADATA_FILENAME = "metadata.json";
static constexpr const char* EXTRA_FILENAME = "extra.json";
static constexpr const char* RVC2_DATA_FILENAME = "crash_reports.json";
static constexpr const char* RVC4_DATA_FILENAME = "crash_dump.tar.gz";

namespace fs = std::filesystem;

// Helper function to write string to a temporary file and return the path
static fs::path writeToTempFile(const std::string& content, std::string_view suffix) {
    auto tempPath = fs::temp_directory_path()
                    / (std::string("crashdump_") + std::string(suffix) + "_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    std::ofstream out(tempPath, std::ios::binary);
    if(!out) {
        throw std::runtime_error("Failed to create temporary file: " + tempPath.string());
    }
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    out.close();
    return tempPath;
}

// Helper function to write binary data to a temporary file and return the path
static fs::path writeToTempFile(const std::vector<uint8_t>& data, std::string_view suffix) {
    auto tempPath = fs::temp_directory_path()
                    / (std::string("crashdump_") + std::string(suffix) + "_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    std::ofstream out(tempPath, std::ios::binary);
    if(!out) {
        throw std::runtime_error("Failed to create temporary file: " + tempPath.string());
    }
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    out.close();
    return tempPath;
}

// Helper function to read file content as string
static std::string readFileAsString(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if(!in) {
        throw std::runtime_error("Failed to open file: " + path.string());
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Helper function to read file content as binary
static std::vector<uint8_t> readFileAsBinary(const fs::path& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if(!in) {
        throw std::runtime_error("Failed to open file: " + path.string());
    }
    auto size = in.tellg();
    in.seekg(0);
    std::vector<uint8_t> data(size);
    in.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

// Helper function to get value from json or return default value
static std::string valueOr(const nlohmann::json& json, const std::string& key, const std::string& defaultValue) {
    if(json.contains(key) && !json[key].is_null()) {
        return json[key].get<std::string>();
    }
    return defaultValue;
}

// Factory method implementation
std::unique_ptr<CrashDump> CrashDump::fromTarFile(const fs::path& tarPath) {
    // First, extract metadata to determine platform
    auto tempDir = fs::temp_directory_path() / ("crashdump_extract_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    fs::create_directories(tempDir);

    auto metadataPath = tempDir / METADATA_FILENAME;
    utility::untarFiles(tarPath, {METADATA_FILENAME}, {metadataPath});

    if(!fs::exists(metadataPath)) {
        fs::remove_all(tempDir);
        throw std::runtime_error("Invalid crash dump tar: missing metadata.json");
    }

    std::string metadataContent = readFileAsString(metadataPath);
    nlohmann::json metadata = nlohmann::json::parse(metadataContent);

    fs::remove_all(tempDir);

    if(!metadata.contains("platform")) {
        throw std::runtime_error("Invalid crash dump metadata: missing platform field");
    }

    Platform platform = string2platform(metadata["platform"].get<std::string>());

    std::unique_ptr<CrashDump> result;
    switch(platform) {
        case Platform::RVC2:
            result = std::make_unique<CrashDumpRVC2>(tarPath);
            break;
        case Platform::RVC4:
            result = std::make_unique<CrashDumpRVC4>(tarPath);
            break;
        case Platform::RVC3:
        default:
            throw std::runtime_error("Unsupported platform in crash dump: " + platform2string(platform));
    }

    return result;
}

std::vector<uint8_t> CrashDump::toBytes() const {
    fs::path tempPath = fs::temp_directory_path() / ("crashdump_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    toTar(tempPath);
    std::vector<uint8_t> bytes = readFileAsBinary(tempPath);
    fs::remove(tempPath);
    return bytes;
}

std::unique_ptr<CrashDump> CrashDump::fromBytes(const std::vector<uint8_t>& bytes) {
    fs::path tempPath = fs::temp_directory_path() / ("crashdump_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    std::ofstream out(tempPath, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    out.close();
    auto crashDump = fromTarFile(tempPath);
    fs::remove(tempPath);
    return crashDump;
}

nlohmann::json& CrashDump::operator[](const std::string& key) {
    return this->extra[key];
}

const nlohmann::json& CrashDump::operator[](const std::string& key) const {
    return this->extra.at(key);
}

void CrashDump::writeMetadata(nlohmann::json& json) const {
    // basic
    json["platform"] = platform2string(getPlatform());
    json["crashdumpVersion"] = getCrashDumpVersion();

    // depthai
    json["depthaiVersion"] = depthaiVersion;
    json["depthaiVersionMajor"] = depthaiVersionMajor;
    json["depthaiVersionMinor"] = depthaiVersionMinor;
    json["depthaiVersionPatch"] = depthaiVersionPatch;
    json["depthaiVersionPreReleaseType"] = depthaiVersionPreReleaseType;
    json["depthaiVersionPreReleaseVersion"] = depthaiVersionPreReleaseVersion;
    json["depthaiVersionBuildInfo"] = depthaiVersionBuildInfo;
    json["depthaiCommitHash"] = depthaiCommitHash;
    json["depthaiCommitDatetime"] = depthaiCommitDatetime;
    json["depthaiBuildDatetime"] = depthaiBuildDatetime;
    json["depthaiDeviceVersion"] = depthaiDeviceVersion;
    json["depthaiBootloaderVersion"] = depthaiBootloaderVersion;
    json["depthaiDeviceRVC3Version"] = depthaiDeviceRVC3Version;
    json["depthaiDeviceRVC4Version"] = depthaiDeviceRVC4Version;

    // device
    json["crashdumpTimestamp"] = crashdumpTimestamp;
    json["deviceId"] = deviceId;
}

void CrashDump::readMetadata(const nlohmann::json& json) {
    // depthai
    depthaiVersion = valueOr(json, "depthaiVersion", "");
    depthaiVersionMajor = valueOr(json, "depthaiVersionMajor", "");
    depthaiVersionMinor = valueOr(json, "depthaiVersionMinor", "");
    depthaiVersionPatch = valueOr(json, "depthaiVersionPatch", "");
    depthaiVersionPreReleaseType = valueOr(json, "depthaiVersionPreReleaseType", "");
    depthaiVersionPreReleaseVersion = valueOr(json, "depthaiVersionPreReleaseVersion", "");
    depthaiVersionBuildInfo = valueOr(json, "depthaiVersionBuildInfo", "");
    depthaiCommitHash = valueOr(json, "depthaiCommitHash", "");
    depthaiCommitDatetime = valueOr(json, "depthaiCommitDatetime", "");
    depthaiBuildDatetime = valueOr(json, "depthaiBuildDatetime", "");
    depthaiDeviceVersion = valueOr(json, "depthaiDeviceVersion", "");
    depthaiBootloaderVersion = valueOr(json, "depthaiBootloaderVersion", "");
    depthaiDeviceRVC3Version = valueOr(json, "depthaiDeviceRVC3Version", "");
    depthaiDeviceRVC4Version = valueOr(json, "depthaiDeviceRVC4Version", "");

    // device
    crashdumpTimestamp = valueOr(json, "crashdumpTimestamp", "");
    deviceId = valueOr(json, "deviceId", "");
}

CrashDumpRVC2::CrashDumpRVC2(const fs::path& tarFile) {
    fromTar(tarFile);
}

void CrashDumpRVC2::toTar(const fs::path& tarPath) const {
    // Create a metadata json
    nlohmann::json metadata;
    writeMetadata(metadata);

    // Write to temporary files
    constexpr size_t DEFAULT_INDENT = 2;
    auto metadataPath = writeToTempFile(metadata.dump(DEFAULT_INDENT), "metadata");
    auto extraPath = writeToTempFile(extra.dump(DEFAULT_INDENT), "extra");
    auto rvc2dataPath = writeToTempFile(nlohmann::json(crashReports).dump(DEFAULT_INDENT), "data");

    // Combine everything into a tar file
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, rvc2dataPath};
    std::vector<std::string> filesInTar = {METADATA_FILENAME, EXTRA_FILENAME, RVC2_DATA_FILENAME};
    utility::tarFiles(tarPath, filesOnDisk, filesInTar);

    // Cleanup temporary files
    fs::remove(metadataPath);
    fs::remove(extraPath);
    fs::remove(rvc2dataPath);
}

void CrashDumpRVC2::fromTar(const fs::path& tarPath) {
    auto tempDir = fs::temp_directory_path() / ("crashdump_rvc2_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    fs::create_directories(tempDir);

    auto metadataPath = tempDir / METADATA_FILENAME;
    auto extraPath = tempDir / EXTRA_FILENAME;
    auto dataPath = tempDir / RVC2_DATA_FILENAME;

    // Extract files
    std::vector<std::string> filesInTar = {METADATA_FILENAME, EXTRA_FILENAME, RVC2_DATA_FILENAME};
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, dataPath};
    utility::untarFiles(tarPath, filesInTar, filesOnDisk);

    // Read metadata
    if(!fs::exists(metadataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing metadata file");
    }
    nlohmann::json metadata = nlohmann::json::parse(readFileAsString(metadataPath));
    readMetadata(metadata);

    // Read extra
    if(fs::exists(extraPath)) {
        extra = nlohmann::json::parse(readFileAsString(extraPath));
        if(extra.is_null()) {  // json was not parsed successfully
            extra = nlohmann::json::object();
        }
    } else {
        extra = nlohmann::json::object();
    }

    // Read crash reports data
    if(!fs::exists(dataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing crash_reports.json");
    }
    nlohmann::json data = nlohmann::json::parse(readFileAsString(dataPath));
    crashReports = data.get<CrashReportCollection>();

    // Cleanup
    fs::remove_all(tempDir);
}

// CrashDumpRVC4 implementation

CrashDumpRVC4::CrashDumpRVC4(const fs::path& tarFile) {
    fromTar(tarFile);
}

void CrashDumpRVC4::toTar(const fs::path& tarPath) const {
    // Create metadata
    nlohmann::json metadata;
    writeMetadata(metadata);
    metadata["originalFilename"] = filename;

    // Write temporary files
    constexpr size_t DEFAULT_INDENT = 2;
    auto metadataPath = writeToTempFile(metadata.dump(DEFAULT_INDENT), "metadata");
    auto extraPath = writeToTempFile(extra.dump(DEFAULT_INDENT), "extra");
    auto rvc4dataPath = writeToTempFile(data, "data");

    // Create tar
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, rvc4dataPath};
    std::vector<std::string> filesInTar = {METADATA_FILENAME, EXTRA_FILENAME, RVC4_DATA_FILENAME};
    utility::tarFiles(tarPath, filesOnDisk, filesInTar);

    // Cleanup temporary files
    fs::remove(metadataPath);
    fs::remove(extraPath);
    fs::remove(rvc4dataPath);
}

void CrashDumpRVC4::fromTar(const fs::path& tarPath) {
    auto tempDir = fs::temp_directory_path() / ("crashdump_rvc4_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    fs::create_directories(tempDir);

    auto metadataPath = tempDir / METADATA_FILENAME;
    auto extraPath = tempDir / EXTRA_FILENAME;
    auto dataPath = tempDir / RVC4_DATA_FILENAME;

    // Extract files
    std::vector<std::string> filesInTar = {METADATA_FILENAME, EXTRA_FILENAME, RVC4_DATA_FILENAME};
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, dataPath};
    utility::untarFiles(tarPath, filesInTar, filesOnDisk);

    // Read metadata
    if(!fs::exists(metadataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing metadata file");
    }
    nlohmann::json metadata = nlohmann::json::parse(readFileAsString(metadataPath));
    readMetadata(metadata);

    // Read extra
    if(fs::exists(extraPath)) {
        extra = nlohmann::json::parse(readFileAsString(extraPath));
        if(extra.is_null()) {  // json was not parsed successfully
            extra = nlohmann::json::object();
        }
    } else {
        extra = nlohmann::json::object();
    }

    // Read binary data
    if(!fs::exists(dataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing crash_dump.tar.gz");
    }
    data = readFileAsBinary(dataPath);

    // Cleanup
    fs::remove_all(tempDir);
}

}  // namespace dai
