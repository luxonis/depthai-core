#include "depthai/device/CrashDump.hpp"

// std
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>

// project
#include "depthai/utility/Compression.hpp"
#include "utility/Platform.hpp"

// third party
#include <fmt/format.h>
#include <fmt/std.h>

#include <nlohmann/json.hpp>

namespace dai {

// Crashdump constants for tar file structure
static constexpr const char* METADATA_FILENAME = "metadata.json";
static constexpr const char* EXTRA_FILENAME = "extra.json";
static constexpr const char* RVC2_DATA_FILENAME = "crash_reports.json";
static constexpr const char* RVC4_DATA_FILENAME = "crash_dump.tar.gz";

namespace fs = std::filesystem;

static fs::path writeToTempFile(const char* content, std::streamsize size, std::string_view suffix) {
    auto tempPath = platform::getTempPath() / fmt::format("crashdump_{}", suffix);

    // Create file
    std::ofstream out(tempPath, std::ios::binary);
    if(!out) {
        throw std::runtime_error(fmt::format("Failed to create temporary file: {}", tempPath));
    }

    // Write to file and close
    out.write(content, size);
    out.close();
    if(out.fail()) {
        throw std::runtime_error(fmt::format("Failed to write temporary file: {}", tempPath));
    }
    return tempPath;
}

static fs::path writeToTempFile(const std::vector<uint8_t>& data, std::string_view suffix) {
    return writeToTempFile(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()), suffix);
}

static fs::path writeToTempFile(const std::string& content, std::string_view suffix) {
    return writeToTempFile(content.data(), static_cast<std::streamsize>(content.size()), suffix);
}

// Helper function to read file content as string
static std::string readFileAsString(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if(!in) {
        throw std::runtime_error(fmt::format("Failed to open file: {}", path));
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Helper function to read file content as binary
static std::vector<uint8_t> readFileAsBinary(const fs::path& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if(!in) {
        throw std::runtime_error(fmt::format("Failed to open file: {}", path));
    }
    auto size = in.tellg();
    in.seekg(0);
    std::vector<uint8_t> data(size);
    in.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

// Helper function to get value from json or return default value
static std::string fromJson(const nlohmann::json& json, const std::string& key, const std::string& defaultValue) {
    if(json.contains(key) && !json[key].is_null()) {
        return json[key].get<std::string>();
    }
    return defaultValue;
}

// Factory method implementation
std::unique_ptr<CrashDump> CrashDump::load(const fs::path& tarPath) {
    // First, extract metadata to determine platform
    auto tempDir = platform::getTempPath() / "crashdump_extract";
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
    auto tempPath = platform::getTempPath() / "crashdump_tobytes";
    toTar(tempPath);
    std::vector<uint8_t> bytes = readFileAsBinary(tempPath);
    fs::remove(tempPath);
    return bytes;
}

std::unique_ptr<CrashDump> CrashDump::fromBytes(const std::vector<uint8_t>& bytes) {
    auto tempPath = platform::getTempPath() / "crashdump_frombytes";
    std::ofstream out(tempPath, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    out.close();
    if(out.fail()) {
        throw std::runtime_error(fmt::format("Failed to write temporary file: {}", tempPath));
    }
    auto crashDump = load(tempPath);
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
    depthaiVersion = fromJson(json, "depthaiVersion", "");
    depthaiVersionMajor = fromJson(json, "depthaiVersionMajor", "");
    depthaiVersionMinor = fromJson(json, "depthaiVersionMinor", "");
    depthaiVersionPatch = fromJson(json, "depthaiVersionPatch", "");
    depthaiVersionPreReleaseType = fromJson(json, "depthaiVersionPreReleaseType", "");
    depthaiVersionPreReleaseVersion = fromJson(json, "depthaiVersionPreReleaseVersion", "");
    depthaiVersionBuildInfo = fromJson(json, "depthaiVersionBuildInfo", "");
    depthaiCommitHash = fromJson(json, "depthaiCommitHash", "");
    depthaiCommitDatetime = fromJson(json, "depthaiCommitDatetime", "");
    depthaiBuildDatetime = fromJson(json, "depthaiBuildDatetime", "");
    depthaiDeviceVersion = fromJson(json, "depthaiDeviceVersion", "");
    depthaiBootloaderVersion = fromJson(json, "depthaiBootloaderVersion", "");
    depthaiDeviceRVC3Version = fromJson(json, "depthaiDeviceRVC3Version", "");
    depthaiDeviceRVC4Version = fromJson(json, "depthaiDeviceRVC4Version", "");

    // device
    crashdumpTimestamp = fromJson(json, "crashdumpTimestamp", "");
    deviceId = fromJson(json, "deviceId", "");
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
    auto tempDir = platform::getTempPath() / "crashdump_rvc2";
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
    auto tempDir = platform::getTempPath() / "crashdump_rvc4";
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
