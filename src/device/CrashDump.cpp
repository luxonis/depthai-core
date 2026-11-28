#include "depthai/device/CrashDump.hpp"

#include <fmt/format.h>
#include <fmt/std.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string_view>

#include "depthai/utility/Compression.hpp"
#include "utility/Platform.hpp"

namespace dai {

static constexpr const char* METADATA_FILENAME = "metadata.json";
static constexpr const char* EXTRA_FILENAME = "extra.json";
static constexpr const char* RVC2_DATA_FILENAME = "crash_reports.json";
static constexpr const char* RVC4_DATA_FILENAME = "crash_dump.tar.gz";

static constexpr int JSON_INDENT = 2;

namespace fs = std::filesystem;

static void writeToFile(const fs::path& path, const char* content, const size_t size) {
    std::ofstream out(path, std::ios::binary);
    if(!out) {
        throw std::ios_base::failure(fmt::format("Failed to create file: {}", path));
    }
    out.write(content, static_cast<std::streamsize>(size));
    out.close();
    if(out.fail()) {
        throw std::ios_base::failure(fmt::format("Failed to write file: {}", path));
    }
}

static void writeToFile(const fs::path& path, const std::string& content) {
    writeToFile(path, content.data(), content.size());
}

static void writeToFile(const fs::path& path, const std::vector<uint8_t>& data) {
    writeToFile(path, reinterpret_cast<const char*>(data.data()), data.size());
}

static void readFromFile(const fs::path& path, std::string& content) {
    std::ifstream in(path, std::ios::binary);
    if(!in) {
        throw std::ios_base::failure(fmt::format("Failed to open file: {}", path));
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    content = ss.str();
    in.close();
    if(in.fail()) {
        throw std::ios_base::failure(fmt::format("Failed to read file: {}", path));
    }
}

static void readFromFile(const fs::path& path, std::vector<uint8_t>& data) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if(!in) {
        throw std::ios_base::failure(fmt::format("Failed to open file: {}", path));
    }
    auto size = in.tellg();
    in.seekg(0);
    data.resize(size);
    in.read(reinterpret_cast<char*>(data.data()), size);
    in.close();
    if(in.fail()) {
        throw std::ios_base::failure(fmt::format("Failed to read file: {}", path));
    }
}

static std::string fromJson(const nlohmann::json& json, const std::string& key, const std::string& defaultValue) {
    if(json.contains(key) && !json[key].is_null()) {
        return json[key].get<std::string>();
    }
    return defaultValue;
}

std::unique_ptr<CrashDump> CrashDump::load(const fs::path& tarPath) {
    fs::path tempDir = platform::getTempPath();

    fs::path metadataPath = tempDir / METADATA_FILENAME;
    utility::extractFiles(tarPath, {METADATA_FILENAME}, {metadataPath});

    if(!fs::exists(metadataPath)) {
        fs::remove_all(tempDir);
        throw std::runtime_error("Invalid crash dump tar: missing metadata.json");
    }

    std::string metadataContent;
    readFromFile(metadataPath, metadataContent);
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

CrashDump::~CrashDump() = default;

std::vector<uint8_t> CrashDump::toBytes() const {
    fs::path tempPath = platform::getTempPath();
    fs::path tarPath = tempPath / "crashdump.tar";
    toTar(tarPath);
    std::vector<uint8_t> bytes;
    readFromFile(tarPath, bytes);
    fs::remove_all(tempPath);
    return bytes;
}

std::unique_ptr<CrashDump> CrashDump::fromBytes(const std::vector<uint8_t>& bytes) {
    fs::path tempPath = platform::getTempPath();
    fs::path tarPath = tempPath / "crashdump.tar";
    writeToFile(tarPath, bytes);
    auto crashDump = load(tarPath);
    fs::remove_all(tempPath);
    return crashDump;
}

nlohmann::json& CrashDump::operator[](const std::string& key) {
    return this->extra[key];
}

const nlohmann::json& CrashDump::operator[](const std::string& key) const {
    return this->extra.at(key);
}

void CrashDump::writeMetadata(nlohmann::json& json) const {
    json["platform"] = platform2string(getPlatform());
    json["crashdumpVersion"] = getCrashDumpVersion();

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
    json["osPlatform"] = osPlatform;

    json["crashdumpTimestamp"] = crashdumpTimestamp;
    json["deviceId"] = deviceId;
}

void CrashDump::readMetadata(const nlohmann::json& json) {
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
    osPlatform = fromJson(json, "osPlatform", "");

    crashdumpTimestamp = fromJson(json, "crashdumpTimestamp", "");
    deviceId = fromJson(json, "deviceId", "");
}

CrashDumpRVC2::CrashDumpRVC2(const fs::path& tarFile) {
    fromTar(tarFile);
}

void CrashDumpRVC2::toTar(const fs::path& tarPath) const {
    nlohmann::json metadata;
    writeMetadata(metadata);

    fs::path tempFolder = platform::getTempPath();

    fs::path metadataPath = tempFolder / METADATA_FILENAME;
    fs::path extraPath = tempFolder / EXTRA_FILENAME;
    fs::path rvc2dataPath = tempFolder / RVC2_DATA_FILENAME;

    writeToFile(metadataPath, metadata.dump(JSON_INDENT));
    writeToFile(extraPath, extra.dump(JSON_INDENT));

    writeToFile(rvc2dataPath, nlohmann::json(crashReports).dump(JSON_INDENT, ' ', false, nlohmann::json::error_handler_t::replace));  // Not always valid UTF-8

    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, rvc2dataPath};
    std::vector<std::string> filesInArchive = {METADATA_FILENAME, EXTRA_FILENAME, RVC2_DATA_FILENAME};
    utility::archiveFilesCompressed(tarPath, filesOnDisk, filesInArchive);

    fs::remove_all(tempFolder);
}

void CrashDumpRVC2::fromTar(const fs::path& tarPath) {
    fs::path tempDir = platform::getTempPath();

    fs::path metadataPath = tempDir / METADATA_FILENAME;
    fs::path extraPath = tempDir / EXTRA_FILENAME;
    fs::path dataPath = tempDir / RVC2_DATA_FILENAME;

    std::vector<std::string> filesInArchive = {METADATA_FILENAME, EXTRA_FILENAME, RVC2_DATA_FILENAME};
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, dataPath};
    utility::extractFiles(tarPath, filesInArchive, filesOnDisk);

    if(!fs::exists(metadataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing metadata file");
    }
    std::string metadataContent;
    readFromFile(metadataPath, metadataContent);
    nlohmann::json metadata = nlohmann::json::parse(metadataContent);
    readMetadata(metadata);

    if(fs::exists(extraPath)) {
        std::string extraContent;
        readFromFile(extraPath, extraContent);
        extra = nlohmann::json::parse(extraContent);
        if(extra.is_null()) {  // json was null
            extra = nlohmann::json::object();
        }
    } else {
        extra = nlohmann::json::object();
    }

    if(!fs::exists(dataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing crash_reports.json");
    }
    std::string dataContent;
    readFromFile(dataPath, dataContent);
    nlohmann::json data = nlohmann::json::parse(dataContent);
    crashReports = data.get<CrashReportCollection>();

    fs::remove_all(tempDir);
}

CrashDumpRVC4::CrashDumpRVC4(const fs::path& tarFile) {
    fromTar(tarFile);
}

void CrashDumpRVC4::toTar(const fs::path& tarPath) const {
    nlohmann::json metadata;
    writeMetadata(metadata);
    metadata["originalFilename"] = filename;

    fs::path tempFolder = platform::getTempPath();

    fs::path metadataPath = tempFolder / METADATA_FILENAME;
    fs::path extraPath = tempFolder / EXTRA_FILENAME;
    fs::path rvc4dataPath = tempFolder / RVC4_DATA_FILENAME;

    writeToFile(metadataPath, metadata.dump(JSON_INDENT));
    writeToFile(extraPath, extra.dump(JSON_INDENT));
    writeToFile(rvc4dataPath, data);

    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, rvc4dataPath};
    std::vector<std::string> filesInArchive = {METADATA_FILENAME, EXTRA_FILENAME, RVC4_DATA_FILENAME};
    utility::archiveFilesCompressed(tarPath, filesOnDisk, filesInArchive);

    fs::remove_all(tempFolder);
}

void CrashDumpRVC4::fromTar(const fs::path& tarPath) {
    fs::path tempDir = platform::getTempPath();

    fs::path metadataPath = tempDir / METADATA_FILENAME;
    fs::path extraPath = tempDir / EXTRA_FILENAME;
    fs::path dataPath = tempDir / RVC4_DATA_FILENAME;

    std::vector<std::string> filesInArchive = {METADATA_FILENAME, EXTRA_FILENAME, RVC4_DATA_FILENAME};
    std::vector<fs::path> filesOnDisk = {metadataPath, extraPath, dataPath};
    utility::extractFiles(tarPath, filesInArchive, filesOnDisk);

    if(!fs::exists(metadataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing metadata file");
    }
    std::string metadataContent;
    readFromFile(metadataPath, metadataContent);
    nlohmann::json metadata = nlohmann::json::parse(metadataContent);
    readMetadata(metadata);
    filename = fromJson(metadata, "originalFilename", "");

    if(fs::exists(extraPath)) {
        std::string extraContent;
        readFromFile(extraPath, extraContent);
        extra = nlohmann::json::parse(extraContent);
        if(extra.is_null()) {  // json was null
            extra = nlohmann::json::object();
        }
    } else {
        extra = nlohmann::json::object();
    }

    if(!fs::exists(dataPath)) {
        throw std::runtime_error("Invalid crash dump tar: missing crash_dump.tar.gz");
    }
    readFromFile(dataPath, data);

    fs::remove_all(tempDir);
}

}  // namespace dai
