#include "depthai/utility/RecordReplay.hpp"

#include <spdlog/spdlog.h>

#include <optional>
#include <stdexcept>

#include "Environment.hpp"
#include "RecordReplayImpl.hpp"
#include "build/version.hpp"
#include "depthai/utility/RecordReplaySchema.hpp"
#include "utility/Compression.hpp"
#include "utility/Platform.hpp"

namespace dai {
namespace utility {

ByteRecorder::~ByteRecorder() {
    close();
}

void ByteRecorder::init(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel, RecordType recordingType) {
    if(initialized) {
        throw std::runtime_error("ByteRecorder already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("ByteRecorder file path is empty");
    }
    {
    }
    {
        const char* schemaText = DEFAULT_SHEMA;
        std::string channelName = "default";
        switch(recordingType) {
            case RecordType::Video:
                channelName = "video";
                schemaText = VIDEO_SHEMA;
                break;
            case RecordType::Imu:
                channelName = "imu";
                schemaText = IMU_SHEMA;
                break;
            case RecordType::Other:
                channelName = "default";
                schemaText = DEFAULT_SHEMA;
                break;
        }
    }

    initialized = true;
}

void ByteRecorder::close() {
    if(initialized) {
        initialized = false;
    }
}

BytePlayer::~BytePlayer() {
    close();
}

void BytePlayer::init(const std::string& filePath) {
    if(initialized) {
        throw std::runtime_error("BytePlayer already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("BytePlayer file path is empty");
    }
    {
    }
    initialized = true;
}

std::optional<nlohmann::json> BytePlayer::next() {
    if(!initialized) {
        throw std::runtime_error("BytePlayer not initialized");
    }

    return nlohmann::json();
}

void BytePlayer::restart() {
    if(!initialized) {
        throw std::runtime_error("BytePlayer not initialized");
    }
}

void BytePlayer::close() {
    if(initialized) {
        initialized = false;
    }
}

std::optional<std::tuple<uint32_t, uint32_t>> BytePlayer::getVideoSize(const std::string& filePath) {
    if(filePath.empty()) {
        throw std::runtime_error("File path is empty in BytePlayer::getVideoSize");
    }
    {
    }
    return std::nullopt;
}

bool checkRecordConfig(std::string& recordPath, RecordConfig& config) {
    if(!platform::checkPathExists(recordPath)) {
        spdlog::warn("DEPTHAI_RECORD path does not exist or is invalid. Record disabled.");
        return false;
    }
    if(platform::checkPathExists(recordPath, true)) {
        // Is a directory
        config.outputDir = recordPath;
    } else {
        // Is a file
        std::string ext = recordPath.substr(recordPath.find_last_of('.') + 1);
        if(ext != "json") {
            spdlog::warn("DEPTHAI_RECORD path is not a directory or a json file. Record disabled.");
            return false;
        }
        try {
            std::ifstream file(recordPath);
            json j = json::parse(file);
            config = j.get<RecordConfig>();

            if(platform::checkPathExists(config.outputDir, true)) {
                // Is a directory
                recordPath = config.outputDir;
            } else {
                spdlog::warn("DEPTHAI_RECORD outputDir is not a directory. Record disabled.");
                return false;
            }
        } catch(const std::exception& e) {
            spdlog::warn("Error while processing DEPTHAI_RECORD json file: {}. Record disabled.", e.what());
            return false;
        }
    }
    return true;
}

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2) {
    for(const auto& el : v1) {
        if(std::find(v2.begin(), v2.end(), el) == v2.end()) return false;
    }
    return true;
}
std::string matchTo(const std::vector<std::string>& deviceIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames) {
    std::string deviceId = "";
    for(const auto& id : deviceIds) {
        std::vector<std::string> matches;
        for(const auto& filename : filenames) {
            if(filename.size() >= 4 && filename.substr(filename.size() - 4, filename.size()) != "meta" && filename.find(id) != std::string::npos) {
                matches.push_back(filename.substr(id.size() + 1, filename.find_last_of('.') - id.size() - 1));
            }
        }
        if(matches.size() == nodenames.size()) {
            if(allMatch(matches, nodenames)) {
                if(deviceId.empty()) {
                    deviceId = id;
                } else {
                    throw std::runtime_error("Multiple recordings match the pipeline configuration - unsupported.");
                }
            }
        }
    }
    return deviceId;
}

}  // namespace utility
}  // namespace dai
