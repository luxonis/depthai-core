#include <fstream>

#include "depthai/utility/RecordReplay.hpp"
#include <spdlog/spdlog.h>
#include "utility/Platform.hpp"

namespace dai {
namespace utility {

VideoRecorderMp4v2::~VideoRecorderMp4v2() {}

bool VideoRecorderMp4v2::init(std::string filePath, int width, int height, int fps) {
    return false;
}

bool VideoRecorderMp4v2::write(span<const uint8_t>&) {
    return false;
}

bool VideoRecorderMp4v2::close() {
    return false;
}

bool checkRecordConfig(std::string& recordPath, utility::RecordConfig& config) {
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
            config = j.get<utility::RecordConfig>();

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
std::string matchTo(const std::vector<std::string>& mxIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames) {
    std::string mxId = "";
    for(const auto& id : mxIds) {
        std::vector<std::string> matches;
        for(const auto& filename : filenames) {
            if(filename.size() >= 4 && filename.substr(filename.size() - 4, filename.size()) != "meta" && filename.find(id) != std::string::npos) {
                matches.push_back(filename.substr(id.size() + 1, filename.find_last_of('.') - id.size() - 1));
            }
        }
        if(matches.size() == nodenames.size()) {
            if(allMatch(matches, nodenames)) {
                if(mxId.empty()) {
                    mxId = id;
                } else {
                    throw std::runtime_error("Multiple recordings match the pipeline configuration - unsupported.");
                }
            }
        }
    }
    return mxId;
}

}  // namespace utility
}  // namespace dai
