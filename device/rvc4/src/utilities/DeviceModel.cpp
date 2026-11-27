#include "DeviceModel.hpp"

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <regex>
#include <cstdint>
#include <regex>

#include "ErrorMacros.hpp"

namespace dai {
namespace gate {

static std::mutex cachedVersionLock;
static std::mutex cachedRevisionLock;
static std::optional<DeviceModel::Model> cachedVersion;
static std::optional<std::optional<std::uint32_t>> cachedRevision;

using Mod = DeviceModel::Model;
static std::vector<std::pair<std::string, DeviceModel::Model>> nameToEnum{
    {"OAK4-D", Mod::OAK4_D},
    {"OAK4-MAX", Mod::OAK4_MAX},
    {"OAK4-S", Mod::OAK4_S},
    {"DEV-KIT", Mod::DEV_KIT},
    {"OAK4-LITE-POE", Mod::OAK4_LITE},
    {"OAK4-LITE-USB", Mod::OAK4_LITE},
    {"OAK4-LITE-STD", Mod::OAK4_LITE},
    {"OAK4-LITE-PRO", Mod::OAK4_LITE},
    {"OAK4-LITE-TOF", Mod::OAK4_LITE},
    {"OAK4-FFC", Mod::OAK4_FFC},
    {"FastStationCamera", Mod::FAST_STATION_CAMERA},
    {"VISUAL-CORTEX", Mod::VISUAL_CORTEX}
};

DeviceModel::Model DeviceModel::get() {
    std::lock_guard lock(cachedVersionLock);
    if(cachedVersion == std::nullopt) {
        const std::filesystem::path deviceModelFile("/proc/device-tree/model");
        std::ifstream osVersionFileStream(deviceModelFile);
        std::string line;
        std::getline(osVersionFileStream, line);
        cachedVersion = Mod::UNKNOWN_MODEL;
        for(const auto& model : nameToEnum) {
            if(line.find(model.first) != std::string::npos) {
                cachedVersion = model.second;
                break;
            }
        }
        if(cachedVersion == Mod::UNKNOWN_MODEL) {
            spdlog::warn(fmt::format("Unknown device: {}", line.c_str()));
        }
    }
    DAI_CHECK_IN(cachedVersion);
    return *cachedVersion;
}

std::optional<std::uint32_t> DeviceModel::getRevision() {
    std::lock_guard lock(cachedRevisionLock);
    if(cachedRevision != std::nullopt) {
        return *cachedRevision;
    }

    cachedRevision.emplace(std::nullopt);

    Model model = get();
    if (model == Model::UNKNOWN_MODEL) {
        return *cachedRevision;
    }

    const std::filesystem::path deviceModelFile("/proc/device-tree/model");
    std::ifstream osVersionFileStream(deviceModelFile);
    std::string line;
    std::getline(osVersionFileStream, line);
    
    std::smatch revisionMatch;
    bool foundRevision = false;
    const auto tryMatch = [&](const char* modelName) {
        const std::string revisionRegexStr = fmt::format("{}[[:space:]]R([0-9]+)", modelName);
        const std::regex revisionRegex(revisionRegexStr, std::regex::extended);
        return std::regex_search(line, revisionMatch, revisionRegex);
    };

    for(const auto& name : nameToEnum) {
        if(name.second == model && tryMatch(name.first.c_str())) {
            foundRevision = true;
            break;
        }
    }

    if(!foundRevision) {
        DAI_ERROR("Unexpected model string: \"{}\"", line);
        return *cachedRevision;
    }

    if (revisionMatch.size() != 2) {
        DAI_ERROR("More than one revision found in model string: \"{}\"", line);
        for (const auto& match : revisionMatch) {
            DAI_ERROR("Match: \"{}\"", match.str());
        }
        return *cachedRevision;
    }

    std::string revisionStr = revisionMatch[1].str();

    std::uint32_t revision = 0;
    try {
        revision = std::stoul(revisionStr);
    }
    catch (const std::invalid_argument& e) {
        DAI_ERROR("Failed to parse revision string: \"{}\" from model string: \"{}\"", revisionStr, line);
        return *cachedRevision;
    }
    catch (const std::out_of_range& e) {
        DAI_ERROR("Failed to parse revision string: \"{}\" from model string: \"{}\"", revisionStr, line);
        return *cachedRevision;
    }

    cachedRevision->emplace(revision);
    return revision;
}

}  // namespace gate
}  // namespace dai
