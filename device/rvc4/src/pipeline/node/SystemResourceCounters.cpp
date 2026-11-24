#include "SystemResourceCounters.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <vector>

namespace system_resource_counters {
namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) return {};
    std::ostringstream contents;
    contents << input.rdbuf();
    std::string value = contents.str();
    while(!value.empty() && (std::isspace(static_cast<unsigned char>(value.back())) || value.back() == '\0')) value.pop_back();
    const auto first = value.find_first_not_of(" \t\r\n\0", 0);
    return first == std::string::npos ? std::string{} : value.substr(first);
}

bool parseNumber(const std::string& text, double& value) {
    std::istringstream stream(text);
    if(!(stream >> value)) return false;
    stream >> std::ws;
    return !text.empty() && stream.eof() && std::isfinite(value);
}

double parsePercent(const std::string& text) {
    std::string value = text;
    if(!value.empty() && value.back() == '%') value.pop_back();
    double percent = -1.0;
    return parseNumber(value, percent) && percent >= 0.0 && percent <= 100.0 ? percent : -1.0;
}

double parseRatioPercent(const std::string& text) {
    std::istringstream stream(text);
    long long busy = -1, total = 0;
    std::string extra;
    if(!(stream >> busy >> total) || (stream >> extra) || busy < 0 || total <= 0 || busy > total) return -1.0;
    return 100.0 * static_cast<double>(busy) / static_cast<double>(total);
}

std::map<std::string, double> parseDsp(const std::string& text) {
    static const std::array<std::pair<std::string, std::regex>, 3> patterns{{
        {"hvx", std::regex("(?:^|\\n)\\s*HVX utilization in percentage\\s*=\\s*([0-9]+(?:\\.[0-9]+)?)\\s*(?:\\n|$)")},
        {"hmx", std::regex("(?:^|\\n)\\s*HMX utilization in percentage\\s*=\\s*([0-9]+(?:\\.[0-9]+)?)\\s*(?:\\n|$)")},
        {"q6", std::regex("(?:^|\\n)\\s*Average Q6 load in KCPS\\s*=\\s*([0-9]+(?:\\.[0-9]+)?)\\s*(?:\\n|$)")},
    }};
    std::map<std::string, double> values;
    for(const auto& pattern : patterns) {
        std::smatch match;
        if(std::regex_search(text, match, pattern.second)) {
            double number = -1.0;
            if(parseNumber(match[1].str(), number) && (pattern.first == "q6" || (number >= 0.0 && number <= 100.0))) values[pattern.first] = number;
        }
    }
    return values;
}

std::map<std::string, std::vector<std::filesystem::path>> discoverPowerRails(const std::filesystem::path& hwmonRoot) {
    std::map<std::string, std::vector<std::filesystem::path>> candidates;
    std::error_code error;
    if(!std::filesystem::is_directory(hwmonRoot, error) || error) return candidates;
    std::filesystem::directory_iterator it(hwmonRoot, error), end;
    if(error) return candidates;
    for(; it != end; it.increment(error)) {
        if(error) return {};
        const bool isDirectory = it->is_directory(error);
        if(error) return {};
        if(!isDirectory || it->path().filename().string().rfind("hwmon", 0) != 0) continue;
        if(readText(it->path() / "name") != "ina700") continue;
        const std::string label = readText(it->path() / "of_node" / "label");
        if(label == "vdd_5v0" || label == "vdd_3v9" || label == "poe" || label == "tof") candidates[label].push_back(it->path() / "power1_input");
    }
    if(error) return {};
    return candidates;
}

double readPower(const std::map<std::string, std::vector<std::filesystem::path>>& candidates, const std::string& rail) {
    const auto it = candidates.find(rail);
    if(it == candidates.end() || it->second.size() != 1) return -1.0;
    double microwatts = -1.0;
    if(!parseNumber(readText(it->second.front()), microwatts) || microwatts < 0.0) return -1.0;
    return microwatts / 1000000.0;
}

}  // namespace

Values Collector::read() const {
    Values values;
    const std::filesystem::path gpuPath = std::filesystem::path(sysfsRoot) / "class/kgsl/kgsl-3d0";
    values.gpuUtilizationPercent = parsePercent(readText(gpuPath / "gpu_busy_percentage"));
    if(values.gpuUtilizationPercent < 0.0) values.gpuUtilizationPercent = parseRatioPercent(readText(gpuPath / "gpubusy"));

    const std::string dsp = readText(std::filesystem::path(debugfsRoot) / "sysmon_subsystem_stats/master_cdsp_stats");
    const auto dspValues = parseDsp(dsp);
    if(auto it = dspValues.find("hvx"); it != dspValues.end()) values.dspHvxActivityPercent = it->second;
    if(auto it = dspValues.find("hmx"); it != dspValues.end()) values.dspHmxActivityPercent = it->second;
    if(auto it = dspValues.find("q6"); it != dspValues.end()) values.dspQ6LoadKcps = it->second;

    const auto powerRails = discoverPowerRails(std::filesystem::path(sysfsRoot) / "class/hwmon");
    values.powerSystemWatts = readPower(powerRails, "vdd_5v0");
    values.powerProcessorWatts = readPower(powerRails, "vdd_3v9");
    values.powerPoeWatts = readPower(powerRails, "poe");
    values.powerTofWatts = readPower(powerRails, "tof");
    return values;
}

}  // namespace system_resource_counters
