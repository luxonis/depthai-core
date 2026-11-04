#include "depthai/utility/RecordingStore.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace dai {
namespace {

bool isNameChar(char character) {
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') || (character >= '0' && character <= '9') || character == '.'
           || character == '_' || character == '-';
}

bool endsWithUnderscoreDigits(const std::string& name) {
    const auto split = name.rfind('_');
    if(split == std::string::npos || split + 1 >= name.size()) return false;
    for(std::size_t index = split + 1; index < name.size(); ++index) {
        if(name[index] < '0' || name[index] > '9') return false;
    }
    return true;
}

bool nameOk(const std::string& name, bool stream) {
    if(name.empty() || name == "." || name == "..") return false;
    for(char character : name) {
        if(!isNameChar(character)) return false;
    }
    if(stream && endsWithUnderscoreDigits(name)) return false;
    return true;
}

}  // namespace

void validateRecordingName(const std::string& name) {
    if(!nameOk(name, false)) {
        throw std::invalid_argument("Record invalid recordingName: " + name);
    }
}

void validateStreamName(const std::string& name) {
    if(!nameOk(name, true)) {
        throw std::invalid_argument("Record invalid streamName: " + name);
    }
}

std::string makeRecordingTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &time);
#else
    localtime_r(&time, &local);
#endif
    std::ostringstream text;
    text << std::put_time(&local, "%Y-%m-%d_%H-%M-%S") << '-' << std::setw(3) << std::setfill('0') << milliseconds.count();
    return text.str();
}

std::string makeDefaultRecordingName() {
    return "recording_" + makeRecordingTimestamp();
}

std::filesystem::path pathWithSegment(const std::filesystem::path& file, int segment) {
    if(segment <= 0 || file.empty()) return file;
    const auto name = file.stem().string() + "_s" + std::to_string(segment) + file.extension().string();
    return file.parent_path() / name;
}

}  // namespace dai
