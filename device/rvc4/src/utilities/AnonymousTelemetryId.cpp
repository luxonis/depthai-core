#include "AnonymousTelemetryId.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace dai {

namespace {

constexpr char TELEMETRY_ID_PATH[] = "/persist/anonymousTelemetryId";
constexpr std::size_t TELEMETRY_ID_LENGTH = 32;
constexpr auto TELEMETRY_ID_TTL = std::chrono::hours(24);

bool isValidTelemetryId(const std::string& value) {
    return value.size() == TELEMETRY_ID_LENGTH && std::all_of(value.begin(), value.end(), [](unsigned char ch) { return std::isalnum(ch) != 0; });
}

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t expiresAtMs() {
    return nowMs() + std::chrono::duration_cast<std::chrono::milliseconds>(TELEMETRY_ID_TTL).count();
}

std::string generateTelemetryId() {
    static std::mt19937 generator(std::random_device{}());
    static const std::string alphabet = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    static std::uniform_int_distribution<std::size_t> distribution(0, alphabet.size() - 1);

    std::string value;
    value.reserve(TELEMETRY_ID_LENGTH);
    for(std::size_t i = 0; i < TELEMETRY_ID_LENGTH; ++i) {
        value.push_back(alphabet[distribution(generator)]);
    }
    return value;
}

std::string readTelemetryId(const fs::path& path) {
    std::ifstream input(path);
    if(!input) {
        return {};
    }

    try {
        nlohmann::json document;
        input >> document;
        const auto id = document.value("value", std::string{});
        const auto expiresAt = document.value("expires_at_ms", int64_t{0});
        if(isValidTelemetryId(id) && expiresAt > nowMs()) {
            return id;
        }
    } catch(const std::exception&) {
        return {};
    }

    return {};
}

void persistTelemetryId(const fs::path& path, const std::string& id) {
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if(!output) {
        throw std::runtime_error("Failed to open anonymous telemetry id file for writing");
    }
    output << nlohmann::json{{"value", id}, {"expires_at_ms", expiresAtMs()}}.dump(2);
    if(!output.good()) {
        throw std::runtime_error("Failed to persist anonymous telemetry id");
    }
}

}  // namespace

std::string getAnonymousTelemetryId() {
    static const std::string telemetryId = []() {
        const fs::path path(TELEMETRY_ID_PATH);
        if(const auto existing = readTelemetryId(path); !existing.empty()) {
            return existing;
        }

        const auto generated = generateTelemetryId();
        persistTelemetryId(path, generated);
        return generated;
    }();

    return telemetryId;
}

}  // namespace dai
