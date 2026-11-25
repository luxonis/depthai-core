#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/common/BsplineCalibration.hpp"

// RVC4 calibration extension. The legacy area remains readable by older firmware.
// An exact copy of the legacy area binds the extension to its matching EEPROM.
// Interrupted writes / older firmware rewrites cannot silently load stale data.
namespace dai::device {
class BsplineCalibrationStore {
    // CRC32 detects accidental coefficient corruption even if CBOR still parses.
    static uint32_t crc32(const std::vector<uint8_t>& bytes) {
        uint32_t crc = 0xffffffffu;
        for(uint8_t byte : bytes) {
            crc ^= byte;
            for(int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
        return ~crc;
    }
    std::filesystem::path directory;
    std::filesystem::path path(bool factory) const {
        return directory / (factory ? "bspline-factory.cbor" : "bspline-user.cbor");
    }

   public:
    explicit BsplineCalibrationStore(std::filesystem::path root = "/data/calibration") : directory(std::move(root)) {}

    std::optional<BsplineStereoCalibration> load(bool factory, const std::vector<uint8_t>& legacy, const std::string& deviceId) const {
        const auto file = path(factory);
        if(!std::filesystem::exists(file)) return std::nullopt;
        if(std::filesystem::file_size(file) > 1024 * 1024) throw std::runtime_error("Spline calibration extension exceeds size limit");
        std::ifstream input(file, std::ios::binary);
        if(!input) throw std::runtime_error("Cannot read spline calibration extension");
        std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        auto envelope = nlohmann::json::from_cbor(bytes);
        if(envelope.at("version") != 1 || envelope.at("legacy").get<std::vector<uint8_t>>() != legacy) {
            throw std::runtime_error("Spline calibration extension does not match current legacy calibration");
        }
        const auto payload = envelope.at("payload").get<std::vector<uint8_t>>();
        if(envelope.at("crc32").get<uint32_t>() != crc32(payload)) throw std::runtime_error("Spline calibration checksum mismatch");
        auto model = nlohmann::json::from_cbor(payload).get<BsplineStereoCalibration>();
        model.validate();
        if(model.deviceId != deviceId) throw std::runtime_error("Spline calibration belongs to another device");
        return model;
    }

    void store(bool factory, const std::vector<uint8_t>& legacy, const std::optional<BsplineStereoCalibration>& model, const std::string& deviceId) const {
        const auto file = path(factory);
        if(!model) {
            if(::unlink(file.c_str()) != 0 && errno != ENOENT) throw std::runtime_error("Cannot remove spline calibration extension");
            if(std::filesystem::exists(directory)) {
                const int fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY);
                if(fd < 0) throw std::runtime_error("Cannot open calibration directory");
                const int result = ::fsync(fd);
                ::close(fd);
                if(result != 0) throw std::runtime_error("Cannot sync calibration directory");
            }
            return;
        }
        model->validate();
        if(model->deviceId != deviceId) throw std::runtime_error("Spline calibration belongs to another device");
        const auto payload = nlohmann::json::to_cbor(nlohmann::json(*model));
        auto bytes = nlohmann::json::to_cbor(nlohmann::json{{"version", 1}, {"legacy", legacy}, {"payload", payload}, {"crc32", crc32(payload)}});
        if(bytes.size() > 1024 * 1024) throw std::runtime_error("Spline calibration extension exceeds size limit");
        std::filesystem::create_directories(directory);
        std::string temporary = file.string() + ".XXXXXX";
        std::vector<char> name(temporary.begin(), temporary.end());
        name.push_back(0);
        const int fd = ::mkstemp(name.data());
        if(fd < 0) throw std::runtime_error("Cannot create spline calibration extension");
        bool closed = false;
        try {
            size_t offset = 0;
            while(offset < bytes.size()) {
                const auto written = ::write(fd, bytes.data() + offset, bytes.size() - offset);
                if(written < 0 && errno == EINTR) continue;
                if(written <= 0) throw std::runtime_error("Cannot write spline calibration extension");
                offset += written;
            }
            if(::fsync(fd) != 0) throw std::runtime_error("Cannot sync spline calibration extension");
            const int closeResult = ::close(fd);
            closed = true;
            if(closeResult != 0) throw std::runtime_error("Cannot close spline calibration extension");
            if(::rename(name.data(), file.c_str()) != 0) throw std::runtime_error("Cannot replace spline calibration extension");
            const int dirFd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY);
            if(dirFd < 0) throw std::runtime_error("Cannot open calibration directory");
            const int result = ::fsync(dirFd);
            ::close(dirFd);
            if(result != 0) throw std::runtime_error("Cannot sync calibration directory");
        } catch(...) {
            if(!closed) ::close(fd);
            ::unlink(name.data());
            throw;
        }
    }
};
}  // namespace dai::device
