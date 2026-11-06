#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/utility/RecordReplay.hpp"
#include "test_sha256.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace record_test {

inline std::filesystem::path recordDataRoot() {
#ifdef RECORD_TEST_DATA
    return std::filesystem::path(RECORD_TEST_DATA);
#else
    return {};
#endif
}

inline bool recordDataAvailable() {
#ifdef RECORD_TEST_DATA_AVAILABLE
    return std::filesystem::is_regular_file(recordDataRoot() / "plain_h264_small.h264");
#else
    (void)recordDataRoot;
    return false;
#endif
}

inline std::chrono::steady_clock::time_point steadyAtMs(std::int64_t ms) {
    return std::chrono::steady_clock::time_point(std::chrono::milliseconds(ms));
}

inline std::array<std::array<float, 3>, 3> intrinsicsT(int width, int height) {
    const float fx = static_cast<float>(width) / 2.f;
    const float fy = static_cast<float>(width) / 2.f;
    const float cx = static_cast<float>(width) / 2.f;
    const float cy = static_cast<float>(height) / 2.f;
    return {std::array<float, 3>{fx, 0.f, cx}, std::array<float, 3>{0.f, fy, cy}, std::array<float, 3>{0.f, 0.f, 1.f}};
}

inline std::vector<std::uint8_t> nv12Solid(int width, int height, std::uint8_t y, std::uint8_t u, std::uint8_t v) {
    const std::size_t ySize = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<std::uint8_t> data(ySize + ySize / 2, 0);
    std::fill(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(ySize), y);
    for(std::size_t i = ySize; i < data.size(); i += 2) {
        data[i] = u;
        data[i + 1] = v;
    }
    return data;
}

inline std::shared_ptr<dai::ImgFrame> makeImg(int width, int height, int n, std::int64_t sequence) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(dai::ImgFrame::Type::NV12);
    frame->setWidth(static_cast<unsigned>(width));
    frame->setHeight(static_cast<unsigned>(height));
    const auto y = static_cast<std::uint8_t>(n % 256);
    frame->setData(nv12Solid(width, height, y, 128, 128));
    frame->setSequenceNum(sequence);
    const auto when = steadyAtMs(static_cast<std::int64_t>(n) * 100);
    frame->setTimestamp(when);
    frame->setTimestampDevice(when);
    frame->sourceFb = frame->fb;
    frame->cam.exposureTimeUs = 1000 + n;
    frame->cam.sensitivityIso = 100;
    frame->setCategory(7);
    frame->setInstanceNum(1);
    frame->event = dai::FrameEvent::READOUT_START;
    frame->transformation = dai::ImgTransformation(static_cast<std::size_t>(width), static_cast<std::size_t>(height), intrinsicsT(width, height));
    return frame;
}

inline std::shared_ptr<dai::ImgFrame> makeImg(int width, int height, int n) {
    return makeImg(width, height, n, n);
}

inline std::vector<std::vector<std::uint8_t>> accessUnits(const std::filesystem::path& path) {
    const auto bytes = test_hash::readBytes(path);
    const std::uint8_t marker[5] = {0x00, 0x00, 0x00, 0x01, 0x67};
    std::vector<std::size_t> starts;
    if(bytes.size() >= 5) {
        for(std::size_t i = 0; i + 5 <= bytes.size(); ++i) {
            if(std::memcmp(bytes.data() + i, marker, 5) == 0) starts.push_back(i);
        }
    }
    std::vector<std::vector<std::uint8_t>> units;
    units.reserve(starts.size());
    for(std::size_t i = 0; i < starts.size(); ++i) {
        const std::size_t end = (i + 1 < starts.size()) ? starts[i + 1] : bytes.size();
        units.emplace_back(bytes.begin() + static_cast<std::ptrdiff_t>(starts[i]), bytes.begin() + static_cast<std::ptrdiff_t>(end));
    }
    return units;
}

inline std::vector<std::uint8_t> accessUnitN(int n) {
    if(!recordDataAvailable()) {
        throw std::runtime_error("generated record data is missing");
    }
    const auto units = accessUnits(recordDataRoot() / "plain_h264_small.h264");
    if(n < 0 || static_cast<std::size_t>(n) >= units.size()) {
        throw std::runtime_error("access unit index out of range");
    }
    return units[static_cast<std::size_t>(n)];
}

inline std::shared_ptr<dai::EncodedFrame> makeEncoded(int n, dai::EncodedFrame::Profile profile, std::vector<std::uint8_t> bitstream) {
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setProfile(profile);
    frame->setFrameType(dai::EncodedFrame::FrameType::I);
    frame->setWidth(64);
    frame->setHeight(48);
    frame->setSequenceNum(n);
    const auto when = steadyAtMs(static_cast<std::int64_t>(n) * 100);
    frame->setTimestamp(when);
    frame->setTimestampDevice(when);
    frame->setQuality(80);
    frame->setLossless(false);
    frame->setBitrate(0);
    frame->setInstanceNum(1);
    frame->transformation = dai::ImgTransformation(static_cast<std::size_t>(64), static_cast<std::size_t>(48), intrinsicsT(64, 48));
    frame->setData(std::move(bitstream));
    return frame;
}

inline std::shared_ptr<dai::EncodedFrame> makeEnc(int n) {
    return makeEncoded(n, dai::EncodedFrame::Profile::AVC, accessUnitN(n));
}

inline std::shared_ptr<dai::EncodedFrame> makeEnc265(int n) {
    return makeEncoded(n, dai::EncodedFrame::Profile::HEVC, accessUnitN(n));
}

inline std::shared_ptr<dai::EncodedFrame> makeEncJ(int n) {
    if(!recordDataAvailable()) throw std::runtime_error("generated record data is missing");
    return makeEncoded(n, dai::EncodedFrame::Profile::JPEG, test_hash::readBytes(recordDataRoot() / "gray_64x48.jpg"));
}

inline std::shared_ptr<dai::EncodedFrame> makeEncPad(int n, std::size_t bytes) {
    auto data = accessUnitN(n);
    if(data.size() > bytes) throw std::runtime_error("access unit is larger than the padded size");
    data.resize(bytes, 0);
    return makeEncoded(n, dai::EncodedFrame::Profile::AVC, std::move(data));
}

inline void makeDaiDir(const std::filesystem::path& dir, int width, int height, int count) {
    std::filesystem::create_directories(dir);
    for(int i = 0; i < count; ++i) {
        auto frame = makeImg(width, height, i);
        frame->save(dir / ("cam_" + std::to_string(i) + ".dai"));
    }
}

struct McapEntry {
    std::int64_t sequenceNum = 0;
    std::int64_t tsMs = 0;
    int width = 0;
    int height = 0;
};

inline void makeMcap(const std::filesystem::path& path, const std::vector<McapEntry>& entries, dai::RecordConfig::CompressionLevel level = dai::RecordConfig::CompressionLevel::DEFAULT) {
    dai::utility::ByteRecorder recorder;
    recorder.init<dai::proto::img_frame::ImgFrame>(path.string(), level, "video");
    for(const auto& entry : entries) {
        dai::ImgFrame frame;
        frame.setWidth(static_cast<unsigned>(entry.width));
        frame.setHeight(static_cast<unsigned>(entry.height));
        frame.setSequenceNum(entry.sequenceNum);
        const auto when = steadyAtMs(entry.tsMs);
        frame.setTimestamp(when);
        frame.setTimestampDevice(when);
        frame.transformation = dai::ImgTransformation(static_cast<std::size_t>(entry.width), static_cast<std::size_t>(entry.height), intrinsicsT(entry.width, entry.height));
        recorder.write(frame.serializeProto(true));
    }
    recorder.close();
}

inline void writeJson(const std::filesystem::path& path, const std::string& json) {
    std::ofstream output(path);
    if(!output) throw std::runtime_error("cannot write " + path.string());
    output << json;
}

inline std::string recordConfigJson(const std::string& outputDir, const std::string& format, bool keepOnDevice, bool writeFormat = true, bool writeKeep = true) {
    std::ostringstream out;
    out << "{\"outputDir\": \"" << outputDir << "\", \"videoEncoding\": {\"enabled\": true, \"bitrate\": 0, \"profile\": \"H264_MAIN\", \"lossless\": false, \"quality\": 80}, \"compressionLevel\": 3, \"syncCameraOutputs\": true";
    if(writeFormat) out << ", \"format\": \"" << format << "\"";
    if(writeKeep) out << ", \"keepOnDevice\": " << (keepOnDevice ? "true" : "false");
    out << "}";
    return out.str();
}

}  // namespace record_test
