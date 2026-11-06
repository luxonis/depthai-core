#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/schemas/IMUData.pb.h"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/utility/Compression.hpp"
#include "record_test_data.hpp"
#include "test_sha256.hpp"

namespace replay_test {

enum class TarKind { FLAT, NESTED, GZ, XZ, TWO_DIRS, EMPTY, ESCAPE, ABSOLUTE, SYMLINK, TRUNCATED };

struct FrameFill {
    enum class Kind { GRADIENT, PLAIN_YUV, PLAIN_BYTE };
    Kind kind = Kind::GRADIENT;
    std::uint8_t y = 0;
    std::uint8_t u = 128;
    std::uint8_t v = 128;
    std::uint8_t byte = 0;
    static FrameFill gradient() {
        return {};
    }
    static FrameFill plainYuv(std::uint8_t y, std::uint8_t u, std::uint8_t v) {
        FrameFill fill;
        fill.kind = Kind::PLAIN_YUV;
        fill.y = y;
        fill.u = u;
        fill.v = v;
        return fill;
    }
    static FrameFill plainByte(std::uint8_t value) {
        FrameFill fill;
        fill.kind = Kind::PLAIN_BYTE;
        fill.byte = value;
        return fill;
    }
};

inline std::filesystem::path replayDataRoot() {
#ifdef REPLAY_TEST_DATA
    return std::filesystem::path(REPLAY_TEST_DATA);
#else
    return {};
#endif
}

inline bool replayDataAvailable() {
#ifdef REPLAY_TEST_DATA_AVAILABLE
    return std::filesystem::is_regular_file(replayDataRoot() / "plain_h264.mp4");
#else
    return false;
#endif
}

inline std::string sha256(const std::vector<std::uint8_t>& bytes) {
    return test_hash::sha256Hex(bytes);
}
inline std::string hashFile(const std::filesystem::path& path) {
    return test_hash::sha256File(path);
}
inline std::string hashDir(const std::filesystem::path& dir) {
    return test_hash::sha256Dir(dir);
}
inline std::string hashFileAndMetadata(const std::filesystem::path& file, const std::filesystem::path& metadata) {
    return test_hash::sha256FileAndMetadata(file, metadata);
}

inline int bytesPerPixel(dai::ImgFrame::Type type) {
    using Type = dai::ImgFrame::Type;
    if(type == Type::NV12 || type == Type::YUV420p) return 0;
    if(type == Type::RAW16) return 2;
    if(type == Type::RGB888i || type == Type::BGR888i) return 3;
    return 1;
}

inline std::vector<std::uint8_t> frameBytes(int width, int height, int index, dai::ImgFrame::Type type, const FrameFill& fill) {
    std::size_t size = 0;
    if(type == dai::ImgFrame::Type::NV12 || type == dai::ImgFrame::Type::YUV420p) {
        size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3 / 2;
    } else {
        size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * static_cast<std::size_t>(bytesPerPixel(type));
    }
    std::vector<std::uint8_t> data(size, 0);
    if(type == dai::ImgFrame::Type::NV12 && fill.kind != FrameFill::Kind::PLAIN_BYTE) {
        const std::size_t ySize = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        for(int y = 0; y < height; ++y) {
            for(int x = 0; x < width; ++x) {
                const auto at = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
                if(fill.kind == FrameFill::Kind::PLAIN_YUV) {
                    data[at] = fill.y;
                } else {
                    data[at] = static_cast<std::uint8_t>((x + y + index) & 255);
                }
            }
        }
        for(int y = 0; y < height / 2; ++y) {
            for(int x = 0; x < width; x += 2) {
                const auto at = ySize + static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
                if(fill.kind == FrameFill::Kind::PLAIN_YUV) {
                    data[at] = fill.u;
                    data[at + 1] = fill.v;
                } else {
                    data[at] = static_cast<std::uint8_t>(x & 255);
                    data[at + 1] = static_cast<std::uint8_t>((y * 2) & 255);
                }
            }
        }
        return data;
    }
    if(fill.kind == FrameFill::Kind::PLAIN_BYTE || fill.kind == FrameFill::Kind::PLAIN_YUV) {
        const auto value = fill.kind == FrameFill::Kind::PLAIN_BYTE ? fill.byte : fill.y;
        std::fill(data.begin(), data.end(), value);
        return data;
    }
    for(std::size_t i = 0; i < data.size(); ++i) data[i] = static_cast<std::uint8_t>(i & 255);
    return data;
}

inline std::vector<std::uint8_t> makeDaiFrame(const std::filesystem::path& path,
                                              int width,
                                              int height,
                                              std::int64_t sequence,
                                              std::int64_t tsMs,
                                              dai::ImgFrame::Type type,
                                              FrameFill fill = {}) {
    if(path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(type);
    frame->setWidth(static_cast<unsigned>(width));
    frame->setHeight(static_cast<unsigned>(height));
    auto bytes = frameBytes(width, height, static_cast<int>(sequence), type, fill);
    frame->setData(bytes);
    frame->setSequenceNum(sequence);
    const auto when = record_test::steadyAtMs(tsMs);
    frame->setTimestamp(when);
    frame->setTimestampDevice(when);
    frame->transformation = dai::ImgTransformation(static_cast<std::size_t>(width), static_cast<std::size_t>(height), record_test::intrinsicsT(width, height));
    frame->save(path);
    return bytes;
}

inline std::vector<std::vector<std::uint8_t>> makeDaiDir(const std::filesystem::path& dir,
                                                         int width,
                                                         int height,
                                                         const std::vector<std::int64_t>& sequences,
                                                         const std::vector<std::int64_t>& tsMs,
                                                         FrameFill fill = {},
                                                         const std::vector<std::string>& fileNames = {}) {
    std::filesystem::create_directories(dir);
    std::vector<std::vector<std::uint8_t>> frames;
    for(std::size_t i = 0; i < sequences.size(); ++i) {
        const auto name = fileNames.empty() ? ("cam_" + std::to_string(sequences[i]) + ".dai") : fileNames[i];
        frames.push_back(makeDaiFrame(dir / name, width, height, sequences[i], tsMs[i], dai::ImgFrame::Type::NV12, fill));
    }
    return frames;
}

inline std::vector<std::int64_t> rangeSeq(int count) {
    std::vector<std::int64_t> values(static_cast<std::size_t>(count));
    for(int i = 0; i < count; ++i) values[static_cast<std::size_t>(i)] = i;
    return values;
}

inline std::vector<std::int64_t> rangeTs(int count, std::int64_t stepMs = 100) {
    std::vector<std::int64_t> values(static_cast<std::size_t>(count));
    for(int i = 0; i < count; ++i) values[static_cast<std::size_t>(i)] = static_cast<std::int64_t>(i) * stepMs;
    return values;
}

enum class OtherKind { IMU, H264, MJPEG };

inline void makeDaiOther(const std::filesystem::path& path, OtherKind kind, std::int64_t sequence = 0) {
    const auto when = record_test::steadyAtMs(sequence * 100);
    if(kind == OtherKind::IMU) {
        dai::IMUData imu;
        imu.setSequenceNum(sequence);
        imu.setTimestamp(when);
        imu.setTimestampDevice(when);
        dai::IMUPacket packet;
        packet.acceleroMeter.sequence = static_cast<std::int32_t>(sequence);
        packet.acceleroMeter.timestamp.sec = (sequence * 100) / 1000;
        packet.acceleroMeter.timestamp.nsec = ((sequence * 100) % 1000) * 1000000;
        packet.acceleroMeter.tsDevice = packet.acceleroMeter.timestamp;
        imu.packets.push_back(packet);
        imu.save(path);
        return;
    }
    if(!replayDataAvailable() && !record_test::recordDataAvailable()) {
        throw std::runtime_error("generated replay data is missing");
    }
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setWidth(64);
    frame->setHeight(48);
    frame->setSequenceNum(sequence);
    frame->setTimestamp(when);
    frame->setTimestampDevice(when);
    if(kind == OtherKind::H264) {
        const auto h264 = replayDataAvailable() ? replayDataRoot() / "plain_h264_small.h264" : record_test::recordDataRoot() / "plain_h264_small.h264";
        const auto units = record_test::accessUnits(h264);
        if(units.empty()) throw std::runtime_error("no access unit");
        frame->setProfile(dai::EncodedFrame::Profile::AVC);
        frame->setFrameType(dai::EncodedFrame::FrameType::I);
        frame->setData(units.front());
    } else {
        const auto jpg = replayDataAvailable() ? replayDataRoot() / "gray100.jpg" : record_test::recordDataRoot() / "gray_64x48.jpg";
        frame->setProfile(dai::EncodedFrame::Profile::JPEG);
        frame->setData(test_hash::readBytes(jpg));
    }
    frame->save(path);
}

inline void makeMcap(const std::filesystem::path& path, const std::vector<record_test::McapEntry>& entries) {
    record_test::makeMcap(path, entries, dai::RecordConfig::CompressionLevel::DEFAULT);
}

inline void makeFullMcap(const std::filesystem::path& path, const std::vector<std::shared_ptr<dai::ImgFrame>>& frames) {
    dai::utility::ByteRecorder recorder;
    recorder.init<dai::proto::img_frame::ImgFrame>(path.string(), dai::RecordConfig::CompressionLevel::DEFAULT, "video");
    for(const auto& frame : frames) recorder.write(frame->serializeProto(false));
    recorder.close();
}

inline void makeImuMcap(const std::filesystem::path& path, int count) {
    dai::utility::ByteRecorder recorder;
    recorder.init<dai::proto::imu_data::IMUData>(path.string(), dai::RecordConfig::CompressionLevel::DEFAULT, "imu");
    for(int i = 0; i < count; ++i) {
        dai::IMUData imu;
        const auto when = record_test::steadyAtMs(static_cast<std::int64_t>(i) * 100);
        imu.setSequenceNum(i);
        imu.setTimestamp(when);
        imu.setTimestampDevice(when);
        dai::IMUPacket packet;
        packet.acceleroMeter.sequence = i;
        packet.acceleroMeter.timestamp.sec = (static_cast<std::int64_t>(i) * 100) / 1000;
        packet.acceleroMeter.timestamp.nsec = ((static_cast<std::int64_t>(i) * 100) % 1000) * 1000000;
        packet.acceleroMeter.tsDevice = packet.acceleroMeter.timestamp;
        imu.packets.push_back(packet);
        recorder.write(imu.serializeProto());
    }
    recorder.close();
}

inline std::string shellQuote(const std::string& text) {
    std::string out = "'";
    for(char ch : text) {
        if(ch == '\'') out += "'\\''";
        else
            out += ch;
    }
    out += "'";
    return out;
}

inline void runTar(const std::string& command) {
    const int code = std::system(command.c_str());
    if(code != 0) throw std::runtime_error("tar failed: " + command);
}

inline std::filesystem::path makeTar(const std::filesystem::path& path, const std::filesystem::path& dir, TarKind kind) {
    std::filesystem::path archive = path;
    if(kind == TarKind::GZ || kind == TarKind::TRUNCATED) {
        if(archive.extension() != ".gz") archive += ".tar.gz";
    } else if(kind == TarKind::XZ) {
        if(archive.extension() != ".xz") archive += ".tar.xz";
    } else if(archive.extension() != ".tar") {
        archive += ".tar";
    }
    std::filesystem::create_directories(archive.parent_path());
    if(kind == TarKind::FLAT) {
        runTar("tar cf " + shellQuote(archive.string()) + " -C " + shellQuote(dir.string()) + " .");
    } else if(kind == TarKind::NESTED) {
        runTar("tar cf " + shellQuote(archive.string()) + " -C " + shellQuote(dir.parent_path().string()) + " " + shellQuote(dir.filename().string()));
    } else if(kind == TarKind::GZ) {
        runTar("tar czf " + shellQuote(archive.string()) + " -C " + shellQuote(dir.string()) + " .");
    } else if(kind == TarKind::XZ) {
        runTar("tar cJf " + shellQuote(archive.string()) + " -C " + shellQuote(dir.string()) + " .");
    } else if(kind == TarKind::TWO_DIRS) {
        const auto stage = dir.parent_path() / (dir.filename().string() + "_two");
        std::filesystem::create_directories(stage / "a");
        std::filesystem::create_directories(stage / "b");
        std::filesystem::copy_file(dir / "cam_0.dai", stage / "a" / "cam_0.dai", std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(dir / "cam_0.dai", stage / "b" / "cam_0.dai", std::filesystem::copy_options::overwrite_existing);
        runTar("tar cf " + shellQuote(archive.string()) + " -C " + shellQuote(stage.string()) + " .");
    } else if(kind == TarKind::EMPTY) {
        runTar("tar cf " + shellQuote(archive.string()) + " -T /dev/null");
    } else if(kind == TarKind::ESCAPE) {
        const auto outside = dir.parent_path() / "escape.dai";
        std::ofstream(outside, std::ios::binary) << "0123456789";
        runTar("tar cPf " + shellQuote(archive.string()) + " -C " + shellQuote(dir.string()) + " ../escape.dai");
    } else if(kind == TarKind::ABSOLUTE) {
        const auto absolute = std::filesystem::path("/tmp/replay_abs_x.dai");
        std::ofstream(absolute, std::ios::binary) << "0123456789";
        runTar("tar cPf " + shellQuote(archive.string()) + " " + shellQuote(absolute.string()));
    } else if(kind == TarKind::SYMLINK) {
        const auto stage = dir.parent_path() / (dir.filename().string() + "_link");
        std::filesystem::create_directories(stage);
        std::filesystem::create_symlink("/etc/hostname", stage / "cam_0.dai");
        runTar("tar cf " + shellQuote(archive.string()) + " -C " + shellQuote(stage.string()) + " .");
    } else if(kind == TarKind::TRUNCATED) {
        const auto full = archive.string() + ".full";
        runTar("tar czf " + shellQuote(full) + " -C " + shellQuote(dir.string()) + " .");
        const auto bytes = test_hash::readBytes(full);
        const std::size_t keep = std::min<std::size_t>(bytes.size() / 2, 3500);
        std::ofstream output(archive, std::ios::binary);
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(keep == 0 ? bytes.size() : keep));
        std::filesystem::remove(full);
    }
    return archive;
}

inline std::filesystem::path makeS1(const std::filesystem::path& dir) {
    const std::vector<std::int64_t> seq{0, 1, 2, 5, 7};
    const std::vector<std::int64_t> ts{0, 100, 200, 500, 700};
    makeDaiDir(dir, 64, 48, seq, ts);
    std::vector<record_test::McapEntry> entries;
    for(std::size_t i = 0; i < seq.size(); ++i) entries.push_back(record_test::McapEntry{seq[i], ts[i], 64, 48});
    record_test::makeMcap(dir / "cam.mcap", entries);
    std::ofstream(dir / "notes.txt") << "notes";
    std::ofstream(dir / "._cam_3.dai", std::ios::binary) << "dot";
    return dir;
}

inline void writeMeta(const std::filesystem::path& path, int count, std::int64_t seq0, std::int64_t ts0, int width, int height, std::int64_t step, std::optional<std::pair<int, std::int64_t>> replaceTs = std::nullopt) {
    std::vector<record_test::McapEntry> entries;
    for(int i = 0; i < count; ++i) {
        auto ts = ts0 + static_cast<std::int64_t>(i) * step;
        if(replaceTs && replaceTs->first == i) ts = replaceTs->second;
        entries.push_back(record_test::McapEntry{seq0 + i, ts, width, height});
    }
    record_test::makeMcap(path, entries);
}

inline void makeHolisticDir(const std::filesystem::path& dir, const std::string& recordConfigJson, const std::string& cameraInfoJson) {
    if(!replayDataAvailable()) throw std::runtime_error("generated replay data is missing");
    std::filesystem::create_directories(dir);
    std::filesystem::copy_file(replayDataRoot() / "plain60_h264.mp4", dir / "CameraCAM_A.mp4", std::filesystem::copy_options::overwrite_existing);
    writeMeta(dir / "CameraCAM_A.mcap", 60, 0, 1000, 1920, 1080, 33);
    std::vector<std::shared_ptr<dai::ImgFrame>> framesB;
    std::vector<std::int64_t> seq(60);
    std::vector<std::int64_t> ts(60);
    for(int i = 0; i < 60; ++i) {
        seq[static_cast<std::size_t>(i)] = i;
        ts[static_cast<std::size_t>(i)] = 1000 + static_cast<std::int64_t>(i) * 33;
    }
    for(int i = 0; i < 60; ++i) {
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setType(dai::ImgFrame::Type::NV12);
        frame->setWidth(1920);
        frame->setHeight(1080);
        frame->setData(frameBytes(1920, 1080, i, dai::ImgFrame::Type::NV12, FrameFill::plainYuv(80, 128, 128)));
        frame->setSequenceNum(i);
        const auto when = record_test::steadyAtMs(ts[static_cast<std::size_t>(i)]);
        frame->setTimestamp(when);
        frame->setTimestampDevice(when);
        frame->transformation = dai::ImgTransformation(1920, 1080, record_test::intrinsicsT(1920, 1080));
        framesB.push_back(frame);
    }
    makeFullMcap(dir / "CameraCAM_B.mcap", framesB);
    std::filesystem::create_directories(dir / "CameraCAM_C");
    for(int i = 0; i < 60; ++i) {
        const auto name = "CameraCAM_C_" + std::to_string(i) + ".dai";
        makeDaiFrame(dir / "CameraCAM_C" / name, 1920, 1080, i, ts[static_cast<std::size_t>(i)], dai::ImgFrame::Type::NV12, FrameFill::plainYuv(50, 128, 128));
    }
    makeImuMcap(dir / "IMU.mcap", 10);
    {
        std::ofstream output(dir / "record_config.json");
        output << recordConfigJson;
    }
    {
        std::ofstream output(dir / "camera_info.json");
        output << cameraInfoJson;
    }
}

}  // namespace replay_test
