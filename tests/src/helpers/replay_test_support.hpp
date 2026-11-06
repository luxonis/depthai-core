#pragma once

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "record_test_support.hpp"
#include "replay_test_api.hpp"
#include "replay_test_data.hpp"

namespace replay_test {

using record_test::DeviceLogCapture;
using record_test::EnvGuard;
using record_test::LineLevel;
using record_test::LogCapture;
using record_test::RecordingGuard;
using record_test::TempDir;
using record_test::timestampMs;
using record_test::waitUntil;

inline void requireReplayData() {
    if(!replayDataAvailable()) SKIP("needs the generated data");
}

inline std::filesystem::path copyFile(const std::filesystem::path& from, const std::filesystem::path& to) {
    std::filesystem::create_directories(to.parent_path());
    std::filesystem::copy_file(from, to, std::filesystem::copy_options::overwrite_existing);
    return to;
}

struct PlaneView {
    int rows = 0;
    int rowBytes = 0;
    int stride = 0;
    int offset = 0;
};

inline std::vector<PlaneView> planesOf(dai::ImgFrame::Type type, int width, int height, int stride, int p2, int p3) {
    std::vector<PlaneView> planes;
    if(type == dai::ImgFrame::Type::NV12) {
        planes.push_back(PlaneView{height, width, stride, 0});
        const int uvOff = p2 > 0 ? p2 : stride * height;
        planes.push_back(PlaneView{height / 2, width, stride, uvOff});
    } else if(type == dai::ImgFrame::Type::YUV420p) {
        planes.push_back(PlaneView{height, width, stride, 0});
        const int uOff = p2 > 0 ? p2 : stride * height;
        const int uStride = stride / 2 > 0 ? stride / 2 : width / 2;
        planes.push_back(PlaneView{height / 2, width / 2, uStride, uOff});
        const int vOff = p3 > 0 ? p3 : uOff + uStride * (height / 2);
        planes.push_back(PlaneView{height / 2, width / 2, uStride, vOff});
    } else if(type == dai::ImgFrame::Type::RGB888i || type == dai::ImgFrame::Type::BGR888i) {
        planes.push_back(PlaneView{height, width * 3, stride > 0 ? stride : width * 3, 0});
    } else if(type == dai::ImgFrame::Type::RAW16) {
        planes.push_back(PlaneView{height, width * 2, stride > 0 ? stride : width * 2, 0});
    } else {
        planes.push_back(PlaneView{height, width, stride > 0 ? stride : width, 0});
    }
    return planes;
}

inline bool pixelsEqual(const dai::ImgFrame& frame, const std::vector<std::uint8_t>& packed, int width, int height, dai::ImgFrame::Type type) {
    const int stride = static_cast<int>(frame.getStride());
    const auto data = frame.getData();
    const auto planes = planesOf(type, width, height, stride > 0 ? stride : width, static_cast<int>(frame.fb.p2Offset), static_cast<int>(frame.fb.p3Offset));
    int packedAt = 0;
    for(const auto& plane : planes) {
        for(int row = 0; row < plane.rows; ++row) {
            const int src = plane.offset + row * plane.stride;
            if(src < 0 || packedAt + plane.rowBytes > static_cast<int>(packed.size())) return false;
            if(src + plane.rowBytes > static_cast<int>(data.size())) return false;
            if(!std::equal(packed.begin() + packedAt, packed.begin() + packedAt + plane.rowBytes, data.begin() + src)) return false;
            packedAt += plane.rowBytes;
        }
    }
    return true;
}

inline bool pixelsWithin(const dai::ImgFrame& left, const dai::ImgFrame& right, int tolerance) {
    if(left.getType() != right.getType() || left.getWidth() != right.getWidth() || left.getHeight() != right.getHeight()) return false;
    const auto type = left.getType();
    const int width = static_cast<int>(left.getWidth());
    const int height = static_cast<int>(left.getHeight());
    const auto planesL = planesOf(type, width, height, static_cast<int>(left.getStride()), static_cast<int>(left.fb.p2Offset), static_cast<int>(left.fb.p3Offset));
    const auto planesR = planesOf(type, width, height, static_cast<int>(right.getStride()), static_cast<int>(right.fb.p2Offset), static_cast<int>(right.fb.p3Offset));
    const auto dataL = left.getData();
    const auto dataR = right.getData();
    if(planesL.size() != planesR.size()) return false;
    for(std::size_t p = 0; p < planesL.size(); ++p) {
        for(int row = 0; row < planesL[p].rows; ++row) {
            const int a = planesL[p].offset + row * planesL[p].stride;
            const int b = planesR[p].offset + row * planesR[p].stride;
            for(int col = 0; col < planesL[p].rowBytes; ++col) {
                if(a + col >= static_cast<int>(dataL.size()) || b + col >= static_cast<int>(dataR.size())) return false;
                if(std::abs(static_cast<int>(dataL[static_cast<std::size_t>(a + col)]) - static_cast<int>(dataR[static_cast<std::size_t>(b + col)])) > tolerance) return false;
            }
        }
    }
    return true;
}

inline double meanY(const dai::ImgFrame& frame) {
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const int stride = std::max(static_cast<int>(frame.getStride()), width);
    const auto data = frame.getData();
    double sum = 0;
    int count = 0;
    for(int row = 0; row < height; ++row) {
        for(int col = 0; col < width; ++col) {
            const auto at = static_cast<std::size_t>(row * stride + col);
            if(at >= data.size()) break;
            sum += data[at];
            ++count;
        }
    }
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

inline bool near(double value, double expected, double tol) {
    return std::fabs(value - expected) <= tol;
}

inline void checkArrivals(const std::vector<std::chrono::steady_clock::time_point>& arrivals, const std::vector<double>& nominalMs) {
    REQUIRE(arrivals.size() == nominalMs.size());
    REQUIRE(arrivals.size() >= 2);
    std::vector<double> gaps;
    std::vector<double> nominalGaps;
    for(std::size_t i = 1; i < arrivals.size(); ++i) {
        const double gap = std::chrono::duration<double, std::milli>(arrivals[i] - arrivals[i - 1]).count();
        const double nominal = nominalMs[i] - nominalMs[i - 1];
        gaps.push_back(gap);
        nominalGaps.push_back(nominal);
        CHECK(gap + 0.01 >= nominal - 10.0);
    }
    const double mean = std::accumulate(gaps.begin(), gaps.end(), 0.0) / static_cast<double>(gaps.size());
    const double nominalMean = std::accumulate(nominalGaps.begin(), nominalGaps.end(), 0.0) / static_cast<double>(nominalGaps.size());
    if(nominalMean > 0.0) CHECK(std::fabs(mean - nominalMean) <= nominalMean * 0.15);
    const double span = std::chrono::duration<double, std::milli>(arrivals.back() - arrivals.front()).count();
    const double nominalSpan = nominalMs.back() - nominalMs.front();
    if(nominalSpan > 0.0) CHECK(std::fabs(span - nominalSpan) <= nominalSpan * 0.25);
}

inline std::set<std::string> hashesOf(const dai::ReplayCacheInfo& info, CacheKind kind) {
    std::set<std::string> hashes;
    for(const auto& entry : info.entries) {
        if(entry.kind == kind) hashes.insert(entry.sourceHash);
    }
    return hashes;
}

inline int countKind(const dai::ReplayCacheInfo& info, CacheKind kind) {
    int count = 0;
    for(const auto& entry : info.entries) {
        if(entry.kind == kind) ++count;
    }
    return count;
}

class CacheGuard {
   public:
    explicit CacheGuard(dai::Device& device) : device(&device), saved(device.getReplayCacheInfo().limitBytes) {
        device.clearReplayCache();
    }
    void watch(dai::Pipeline& pipeline) {
        watched = &pipeline;
    }
    ~CacheGuard() {
        try {
            if(watched != nullptr && watched->isRunning()) watched->stop();
            if(device != nullptr) {
                device->clearReplayCache();
                device->setReplayCacheLimit(saved);
            }
        } catch(const std::exception&) {
        }
    }
    CacheGuard(const CacheGuard&) = delete;
    CacheGuard& operator=(const CacheGuard&) = delete;

   private:
    dai::Device* device = nullptr;
    dai::Pipeline* watched = nullptr;
    std::uint64_t saved = 0;
};

class HostCacheGuard {
   public:
    explicit HostCacheGuard(dai::Pipeline* pipeline = nullptr) : pipeline(pipeline), saved(getHostCacheInfo().limitBytes) {
        clearHostCache();
    }
    ~HostCacheGuard() {
        try {
            if(pipeline != nullptr && pipeline->isRunning()) pipeline->stop();
            clearHostCache();
            setHostCacheLimit(saved);
        } catch(const std::exception&) {
        }
    }
    HostCacheGuard(const HostCacheGuard&) = delete;
    HostCacheGuard& operator=(const HostCacheGuard&) = delete;

   private:
    dai::Pipeline* pipeline = nullptr;
    std::uint64_t saved = 0;
};

struct HostReplay {
    TempDir dir;
    LogCapture log;
    dai::Pipeline pipeline{false};
    std::shared_ptr<dai::node::ReplayVideo> node;
    std::shared_ptr<dai::MessageQueue> out;
    std::filesystem::path source;

    HostReplay() {
        node = pipeline.create<dai::node::ReplayVideo>();
        record_test::watchNode(*node);
        out = node->out.createOutputQueue(16, true);
    }
    void queueOf(int size, bool blocking) {
        out = node->out.createOutputQueue(static_cast<unsigned>(size), blocking);
    }
    std::filesystem::path frames(int width, int height, int count, FrameFill fill = {}) {
        source = dir.path / "src";
        makeDaiDir(source, width, height, rangeSeq(count), rangeTs(count), fill);
        setSource(*node, source);
        setFast(*node);
        return source;
    }
};

inline std::vector<std::shared_ptr<dai::ImgFrame>> takeFrames(const std::shared_ptr<dai::MessageQueue>& queue, int count) {
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    frames.reserve(static_cast<std::size_t>(count));
    for(int i = 0; i < count; ++i) {
        auto frame = record_test::getFor<dai::ImgFrame>(*queue, std::chrono::seconds(10));
        REQUIRE(frame != nullptr);
        frames.push_back(frame);
    }
    return frames;
}

inline std::vector<std::chrono::steady_clock::time_point> takeTimed(const std::shared_ptr<dai::MessageQueue>& queue, int count) {
    std::vector<std::chrono::steady_clock::time_point> times;
    for(int i = 0; i < count; ++i) {
        auto frame = record_test::getFor<dai::ImgFrame>(*queue, std::chrono::seconds(30));
        REQUIRE(frame != nullptr);
        times.push_back(std::chrono::steady_clock::now());
        (void)frame;
    }
    return times;
}

struct DeviceReplay {
    std::shared_ptr<dai::Device> device;
    EnvGuard unsetReplay;
    EnvGuard unsetRecord;
    EnvGuard unsetDecode;
    TempDir dir;
    DeviceLogCapture devLog;
    LogCapture hostLog;
    std::unique_ptr<dai::Pipeline> pipeline;
    CacheGuard cache;
    std::unique_ptr<RecordingGuard> recordings;

    DeviceReplay()
        : device(record_test::openRvc4OrSkip()),
          unsetReplay("DEPTHAI_REPLAY"),
          unsetRecord("DEPTHAI_RECORD"),
          unsetDecode("DEPTHAI_REPLAY_DECODE_MODE"),
          devLog(*device, dai::LogLevel::DEBUG),
          cache(*device) {}

    dai::Pipeline& begin() {
        pipeline = std::make_unique<dai::Pipeline>(device);
        cache.watch(*pipeline);
        recordings = std::make_unique<RecordingGuard>(*device, *pipeline);
        return *pipeline;
    }
};

inline std::string makeDeviceSource(const std::shared_ptr<dai::Device>& device, const std::filesystem::path& hostDaiDir) {
    int count = 0;
    for(const auto& entry : std::filesystem::directory_iterator(hostDaiDir)) {
        if(entry.is_regular_file() && entry.path().extension() == ".dai") ++count;
    }
    const auto name = record_test::uniqueName("src");
    dai::Pipeline pipeline(device);
    record_test::DeviceLogCapture log(*device, dai::LogLevel::DEBUG);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    setSource(*replay, hostDaiDir);
    setFast(*replay);
    auto rec = pipeline.create<dai::node::RecordVideo>();
    record_test::setFormat(*rec, record_test::Format::RAW_FRAMES);
    record_test::setRecordingName(*rec, name);
    record_test::setStreamName(*rec, "cam");
    record_test::setTransferToHost(*rec, false);
    record_test::setKeepOnDevice(*rec, true);
    record_test::setOutputDirectory(*rec, std::filesystem::path(dai::platform::getTempPath()));
    replay->out.link(rec->input);
    auto status = rec->status.createOutputQueue(16, true);
    pipeline.start();
    REQUIRE(record_test::getFor<dai::RecordStatus>(*status, std::chrono::seconds(20)) != nullptr);
    const auto last = "Record wrote frame " + std::to_string(count - 1);
    REQUIRE(log.waitFor(last, std::chrono::seconds(60)));
    const auto done = record_test::getStatus(*rec);
    REQUIRE(done != nullptr);
    const auto path = done->path;
    pipeline.stop();
    return path + "/cam";
}

}  // namespace replay_test
