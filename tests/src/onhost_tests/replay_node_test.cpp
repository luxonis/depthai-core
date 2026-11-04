#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/Serialization.hpp"
#include "opencv/ReplayInternal.hpp"
#include "../ReplayImageArchiveFixture.hpp"

#ifdef _WIN32
    #include <process.h>
#else
    #include <unistd.h>
#endif

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;

namespace {

std::filesystem::path cacheDir() {
    static const auto dir = std::filesystem::temp_directory_path()
                            / ("depthai_replay_node_"
                               + std::to_string(
#ifdef _WIN32
                                   _getpid()
#else
                                   getpid()
#endif
                                       ));
    return dir;
}

void setEnv(const char* key, const std::string& value) {
#ifdef _WIN32
    _putenv_s(key, value.c_str());
#else
    setenv(key, value.c_str(), 1);
#endif
}

void unsetEnv(const char* key) {
#ifdef _WIN32
    _putenv_s(key, "");
#else
    unsetenv(key);
#endif
}

void resetCache() {
    const auto dir = cacheDir();
    std::filesystem::create_directories(dir);
    setEnv("DEPTHAI_REPLAY_CACHE", dir.string());
    unsetEnv("DEPTHAI_REPLAY_DECODE_MODE");
    unsetEnv("DEPTHAI_REPLAY_PERSISTENT_UPLOAD");
    dai::node::ReplayVideo::setReplayCacheLimit(32ull << 30);
    dai::node::ReplayVideo::clearReplayCache();
}

std::filesystem::path makeRoot(const std::string& name) {
    auto root = cacheDir() / "sources" / name;
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    return root;
}

dai::ImgTransformation pinhole(int width, int height, float fx, float fy, float cx, float cy) {
    dai::ImgTransformation transformation(static_cast<std::size_t>(width), static_cast<std::size_t>(height));
    transformation.setIntrinsicMatrix({{{fx, 0.f, cx}, {0.f, fy, cy}, {0.f, 0.f, 1.f}}});
    return transformation;
}

void writeNv12(const std::filesystem::path& path,
               std::int64_t sequence,
               int timestampMs,
               int width,
               int height,
               std::uint8_t yValue,
               const dai::ImgTransformation& transformation) {
    dai::ImgFrame frame;
    frame.setType(dai::ImgFrame::Type::NV12);
    frame.setSize(static_cast<unsigned>(width), static_cast<unsigned>(height));
    frame.setSequenceNum(sequence);
    const auto stamp = std::chrono::steady_clock::time_point(std::chrono::milliseconds(timestampMs));
    frame.setTimestamp(stamp);
    frame.setTimestampDevice(stamp);
    std::vector<std::uint8_t> data(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3 / 2, 128);
    std::fill(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(width * height), yValue);
    frame.setData(data);
    frame.setTransformation(transformation);
    frame.save(path);
}

void writeTyped(
    const std::filesystem::path& path, dai::ImgFrame::Type type, int width, int height, const std::vector<std::uint8_t>& data, std::int64_t sequence) {
    dai::ImgFrame frame;
    frame.setType(type);
    frame.setSize(static_cast<unsigned>(width), static_cast<unsigned>(height));
    frame.setSequenceNum(sequence);
    const auto stamp = std::chrono::steady_clock::time_point(std::chrono::milliseconds(sequence * 100));
    frame.setTimestamp(stamp);
    frame.setTimestampDevice(stamp);
    frame.setData(data);
    frame.save(path);
}

struct Spec {
    int fileSeq = 0;
    std::int64_t sequence = 0;
    int timestampMs = 0;
    std::uint8_t y = 100;
};

std::filesystem::path writeStream(const std::string& name, int width, int height, const std::vector<Spec>& frames, float fx = -1.f) {
    auto dir = makeRoot(name);
    const float focal = fx > 0.f ? fx : static_cast<float>(width) / 2.f;
    const auto transformation = pinhole(width, height, focal, focal, static_cast<float>(width) / 2.f, static_cast<float>(height) / 2.f);
    for(const auto& frame : frames) {
        writeNv12(dir / ("cam_" + std::to_string(frame.fileSeq) + ".dai"), frame.sequence, frame.timestampMs, width, height, frame.y, transformation);
    }
    return dir;
}

std::vector<std::shared_ptr<dai::ImgFrame>> collect(const std::filesystem::path& source, int count, float fps, bool loop, bool stopAtEnd) {
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setFps(fps);
    replay->setLoop(loop);
    replay->setStopPipelineAtEnd(stopAtEnd);
    auto queue = replay->out.createOutputQueue(8, true);
    pipeline.start();
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    frames.reserve(static_cast<std::size_t>(count));
    for(int index = 0; index < count; index++) frames.push_back(queue->get<dai::ImgFrame>());
    if(!loop && stopAtEnd)
        pipeline.wait();
    else
        pipeline.stop();
    return frames;
}

double meanY(const dai::ImgFrame& frame) {
    const auto data = frame.getData();
    const auto count = static_cast<std::size_t>(frame.getWidth()) * static_cast<std::size_t>(frame.getHeight());
    double sum = 0.0;
    for(std::size_t index = 0; index < count && index < data.size(); index++) sum += data[index];
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
}

}  // namespace

TEST_CASE("Replay hash and properties") {
    const std::uint8_t abc[] = {'a', 'b', 'c'};
    REQUIRE(dai::replay::sha256OfBytes(abc, 3) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    const auto empty = cacheDir() / "empty.bin";
    std::filesystem::create_directories(cacheDir());
    { std::ofstream output(empty, std::ios::binary); }
    REQUIRE(dai::replay::sha256OfFile(empty) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    dai::ReplayVideoProperties properties;
    properties.sourcePath = "clip.mp4";
    properties.sourcePathSet = true;
    properties.fps = 20.f;
    properties.fpsSet = true;
    properties.loop = false;
    properties.outFrameType = 2;
    dai::ReplayVideoProperties restored;
    dai::utility::deserialize(dai::utility::serialize(properties), restored);
    REQUIRE(restored.sourcePath == "clip.mp4");
    REQUIRE(restored.fps == 20.f);
    REQUIRE(restored.loop == false);
    REQUIRE(restored.outFrameType == 2);
}

TEST_CASE("ReplayVideo setRunOnHost false without RVC4") {
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setRunOnHost(false);
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("setRunOnHost(false)"));
}

TEST_CASE("ReplayVideo fps must be finite") {
    resetCache();
    const auto source = writeStream("nan", 64, 48, {{0, 0, 0, 100}});
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setFps(std::numeric_limits<float>::quiet_NaN());
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("fps is not a finite number"));
}

TEST_CASE("S1 dai stream with gaps") {
    resetCache();
    const auto source = writeStream("s1", 64, 48, {{0, 0, 0, 10}, {1, 1, 100, 20}, {2, 2, 200, 30}, {5, 5, 500, 40}, {7, 7, 700, 50}});
    {
        std::ofstream notes(source / "notes.txt");
        notes << "ignore";
        std::ofstream hidden(source / "._cam_3.dai", std::ios::binary);
        hidden << "hidden";
        std::ofstream metadata(source / "cam.mcap", std::ios::binary);
        metadata << "sidecar";
    }
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setFps(1000.f);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(false);
    REQUIRE(replay->getOutFrameType() == dai::ImgFrame::Type::YUV420p);
    auto queue = replay->out.createOutputQueue();
    pipeline.start();
    const std::vector<std::int64_t> sequences{0, 1, 2, 5, 7};
    const std::vector<int> times{0, 100, 200, 500, 700};
    for(int index = 0; index < 5; index++) {
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getSequenceNum() == sequences[static_cast<std::size_t>(index)]);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(frame->getWidth() == 64);
        REQUIRE(frame->getHeight() == 48);
        const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(frame->getTimestamp().time_since_epoch()).count();
        REQUIRE(millis == times[static_cast<std::size_t>(index)]);
        REQUIRE(frame->getTimestampDevice() == frame->getTimestamp());
    }
    std::this_thread::sleep_for(50ms);
    REQUIRE(queue->tryGet<dai::ImgFrame>() == nullptr);
    REQUIRE(pipeline.isRunning());
    pipeline.stop();
}

TEST_CASE("S2 loop metadata") {
    resetCache();
    const auto source = writeStream("s2", 64, 48, {{10, 10, 1000, 10}, {11, 11, 1100, 20}, {12, 12, 1300, 30}});
    auto frames = collect(source, 7, 1000.f, true, true);
    const std::vector<std::int64_t> sequences{10, 11, 12, 13, 14, 15, 16};
    const std::vector<int> times{1000, 1100, 1300, 1450, 1550, 1750, 1900};
    for(int index = 0; index < 7; index++) {
        REQUIRE(frames[static_cast<std::size_t>(index)]->getSequenceNum() == sequences[static_cast<std::size_t>(index)]);
        const auto millis =
            std::chrono::duration_cast<std::chrono::milliseconds>(frames[static_cast<std::size_t>(index)]->getTimestamp().time_since_epoch()).count();
        REQUIRE(millis == times[static_cast<std::size_t>(index)]);
        REQUIRE(frames[static_cast<std::size_t>(index)]->getTimestampDevice() == frames[static_cast<std::size_t>(index)]->getTimestamp());
    }

    auto paced = collect(source, 4, 20.f, true, true);
    REQUIRE(paced[3]->getSequenceNum() == 13);
    const auto pacedMs = std::chrono::duration_cast<std::chrono::milliseconds>(paced[3]->getTimestamp().time_since_epoch()).count();
    REQUIRE(pacedMs == 1350);
}

TEST_CASE("S2b S1 data looped at fps 1000") {
    resetCache();
    const auto source = writeStream("s2b", 64, 48, {{0, 0, 0, 10}, {1, 1, 100, 20}, {2, 2, 200, 30}, {5, 5, 500, 40}, {7, 7, 700, 50}});
    auto frames = collect(source, 10, 1000.f, true, true);
    const std::vector<std::int64_t> pass1{8, 9, 10, 13, 15};
    const std::vector<int> times{875, 975, 1075, 1375, 1575};
    for(int index = 0; index < 5; index++) {
        REQUIRE(frames[static_cast<std::size_t>(index + 5)]->getSequenceNum() == pass1[static_cast<std::size_t>(index)]);
        const auto millis =
            std::chrono::duration_cast<std::chrono::milliseconds>(frames[static_cast<std::size_t>(index + 5)]->getTimestamp().time_since_epoch()).count();
        REQUIRE(millis == times[static_cast<std::size_t>(index)]);
    }
}

TEST_CASE("S2c sequence numbers do not increase") {
    resetCache();
    const auto source = writeStream("s2c", 64, 48, {{0, 5, 0, 10}, {1, 4, 100, 20}});
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setFps(1000.f);
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("sequence numbers do not increase"));
}

TEST_CASE("S4 resize scales the transformation") {
    resetCache();
    const auto source = writeStream("s4", 640, 480, {{0, 0, 0, 80}, {1, 1, 100, 90}}, 320.f);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setSize(320, 240);
    replay->setFps(1000.f);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(false);
    auto queue = replay->out.createOutputQueue();
    pipeline.start();
    for(int index = 0; index < 2; index++) {
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getWidth() == 320);
        REQUIRE(frame->getHeight() == 240);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        const auto matrix = frame->transformation.getIntrinsicMatrix();
        REQUIRE(matrix[0][0] == Catch::Approx(160.f).margin(0.05));
        REQUIRE(matrix[1][1] == Catch::Approx(160.f).margin(0.05));
        REQUIRE(matrix[0][2] == Catch::Approx(160.f).margin(0.05));
        REQUIRE(matrix[1][2] == Catch::Approx(120.f).margin(0.05));
    }
    pipeline.stop();

    dai::Pipeline odd(false);
    auto oddReplay = odd.create<dai::node::ReplayVideo>();
    oddReplay->setReplayVideoFile(source);
    oddReplay->setSize(321, 240);
    REQUIRE_THROWS_WITH(odd.start(), ContainsSubstring("321x240"));
}

TEST_CASE("S6 output type") {
    resetCache();
    const auto source = writeStream("s6", 64, 48, {{0, 0, 0, 100}});
    auto unchanged = collect(source, 1, 1000.f, false, false);
    REQUIRE(unchanged[0]->getType() == dai::ImgFrame::Type::NV12);
    REQUIRE(meanY(*unchanged[0]) == Catch::Approx(100.0).margin(0.01));

    {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(source);
        replay->setOutFrameType(dai::ImgFrame::Type::GRAY8);
        replay->setFps(1000.f);
        replay->setLoop(false);
        replay->setStopPipelineAtEnd(false);
        auto queue = replay->out.createOutputQueue();
        pipeline.start();
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getType() == dai::ImgFrame::Type::GRAY8);
        REQUIRE(frame->getData().size() == 64u * 48u);
        REQUIRE(frame->getData()[0] == 100);
        pipeline.stop();
    }
    {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(source);
        replay->setOutFrameType(dai::ImgFrame::Type::RGB888i);
        replay->setFps(1000.f);
        replay->setLoop(false);
        replay->setStopPipelineAtEnd(false);
        auto queue = replay->out.createOutputQueue();
        pipeline.start();
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getType() == dai::ImgFrame::Type::RGB888i);
        REQUIRE(std::abs(static_cast<int>(frame->getData()[0]) - 98) <= 2);
        pipeline.stop();
    }
    {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(source);
        replay->setOutFrameType(dai::ImgFrame::Type::RAW16);
        REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("not supported"));
    }

    const auto rawDir = makeRoot("raw16");
    std::vector<std::uint8_t> raw(64u * 48u * 2, 7);
    writeTyped(rawDir / "cam_0.dai", dai::ImgFrame::Type::RAW16, 64, 48, raw, 0);
    auto rawFrames = collect(rawDir, 1, 1000.f, false, false);
    REQUIRE(rawFrames[0]->getType() == dai::ImgFrame::Type::RAW16);
    REQUIRE(std::vector<std::uint8_t>(rawFrames[0]->getData().begin(), rawFrames[0]->getData().end()) == raw);

    const auto grayDir = makeRoot("gray");
    std::vector<std::uint8_t> gray(64u * 48u, 100);
    writeTyped(grayDir / "cam_0.dai", dai::ImgFrame::Type::GRAY8, 64, 48, gray, 0);
    {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(grayDir);
        replay->setOutFrameType(dai::ImgFrame::Type::NV12);
        replay->setFps(1000.f);
        replay->setLoop(false);
        replay->setStopPipelineAtEnd(false);
        auto queue = replay->out.createOutputQueue();
        pipeline.start();
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(meanY(*frame) == Catch::Approx(100.0).margin(0.01));
        const auto uv = frame->getData()[64u * 48u];
        REQUIRE(uv == 128);
        pipeline.stop();
    }
}

TEST_CASE("S7 cache hit keeps a predecoded source") {
    resetCache();
    std::vector<Spec> frames;
    for(int index = 0; index < 8; index++) frames.push_back(Spec{index, index, index * 100, static_cast<std::uint8_t>(20 + index)});
    const auto source = writeStream("s7", 64, 48, frames);
    const auto hash = dai::replay::sha256OfDirectory(source);
    auto run = [&]() {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(source);
        replay->setRunOnHost(true);
        replay->setDecodeMode(dai::ReplayDecodeMode::PRE_DECODE);
        replay->setPersistentUpload(true);
        replay->setFps(1000.f);
        replay->setLoop(false);
        replay->setStopPipelineAtEnd(false);
        auto queue = replay->out.createOutputQueue();
        pipeline.start();
        REQUIRE(queue->get<dai::ImgFrame>() != nullptr);
        pipeline.stop();
    };
    run();
    auto info = dai::node::ReplayVideo::getReplayCacheInfo();
    int sources = 0;
    int decoded = 0;
    std::int64_t firstUse = 0;
    for(const auto& entry : info.entries) {
        if(entry.sourceHash != hash) continue;
        if(entry.kind == dai::ReplayCacheKind::SOURCE) sources++;
        if(entry.kind == dai::ReplayCacheKind::DECODED) decoded++;
        firstUse = std::max(firstUse, entry.lastUseMs);
    }
    REQUIRE(sources == 1);
    REQUIRE(decoded == 1);
    std::this_thread::sleep_for(5ms);
    run();
    info = dai::node::ReplayVideo::getReplayCacheInfo();
    sources = 0;
    decoded = 0;
    std::int64_t secondUse = 0;
    for(const auto& entry : info.entries) {
        if(entry.sourceHash != hash) continue;
        if(entry.kind == dai::ReplayCacheKind::SOURCE) sources++;
        if(entry.kind == dai::ReplayCacheKind::DECODED) decoded++;
        secondUse = std::max(secondUse, entry.lastUseMs);
    }
    REQUIRE(sources == 1);
    REQUIRE(decoded == 1);
    REQUIRE(secondUse >= firstUse);
}

TEST_CASE("TR-18 a non-persistent entry is removed at stop") {
    resetCache();
    const auto source = writeStream("drop", 64, 48, {{0, 0, 0, 10}});
    auto frames = collect(source, 1, 1000.f, false, false);
    REQUIRE(frames[0] != nullptr);
    REQUIRE(dai::node::ReplayVideo::getReplayCacheInfo().entries.empty());
}

TEST_CASE("S8 eviction") {
    resetCache();
    auto makeLarge = [](const std::string& name, std::uint8_t salt) {
        auto dir = makeRoot(name);
        const auto transformation = pinhole(640, 480, 320.f, 320.f, 320.f, 240.f);
        for(int index = 0; index < 8; index++) {
            writeNv12(dir / ("cam_" + std::to_string(index) + ".dai"), index, index * 100, 640, 480, static_cast<std::uint8_t>(index + salt), transformation);
        }
        return dir;
    };
    const auto sourceA = makeLarge("evict-a", 1);
    const auto sourceB = makeLarge("evict-b", 40);
    const auto sourceC = makeLarge("evict-c", 80);
    std::uint64_t bytes = 0;
    for(const auto& entry : std::filesystem::directory_iterator(sourceA)) {
        if(entry.is_regular_file()) bytes += static_cast<std::uint64_t>(entry.file_size());
    }
    const auto twoFit = bytes * 2 + bytes / 2;
    dai::node::ReplayVideo::setReplayCacheLimit(twoFit);
    auto play = [](const std::filesystem::path& source) {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(source);
        replay->setRunOnHost(true);
        replay->setPersistentUpload(true);
        replay->setFps(1000.f);
        replay->setLoop(false);
        replay->setStopPipelineAtEnd(false);
        auto queue = replay->out.createOutputQueue();
        pipeline.start();
        REQUIRE(queue->get<dai::ImgFrame>() != nullptr);
        pipeline.stop();
    };
    const auto hashA = dai::replay::sha256OfDirectory(sourceA);
    const auto hashB = dai::replay::sha256OfDirectory(sourceB);
    const auto hashC = dai::replay::sha256OfDirectory(sourceC);
    auto hashes = []() {
        std::vector<std::string> values;
        for(const auto& entry : dai::node::ReplayVideo::getReplayCacheInfo().entries) {
            if(entry.kind == dai::ReplayCacheKind::SOURCE) values.push_back(entry.sourceHash);
        }
        std::sort(values.begin(), values.end());
        return values;
    };
    play(sourceA);
    std::this_thread::sleep_for(5ms);
    play(sourceB);
    std::this_thread::sleep_for(5ms);
    auto has = [&](const std::string& hash) {
        const auto present = hashes();
        return std::find(present.begin(), present.end(), hash) != present.end();
    };
    play(sourceC);
    REQUIRE_FALSE(has(hashA));
    REQUIRE(has(hashB));
    REQUIRE(has(hashC));

    std::this_thread::sleep_for(5ms);
    play(sourceB);
    REQUIRE_FALSE(has(hashA));
    REQUIRE(has(hashB));
    REQUIRE(has(hashC));

    std::this_thread::sleep_for(5ms);
    play(sourceA);
    REQUIRE(has(hashA));
    REQUIRE(has(hashB));
    REQUIRE_FALSE(has(hashC));

    dai::node::ReplayVideo::setReplayCacheLimit(bytes / 2);
    dai::Pipeline tooBig(false);
    auto replay = tooBig.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(sourceC);
    replay->setRunOnHost(true);
    replay->setPersistentUpload(true);
    REQUIRE_THROWS_WITH(tooBig.start(), ContainsSubstring("Replay cache needs"));
    REQUIRE(has(hashA));
    REQUIRE(has(hashB));
    REQUIRE_FALSE(has(hashC));

    dai::node::ReplayVideo::setReplayCacheLimit(twoFit);
    dai::Pipeline both(false);
    auto nodeA = both.create<dai::node::ReplayVideo>();
    auto nodeC = both.create<dai::node::ReplayVideo>();
    for(auto* node : {nodeA.get(), nodeC.get()}) {
        node->setRunOnHost(true);
        node->setPersistentUpload(true);
        node->setFps(1000.f);
        node->setLoop(false);
        node->setStopPipelineAtEnd(false);
    }
    nodeA->setReplayVideoFile(sourceA);
    nodeC->setReplayVideoFile(sourceC);
    auto queueA = nodeA->out.createOutputQueue();
    auto queueC = nodeC->out.createOutputQueue();
    both.start();
    REQUIRE(queueA->get<dai::ImgFrame>() != nullptr);
    REQUIRE(queueC->get<dai::ImgFrame>() != nullptr);
    both.stop();
    REQUIRE(has(hashA));
    REQUIRE_FALSE(has(hashB));
    REQUIRE(has(hashC));
}

TEST_CASE("S9 unpaced replay does not drop", "[pacing]") {
    resetCache();
    std::vector<Spec> frames;
    for(int index = 0; index < 20; index++) frames.push_back(Spec{index, index, index * 10, 40});
    const auto source = writeStream("s9", 64, 48, frames);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setFps(1000.f);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(false);
    auto queue = replay->out.createOutputQueue(1, true);
    pipeline.start();
    const auto first = std::chrono::steady_clock::now();
    for(int index = 0; index < 20; index++) {
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getSequenceNum() == index);
        std::this_thread::sleep_for(50ms);
    }
    const auto elapsed = std::chrono::steady_clock::now() - first;
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    REQUIRE(millis > 760);
    REQUIRE(millis < 1140);
    pipeline.stop();
}

TEST_CASE("S10 paced replay slips and does not drop", "[pacing]") {
    resetCache();
    std::vector<Spec> frames;
    for(int index = 0; index < 10; index++) frames.push_back(Spec{index, index, index * 10, 40});
    const auto source = writeStream("s10", 64, 48, frames);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setFps(10.f);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(false);
    auto queue = replay->out.createOutputQueue(1, true);
    pipeline.start();
    auto first = queue->get<dai::ImgFrame>();
    const auto start = std::chrono::steady_clock::now();
    std::vector<long long> arrivals{0};
    REQUIRE(first->getSequenceNum() == 0);
    for(int index = 1; index < 10; index++) {
        if(index == 4) std::this_thread::sleep_until(start + 650ms);
        auto frame = queue->get<dai::ImgFrame>();
        REQUIRE(frame->getSequenceNum() == index);
        arrivals.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
    }
    const std::vector<long long> expected{0, 100, 200, 300, 650, 650, 650, 750, 850, 950};
    for(int index = 1; index < 10; index++) {
        REQUIRE(std::llabs(arrivals[static_cast<std::size_t>(index)] - expected[static_cast<std::size_t>(index)]) <= 20);
    }
    pipeline.stop();
}

TEST_CASE("S11 frames that do not match") {
    resetCache();
    const auto mismatch = makeRoot("mismatch");
    const auto transformation = pinhole(64, 48, 32.f, 32.f, 32.f, 24.f);
    writeNv12(mismatch / "cam_0.dai", 0, 0, 64, 48, 10, transformation);
    writeNv12(mismatch / "cam_1.dai", 1, 100, 32, 24, 10, pinhole(32, 24, 16.f, 16.f, 16.f, 12.f));
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(mismatch);
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("cam_1.dai"));

    const auto mixed = makeRoot("mixed");
    writeNv12(mixed / "cam_0.dai", 0, 0, 64, 48, 10, transformation);
    {
        cv::Mat image(48, 64, CV_8UC3, cv::Scalar(10, 20, 30));
        cv::imwrite((mixed / "cam_1.jpg").string(), image);
    }
    dai::Pipeline extensions(false);
    auto extensionReplay = extensions.create<dai::node::ReplayVideo>();
    extensionReplay->setReplayVideoFile(mixed);
    REQUIRE_THROWS_WITH(extensions.start(), ContainsSubstring("more than one extension"));

    const auto duplicate = makeRoot("duplicate");
    writeNv12(duplicate / "cam_1.dai", 1, 0, 64, 48, 10, transformation);
    writeNv12(duplicate / "cam_01.dai", 1, 100, 64, 48, 10, transformation);
    dai::Pipeline duplicates(false);
    auto duplicateReplay = duplicates.create<dai::node::ReplayVideo>();
    duplicateReplay->setReplayVideoFile(duplicate);
    REQUIRE_THROWS_WITH(duplicates.start(), ContainsSubstring("duplicate frame number"));
}

TEST_CASE("S12 one archive is one cache entry") {
    resetCache();
    const auto camA = makeRoot("arc-a");
    const auto camB = makeRoot("arc-b");
    const auto transformation = pinhole(64, 48, 32.f, 32.f, 32.f, 24.f);
    writeNv12(camA / "cam_0.dai", 0, 0, 64, 48, 10, transformation);
    writeNv12(camB / "cam_0.dai", 0, 0, 64, 48, 20, transformation);
    const auto archive = cacheDir() / "sources" / "holistic.tar";
    dai::utility::archiveFiles(archive, {camA / "cam_0.dai", camB / "cam_0.dai"}, {"camA/camA_0.dai", "camB/camB_0.dai"});
    dai::Pipeline pipeline(false);
    auto first = pipeline.create<dai::node::ReplayVideo>();
    auto second = pipeline.create<dai::node::ReplayVideo>();
    for(auto* node : {first.get(), second.get()}) {
        node->setSource(archive);
        node->setRunOnHost(true);
        node->setPersistentUpload(true);
        node->setFps(1000.f);
        node->setLoop(false);
        node->setStopPipelineAtEnd(false);
    }
    first->setArchiveEntry("camA");
    second->setArchiveEntry("camB");
    auto queueA = first->out.createOutputQueue();
    auto queueB = second->out.createOutputQueue();
    pipeline.start();
    REQUIRE(queueA->get<dai::ImgFrame>() != nullptr);
    REQUIRE(queueB->get<dai::ImgFrame>() != nullptr);
    int sources = 0;
    for(const auto& entry : dai::node::ReplayVideo::getReplayCacheInfo().entries) {
        if(entry.kind == dai::ReplayCacheKind::SOURCE) sources++;
    }
    REQUIRE(sources == 1);
    pipeline.stop();
}

TEST_CASE("LP-3 loop off stops the pipeline") {
    resetCache();
    const auto source = writeStream("lp3", 64, 48, {{0, 0, 0, 10}, {1, 1, 100, 20}});
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(true);
    replay->setFps(1000.f);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(true);
    auto queue = replay->out.createOutputQueue();
    pipeline.start();
    REQUIRE(queue->get<dai::ImgFrame>() != nullptr);
    REQUIRE(queue->get<dai::ImgFrame>() != nullptr);
    pipeline.wait();
    // stopPipeline() hands shutdown to another thread so it never joins the node thread.
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while(pipeline.isRunning() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(10ms);
    REQUIRE_FALSE(pipeline.isRunning());
}

TEST_CASE("ReplayVideo odd source size and unknown extension") {
    resetCache();
    const auto odd = makeRoot("odd-jpg");
    cv::Mat image(481, 641, CV_8UC3, cv::Scalar(98, 98, 98));
    cv::imwrite((odd / "cam_0.jpg").string(), image);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(odd);
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("641x481"));

    const auto text = cacheDir() / "sources" / "notes.txt";
    std::filesystem::create_directories(text.parent_path());
    {
        std::ofstream output(text);
        output << "no";
    }
    dai::Pipeline unknown(false);
    auto unknownReplay = unknown.create<dai::node::ReplayVideo>();
    unknownReplay->setReplayVideoFile(text);
    REQUIRE_THROWS_WITH(unknown.start(), ContainsSubstring("unknown extension"));
}

TEST_CASE("Replay JPEG and PNG archives decode every image in numeric order", "[archive]") {
    resetCache();
    const auto imageExtension = GENERATE(".jpg", ".png");
    const auto archiveExtension = GENERATE(".tar", ".tar.gz", ".tgz", ".tar.xz");
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    replaytest::ImageArchive source(imageExtension, archiveExtension, true);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(true).setLoop(true).setStopPipelineAtEnd(false).setFps(1000.f).setDecodeMode(mode);
    auto queue = replay->out.createOutputQueue(8, true);
    pipeline.start();
    std::vector<int> luma;
    for(int index = 0; index < 6; ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(3s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        REQUIRE(frame->getWidth() == 18);
        REQUIRE(frame->getHeight() == 10);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::YUV420p);
        REQUIRE(frame->getSequenceNum() == index);
        REQUIRE(frame->getData().size() >= 18 * 10 * 3 / 2);
        luma.push_back(frame->getData()[0]);
    }
    pipeline.stop();
    REQUIRE(luma[1] > luma[0] + 20);
    REQUIRE(luma[2] > luma[1] + 20);
    REQUIRE(luma[3] == luma[0]);
    REQUIRE(luma[4] == luma[1]);
    REQUIRE(luma[5] == luma[2]);
}

TEST_CASE("Replay archives resize odd and one-pixel source images", "[archive]") {
    resetCache();
    const auto dimensions = GENERATE(std::make_pair(17, 9), std::make_pair(1, 1));
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    replaytest::ImageArchive source(".png", ".tar.gz", true, dimensions.first, dimensions.second);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(true).setLoop(false).setStopPipelineAtEnd(false).setDecodeMode(mode);
    replay->setSize(16, 8);
    auto queue = replay->out.createOutputQueue(8, true);
    pipeline.start();
    for(int index = 0; index < 3; ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(3s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        REQUIRE(frame->getWidth() == 16);
        REQUIRE(frame->getHeight() == 8);
        REQUIRE(frame->getData().size() >= 16 * 8 * 3 / 2);
    }
    pipeline.stop();
}

TEST_CASE("Replay image resize retains chroma from a one-pixel source", "[archive]") {
    dai::replay::DecodedPixels source;
    source.width = 1;
    source.height = 1;
    source.type = dai::ImgFrame::Type::BGR888i;
    source.data = {0, 0, 255};
    const auto pixels = dai::replay::convertPixels(source, dai::ImgFrame::Type::YUV420p, 16, 8);
    REQUIRE(pixels.data.size() == 16 * 8 * 3 / 2);
    REQUIRE(pixels.data[0] == 82);
    REQUIRE(pixels.data[16 * 8] == 90);
    REQUIRE(pixels.data[16 * 8 * 5 / 4] == 240);
}

TEST_CASE("PC-2 recorded frame gaps determine wall-clock playback", "[pacing]") {
    resetCache();
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    const auto explicitlyUnset = GENERATE(false, true);
    const auto source = writeStream("recorded-pacing", 64, 48, {{0, 10, 1000, 20}, {1, 11, 1080, 30}, {2, 12, 1080, 40}, {3, 13, 1300, 50}});
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source).setRunOnHost(true).setDecodeMode(mode).setLoop(true).setStopPipelineAtEnd(false);
    if(explicitlyUnset) replay->setFps(0.f);
    auto queue = replay->out.createOutputQueue(1, true);
    pipeline.start();
    std::chrono::steady_clock::time_point start;
    const std::vector<long long> expected{0, 80, 113, 333, 433, 513};
    const std::vector<long long> stamps{1000, 1080, 1080, 1300, 1400, 1480};
    for(int index = 0; index < 6; ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(2s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        const auto received = std::chrono::steady_clock::now();
        if(index == 0) start = received;
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(received - start).count();
        REQUIRE(std::llabs(elapsed - expected[index]) <= 25);
        REQUIRE(frame->getSequenceNum() == 10 + index);
        const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(frame->getTimestampDevice().time_since_epoch()).count();
        REQUIRE(stamp == stamps[index]);
    }
    pipeline.stop();
}

TEST_CASE("PC-3 frames without metadata use actual send timestamps at 30 fps", "[pacing]") {
    resetCache();
    replaytest::ImageArchive source(".png", ".tar", true);
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source.archive).setRunOnHost(true).setLoop(false).setStopPipelineAtEnd(false);
    auto queue = replay->out.createOutputQueue(1, true);
    pipeline.start();
    std::optional<std::chrono::steady_clock::time_point> previous;
    for(int index = 0; index < 3; ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(2s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        const auto received = std::chrono::steady_clock::now();
        const auto sent = frame->getTimestampDevice();
        REQUIRE(sent <= received);
        REQUIRE(received - sent < 25ms);
        if(previous) {
            REQUIRE(sent - *previous >= std::chrono::nanoseconds(33333333));
            REQUIRE(sent - *previous < 60ms);
        }
        previous = sent;
    }
    pipeline.stop();
}

TEST_CASE("Replay rejects a negative fps at start", "[pacing]") {
    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setFps(-1.f);
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("fps must be 0 or more"));
}
