extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <algorithm>
#include <atomic>
#include <fstream>
#include <optional>
#include <regex>
#include <set>
#include <thread>

#include "../../../helpers/replay_test_support.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/utility/Compression.hpp"

using Catch::Matchers::ContainsSubstring;
using record_test::deleteRecording;
using record_test::LineLevel;
using record_test::timestampMs;
using replay_test::buildCamera;
using replay_test::CacheKind;
using replay_test::clearReplayCache;
using replay_test::countKind;
using replay_test::DecodeMode;
using replay_test::DeviceReplay;
using replay_test::FrameFill;
using replay_test::getReplayCacheInfo;
using replay_test::hashDir;
using replay_test::hashFile;
using replay_test::hashFileAndMetadata;
using replay_test::HostReplay;
using replay_test::Location;
using replay_test::makeDaiDir;
using replay_test::makeDaiFrame;
using replay_test::makeDeviceSource;
using replay_test::makeHolisticDir;
using replay_test::makeTar;
using replay_test::meanY;
using replay_test::pixelsEqual;
using replay_test::rangeSeq;
using replay_test::rangeTs;
using replay_test::replayDataAvailable;
using replay_test::replayDataRoot;
using replay_test::setArchiveEntry;
using replay_test::setDecodeMode;
using replay_test::setFast;
using replay_test::setFps;
using replay_test::setLoop;
using replay_test::setMetadataFile;
using replay_test::setOutFrameType;
using replay_test::setPersistentUpload;
using replay_test::setReplayCacheLimit;
using replay_test::setReplayFile;
using replay_test::setRunOnHost;
using replay_test::setSource;
using replay_test::setStopPipelineAtEnd;
using replay_test::takeFrames;
using replay_test::TarKind;

namespace {

constexpr const char* kHolisticConfig =
    R"({"outputDir":".","videoEncoding":{"enabled":true,"bitrate":0,"profile":"H264_MAIN","lossless":false,"quality":80},"compressionLevel":3,"syncCameraOutputs":true,"format":"VIDEO","streams":{"CameraCAM_A":{"fps":30},"CameraCAM_B":{"fps":30},"CameraCAM_C":{"fps":30}}})";

constexpr const char* kHolisticConfigNoStreams =
    R"({"outputDir":".","videoEncoding":{"enabled":true,"bitrate":0,"profile":"H264_MAIN","lossless":false,"quality":80},"compressionLevel":3,"syncCameraOutputs":true,"format":"VIDEO"})";

struct Play {
    std::shared_ptr<dai::node::ReplayVideo> node;
    std::shared_ptr<dai::MessageQueue> out;
};

void requireData() {
    if(!replayDataAvailable()) SKIP("needs the generated data");
}

void requireStart(dai::Pipeline& pipeline, const char* text = nullptr) {
    if(text == nullptr) {
        REQUIRE_THROWS(pipeline.start());
    } else {
        REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring(text));
    }
}

Play addPlay(dai::Pipeline& pipeline, const std::filesystem::path& source, bool persistent, Location location = Location::HOST) {
    Play play;
    play.node = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*play.node);
    setSource(*play.node, source, location);
    setLoop(*play.node, false);
    setStopPipelineAtEnd(*play.node, false);
    setFps(*play.node, 1000.f);
    if(persistent) setPersistentUpload(*play.node, true);
    play.out = play.node->out.createOutputQueue(16, true);
    return play;
}

std::filesystem::path makeFrames(const std::filesystem::path& dir, int width, int height, int count, FrameFill fill = {}) {
    makeDaiDir(dir, width, height, rangeSeq(count), rangeTs(count), fill);
    return dir;
}

std::set<std::string> sourceHashes(const std::shared_ptr<dai::Device>& device) {
    return replay_test::hashesOf(getReplayCacheInfo(*device), CacheKind::SOURCE);
}

const dai::ReplayCacheEntry* entryWith(const dai::ReplayCacheInfo& info, CacheKind kind, const std::string& hash) {
    for(const auto& entry : info.entries) {
        if(entry.kind == kind && entry.sourceHash == hash) return &entry;
    }
    return nullptr;
}

void saveShortFrame(const std::filesystem::path& path, std::int64_t sequence) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setType(dai::ImgFrame::Type::NV12);
    frame->setWidth(64);
    frame->setHeight(48);
    frame->setData(std::vector<std::uint8_t>(100, 7));
    frame->setSequenceNum(sequence);
    const auto when = record_test::steadyAtMs(sequence * 100);
    frame->setTimestamp(when);
    frame->setTimestampDevice(when);
    frame->save(path);
}

std::uint64_t directoryBytes(const std::filesystem::path& dir) {
    std::uint64_t sum = 0;
    for(const auto& entry : std::filesystem::directory_iterator(dir)) {
        if(entry.is_regular_file()) sum += static_cast<std::uint64_t>(entry.file_size());
    }
    return sum;
}

std::string cameraInfoFor(const std::shared_ptr<dai::Device>& device) {
    if(device->getConnectedCameras().size() < 3) return {};
    static std::string cached;
    if(!cached.empty()) return cached;
    record_test::TempDir scratch;
    dai::Pipeline pipeline(device);
    dai::RecordConfig config;
    config.outputDir = scratch.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        auto camera = pipeline.create<dai::node::Camera>()->build(socket);
        auto* output = camera->requestOutput({1280, 720}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);
        (void)output->createOutputQueue(4, false);
    }
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    pipeline.stop();
    std::regex name("^recording_.*\\.tar$");
    std::filesystem::path tar;
    for(const auto& entry : std::filesystem::directory_iterator(scratch.path)) {
        if(entry.is_regular_file() && std::regex_match(entry.path().filename().string(), name)) tar = entry.path();
    }
    REQUIRE(!tar.empty());
    const auto bytes = dai::utility::readFileInArchive(tar, "camera_info.json");
    cached.assign(bytes.begin(), bytes.end());
    return cached;
}

}  // namespace

TEST_CASE("VA1 three frames upload one source entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto hash = hashDir(dir);
    CHECK(fx.devLog.count(LineLevel::INFO, "Replay cache upload " + hash) >= 1);
    CHECK(sourceHashes(fx.device) == std::set<std::string>{hash});
}

TEST_CASE("VA2 the second run hits the cache", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto hash = hashDir(dir);
    const auto* before = entryWith(getReplayCacheInfo(*fx.device), CacheKind::SOURCE, hash);
    REQUIRE(before != nullptr);
    const auto used = before->lastUseMs;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const int uploads = fx.devLog.countSubstring("Replay cache upload");
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto* after = entryWith(getReplayCacheInfo(*fx.device), CacheKind::SOURCE, hash);
    REQUIRE(after != nullptr);
    CHECK(after->lastUseMs > used);
    CHECK(fx.devLog.countSubstring("Replay cache hit " + hash) >= 1);
    CHECK(fx.devLog.countSubstring("Replay cache upload") == uploads);
}

TEST_CASE("VA3 a changed frame uploads a new hash", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto first = hashDir(dir);
    auto loaded = record_test::loadImg(dir / "cam_0.dai");
    REQUIRE(loaded != nullptr);
    auto bytes = std::vector<std::uint8_t>(loaded->getData().begin(), loaded->getData().end());
    REQUIRE(!bytes.empty());
    bytes[0] = static_cast<std::uint8_t>(bytes[0] ^ 0x1u);
    loaded->setData(bytes);
    loaded->save(dir / "cam_0.dai");
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto second = hashDir(dir);
    CHECK(second != first);
    CHECK(fx.devLog.count(LineLevel::INFO, "Replay cache upload " + second) >= 1);
    CHECK(sourceHashes(fx.device) == std::set<std::string>({first, second}));
}

TEST_CASE("VA4 notes.txt changes the directory hash", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    pipeline.stop();
    const int uploads = fx.devLog.countSubstring("Replay cache upload");
    std::ofstream(dir / "notes.txt") << "notes";
    pipeline.start();
    pipeline.stop();
    CHECK(fx.devLog.countSubstring("Replay cache upload") == uploads + 1);
    CHECK(sourceHashes(fx.device).count(hashDir(dir)) == 1);
}

TEST_CASE("VA5 a dot file does not change the hash", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    pipeline.stop();
    const auto hash = hashDir(dir);
    const int uploads = fx.devLog.countSubstring("Replay cache upload");
    std::ofstream(dir / "._x") << "dot";
    pipeline.start();
    pipeline.stop();
    CHECK(fx.devLog.countSubstring("Replay cache hit " + hash) >= 1);
    CHECK(fx.devLog.countSubstring("Replay cache upload") == uploads);
}

TEST_CASE("VA6 a tar hash differs from the directory hash", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    const auto tar = makeTar(fx.dir.path / "flat.tar", dir, TarKind::FLAT);
    auto play = addPlay(pipeline, tar, true);
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto hash = hashFile(tar);
    CHECK(hash != hashDir(dir));
    CHECK(sourceHashes(fx.device) == std::set<std::string>{hash});
}

TEST_CASE("VA7 video plus metadata uses the combined hash", "[replay][device]") {
    requireData();
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto video = replayDataRoot() / "plain_h264.mp4";
    const auto meta = fx.dir.path / "meta30.mcap";
    replay_test::writeMeta(meta, 30, 100, 2000, 1920, 1080, 100);
    auto play = addPlay(pipeline, video, true);
    setMetadataFile(*play.node, meta);
    pipeline.start();
    pipeline.stop();
    const auto hash = hashFileAndMetadata(video, meta);
    CHECK(sourceHashes(fx.device) == std::set<std::string>{hash});
}

TEST_CASE("VA8 two nodes upload one source", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto first = addPlay(pipeline, dir, true);
    auto second = addPlay(pipeline, dir, true);
    pipeline.start();
    CHECK(takeFrames(first.out, 3).size() == 3);
    CHECK(takeFrames(second.out, 3).size() == 3);
    pipeline.stop();
    CHECK(fx.devLog.countSubstring("Replay cache upload") == 1);
    CHECK(sourceHashes(fx.device).size() == 1);
}

TEST_CASE("VA9 predecode keeps a source and a decoded entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 100);
    auto play = addPlay(pipeline, dir, true);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    REQUIRE(fx.devLog.waitFor("100/100", std::chrono::seconds(60)));
    pipeline.stop();
    const auto hash = hashDir(dir);
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(countKind(info, CacheKind::SOURCE) == 1);
    CHECK(countKind(info, CacheKind::DECODED) == 1);
    const auto* source = entryWith(info, CacheKind::SOURCE, hash);
    const auto* decoded = entryWith(info, CacheKind::DECODED, hash);
    REQUIRE(source != nullptr);
    REQUIRE(decoded != nullptr);
    CHECK(source->key != decoded->key);
}

TEST_CASE("VA10 two output types keep two decoded entries", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto color = addPlay(pipeline, dir, true);
    auto gray = addPlay(pipeline, dir, true);
    setDecodeMode(*color.node, DecodeMode::PRE_DECODE);
    setDecodeMode(*gray.node, DecodeMode::PRE_DECODE);
    setOutFrameType(*gray.node, dai::ImgFrame::Type::GRAY8);
    pipeline.start();
    auto grayFrames = takeFrames(gray.out, 3);
    auto colorFrames = takeFrames(color.out, 3);
    CHECK(grayFrames[0]->getType() == dai::ImgFrame::Type::GRAY8);
    CHECK(colorFrames[0]->getType() == dai::ImgFrame::Type::NV12);
    pipeline.stop();
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(countKind(info, CacheKind::SOURCE) == 1);
    CHECK(countKind(info, CacheKind::DECODED) == 2);
    REQUIRE(info.entries.size() >= 2);
    CHECK(info.entries[0].key != info.entries[1].key);
}

TEST_CASE("VA11 sibling directories stay separate cache entries", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto holistic = fx.dir.path / "holistic";
    makeFrames(holistic / "CameraCAM_C", 64, 48, 60, FrameFill::plainYuv(50, 128, 128));
    const auto other = makeFrames(fx.dir.path / "A", 640, 480, 3);
    auto nodeC = addPlay(pipeline, holistic / "CameraCAM_C", true);
    auto nodeA = addPlay(pipeline, other, true);
    setDecodeMode(*nodeC.node, DecodeMode::PRE_DECODE);
    setDecodeMode(*nodeA.node, DecodeMode::PRE_DECODE);
    setLoop(*nodeC.node, false);
    pipeline.start();
    auto framesC = takeFrames(nodeC.out, 60);
    auto framesA = takeFrames(nodeA.out, 3);
    CHECK(std::fabs(meanY(*framesC[0]) - 50.0) <= 1.0);
    CHECK(framesA.size() == 3);
    pipeline.stop();
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(countKind(info, CacheKind::SOURCE) == 2);
    CHECK(countKind(info, CacheKind::DECODED) == 2);
    CHECK(hashDir(holistic / "CameraCAM_C") != hashDir(other));
}

TEST_CASE("VA12 three directories evict down to the two that fit", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 3, FrameFill::plainYuv(20, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    const std::vector<std::filesystem::path> order{dirA, dirB, dirC, dirB, dirA};
    const std::vector<std::set<std::string>> expected{
        {hashDir(dirA)}, {hashDir(dirA), hashDir(dirB)}, {hashDir(dirB), hashDir(dirC)}, {hashDir(dirB), hashDir(dirC)}, {hashDir(dirA), hashDir(dirB)}};
    for(std::size_t step = 0; step < order.size(); ++step) {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, order[step], true);
        pipeline.start();
        CHECK(takeFrames(play.out, 3).size() == 3);
        pipeline.stop();
        CHECK(sourceHashes(fx.device) == expected[step]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        fx.pipeline->stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
}

TEST_CASE("VA13 a one megabyte limit names the size of C", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 3, FrameFill::plainYuv(20, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    for(const auto& dir : {dirA, dirB, dirC, dirB, dirA}) {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, dir, true);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    setReplayCacheLimit(*fx.device, 1000000);
    const auto kept = sourceHashes(fx.device);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dirC, true);
    requireStart(pipeline, std::to_string(directoryBytes(dirC)).c_str());
    CHECK(sourceHashes(fx.device) == kept);
    CHECK(kept == std::set<std::string>({hashDir(dirA), hashDir(dirB)}));
}

TEST_CASE("VA14 two nodes evict B and keep A with C", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 3, FrameFill::plainYuv(20, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    {
        auto& pipeline = fx.begin();
        auto playA = addPlay(pipeline, dirA, true);
        auto playB = addPlay(pipeline, dirB, true);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    auto& pipeline = fx.begin();
    auto playA = addPlay(pipeline, dirA, true);
    auto playC = addPlay(pipeline, dirC, true);
    (void)playA;
    (void)playC;
    pipeline.start();
    pipeline.stop();
    CHECK(sourceHashes(fx.device) == std::set<std::string>({hashDir(dirA), hashDir(dirC)}));
}

TEST_CASE("VA15 predecode of C evicts A and B", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 3, FrameFill::plainYuv(20, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    {
        auto& pipeline = fx.begin();
        addPlay(pipeline, dirA, true);
        addPlay(pipeline, dirB, true);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dirC, true);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    pipeline.stop();
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(countKind(info, CacheKind::SOURCE) == 1);
    CHECK(countKind(info, CacheKind::DECODED) == 1);
    CHECK(entryWith(info, CacheKind::SOURCE, hashDir(dirC)) != nullptr);
    CHECK(entryWith(info, CacheKind::DECODED, hashDir(dirC)) != nullptr);
}

TEST_CASE("VA16 a stream run drops the decoded unit", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, dirC, true);
        setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dirA, true);
    setDecodeMode(*play.node, DecodeMode::STREAM);
    pipeline.start();
    pipeline.stop();
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(sourceHashes(fx.device) == std::set<std::string>{hashDir(dirA)});
    CHECK(countKind(info, CacheKind::DECODED) == 0);
}

TEST_CASE("VA17 the larger entry at the same time is evicted first", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 4500000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 4, FrameFill::plainYuv(20, 128, 128));
    const auto dirC = makeFrames(fx.dir.path / "C", 640, 480, 3, FrameFill::plainYuv(30, 128, 128));
    {
        auto& pipeline = fx.begin();
        addPlay(pipeline, dirA, true);
        addPlay(pipeline, dirB, true);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dirC, true);
    pipeline.start();
    pipeline.stop();
    CHECK(sourceHashes(fx.device) == std::set<std::string>({hashDir(dirA), hashDir(dirC)}));
}

TEST_CASE("VA18 the cache limit survives a closed device", "[replay][device]") {
    auto first = record_test::openRvc4OrSkip();
    const auto saved = getReplayCacheInfo(*first).limitBytes;
    first.reset();
    {
        dai::Device second;
        second.setReplayCacheLimit(3000000);
    }
    dai::Device third;
    replay_test::CacheGuard guard(third);
    CHECK(getReplayCacheInfo(third).limitBytes == 3000000);
    third.setReplayCacheLimit(saved);
}

TEST_CASE("VA19 lowering the limit does not evict by itself", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 3000000);
    const auto dirA = makeFrames(fx.dir.path / "A", 640, 480, 3, FrameFill::plainYuv(10, 128, 128));
    const auto dirB = makeFrames(fx.dir.path / "B", 640, 480, 3, FrameFill::plainYuv(20, 128, 128));
    auto& pipeline = fx.begin();
    addPlay(pipeline, dirA, true);
    addPlay(pipeline, dirB, true);
    pipeline.start();
    pipeline.stop();
    setReplayCacheLimit(*fx.device, 1000000);
    CHECK(sourceHashes(fx.device) == std::set<std::string>({hashDir(dirA), hashDir(dirB)}));
}

TEST_CASE("VA20 clear removes the uploaded entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    pipeline.stop();
    clearReplayCache(*fx.device);
    CHECK(getReplayCacheInfo(*fx.device).entries.empty());
}

TEST_CASE("VA21 an entry in use survives clear", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "A", 64, 48, 3);
    auto play = addPlay(pipeline, dir, true);
    pipeline.start();
    REQUIRE(takeFrames(play.out, 1).size() == 1);
    const auto during = getReplayCacheInfo(*fx.device);
    const auto* live = entryWith(during, CacheKind::SOURCE, hashDir(dir));
    REQUIRE(live != nullptr);
    CHECK(live->inUse);
    clearReplayCache(*fx.device);
    CHECK(entryWith(getReplayCacheInfo(*fx.device), CacheKind::SOURCE, hashDir(dir)) != nullptr);
    pipeline.stop();
}

TEST_CASE("VA22 stopping an upload returns an error and no entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 87);
    auto play = addPlay(pipeline, dir, true);
    std::exception_ptr error;
    std::atomic<bool> returned{false};
    std::thread worker([&] {
        try {
            pipeline.start();
            returned = true;
        } catch(...) {
            error = std::current_exception();
        }
    });
    const bool saw = fx.devLog.waitFor("Replay upload", std::chrono::seconds(30));
    pipeline.stop();
    worker.join();
    if(!saw || returned) SKIP("start was too fast");
    REQUIRE(error != nullptr);
    CHECK(getReplayCacheInfo(*fx.device).entries.empty());
}

TEST_CASE("VA23 a source over the limit errors before upload", "[replay][device]") {
    DeviceReplay fx;
    setReplayCacheLimit(*fx.device, 1000000);
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 3);
    auto play = addPlay(pipeline, dir, true);
    try {
        pipeline.start();
        FAIL("expected an error at the start");
    } catch(const std::exception& error) {
        const std::string text = error.what();
        INFO(text);
        CHECK(text.find(std::to_string(directoryBytes(dir))) != std::string::npos);
        CHECK(text.find("1000000") != std::string::npos);
    }
    CHECK(fx.devLog.countSubstring("Replay cache upload") == 0);
}

TEST_CASE("VA24 duplicate names error before upload", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = fx.dir.path / "src";
    makeDaiDir(dir, 64, 48, {1, 2}, {100, 200}, {}, {"cam_1.dai", "cam_01.dai"});
    auto play = addPlay(pipeline, dir, true);
    requireStart(pipeline);
    CHECK(fx.devLog.countSubstring("upload") == 0);
}

TEST_CASE("VA25 a bad dai stays cached after the upload", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = fx.dir.path / "src";
    std::filesystem::create_directories(dir);
    replay_test::makeDaiOther(dir / "cam_0.dai", replay_test::OtherKind::IMU);
    auto play = addPlay(pipeline, dir, true);
    requireStart(pipeline);
    CHECK(fx.devLog.countSubstring("Replay cache upload") >= 1);
    CHECK(sourceHashes(fx.device).size() == 1);
}

TEST_CASE("VA26 an exact byte limit starts and one byte less does not", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 3);
    const auto bytes = directoryBytes(dir);
    setReplayCacheLimit(*fx.device, bytes);
    {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, dir, true);
        pipeline.start();
        CHECK(pipeline.isRunning());
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    clearReplayCache(*fx.device);
    setReplayCacheLimit(*fx.device, bytes - 1);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, true);
    requireStart(pipeline);
}

TEST_CASE("VA27 a device source is not uploaded", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    const auto devicePath = makeDeviceSource(fx.device, dir);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, devicePath, true, Location::DEVICE);
    pipeline.start();
    auto frames = takeFrames(play.out, 3);
    CHECK(frames.size() == 3);
    CHECK(frames[0]->getSequenceNum() == 0);
    CHECK(frames[2]->getSequenceNum() == 2);
    pipeline.stop();
    CHECK(countKind(getReplayCacheInfo(*fx.device), CacheKind::SOURCE) == 0);
    CHECK(fx.devLog.countSubstring("upload") == 0);
    deleteRecording(*fx.device, std::filesystem::path(devicePath).parent_path().filename().string());
}

TEST_CASE("VA28 a device source reuses one decoded entry", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    const auto devicePath = makeDeviceSource(fx.device, dir);
    std::string hash;
    {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, devicePath, true, Location::DEVICE);
        setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
        pipeline.start();
        CHECK(takeFrames(play.out, 3).size() == 3);
        pipeline.stop();
        const auto info = getReplayCacheInfo(*fx.device);
        CHECK(countKind(info, CacheKind::SOURCE) == 0);
        CHECK(countKind(info, CacheKind::DECODED) == 1);
        hash = info.entries[0].sourceHash;
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    const int decodes = fx.devLog.countSubstring("Replay decode");
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, devicePath, true, Location::DEVICE);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    pipeline.stop();
    const auto info = getReplayCacheInfo(*fx.device);
    CHECK(countKind(info, CacheKind::DECODED) == 1);
    CHECK(countKind(info, CacheKind::SOURCE) == 0);
    CHECK(info.entries[0].sourceHash == hash);
    CHECK(fx.devLog.countSubstring("Replay decode") == decodes);
    deleteRecording(*fx.device, std::filesystem::path(devicePath).parent_path().filename().string());
}

TEST_CASE("VA29 a missing device directory errors at start", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, "/data/depthai/no_such_dir", true, Location::DEVICE);
    requireStart(pipeline);
}

TEST_CASE("VA30 a relative device source errors at start", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, "depthai/relative", true, Location::DEVICE);
    requireStart(pipeline);
}

TEST_CASE("VA32 the default upload drops the entry at stop", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, false);
    pipeline.start();
    REQUIRE(takeFrames(play.out, 1).size() == 1);
    const auto during = getReplayCacheInfo(*fx.device);
    const auto* live = entryWith(during, CacheKind::SOURCE, hashDir(dir));
    REQUIRE(live != nullptr);
    CHECK(live->inUse);
    pipeline.stop();
    CHECK(getReplayCacheInfo(*fx.device).entries.empty());
    CHECK(fx.devLog.countSubstring("Replay cache upload") == 1);
}

TEST_CASE("VA33 a non-persistent predecode drops both entries", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    REQUIRE(takeFrames(play.out, 1).size() == 1);
    const auto during = getReplayCacheInfo(*fx.device);
    CHECK(countKind(during, CacheKind::SOURCE) == 1);
    CHECK(countKind(during, CacheKind::DECODED) == 1);
    pipeline.stop();
    CHECK(getReplayCacheInfo(*fx.device).entries.empty());
}

TEST_CASE("VA34 a later non-persistent run keeps the persistent entry", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    {
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, dir, true);
        pipeline.start();
        pipeline.stop();
        fx.recordings.reset();
        fx.pipeline.reset();
    }
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    pipeline.start();
    pipeline.stop();
    CHECK(fx.devLog.countSubstring("Replay cache hit") >= 1);
    CHECK(sourceHashes(fx.device).count(hashDir(dir)) == 1);
}

TEST_CASE("VA35 one persistent node keeps the shared entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto kept = addPlay(pipeline, dir, true);
    auto transient = addPlay(pipeline, dir, false);
    (void)kept;
    (void)transient;
    pipeline.start();
    pipeline.stop();
    CHECK(fx.devLog.countSubstring("Replay cache upload") == 1);
    CHECK(sourceHashes(fx.device).size() == 1);
}

TEST_CASE("VA36 an error with the default upload drops the entry", "[replay][device]") {
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    const auto dir = fx.dir.path / "src";
    makeDaiDir(dir, 64, 48, {0, 1}, {0, 100});
    saveShortFrame(dir / "cam_2.dai", 2);
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::STREAM);
    pipeline.start();
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(15)));
    CHECK(getReplayCacheInfo(*fx.device).entries.empty());
}

TEST_CASE("VA31 a source that is still recording errors at start", "[replay][device]") {
    DeviceReplay fx;
    const auto name = record_test::uniqueName("VA31");
    std::string devicePath;
    {
        dai::Pipeline recording(fx.device);
        auto record = recording.create<dai::node::RecordVideo>();
        record_test::setFormat(*record, record_test::Format::RAW_FRAMES);
        record_test::setRecordingName(*record, name);
        record_test::setOutputDirectory(*record, fx.dir.path);
        record_test::setStreamName(*record, "cam");
        record_test::setTransferToHost(*record, false);
        record_test::setKeepOnDevice(*record, true);
        auto input = record->input.createInputQueue();
        recording.start();
        record_test::sendFrame(input, record_test::makeImg(64, 48, 0));
        REQUIRE(fx.devLog.waitFor("Record wrote frame 0", std::chrono::seconds(10)));
        devicePath = record_test::getStatus(*record)->path + "/cam";
        auto& pipeline = fx.begin();
        auto play = addPlay(pipeline, devicePath, true, Location::DEVICE);
        requireStart(pipeline);
        recording.stop();
    }
    deleteRecording(*fx.device, name);
}

#include "replay_node_device_catalog.inc"
