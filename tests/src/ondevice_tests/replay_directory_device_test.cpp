#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <fstream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "../ReplayArchiveFixture.hpp"
#include "depthai/depthai.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "opencv/ReplayInternal.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace {

std::shared_ptr<dai::Device> openRvc4() {
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(info.platform == X_LINK_RVC4) return std::make_shared<dai::Device>(info);
    }
    return nullptr;
}

struct RestoreLimit {
    std::shared_ptr<dai::Device>& device;
    std::uint64_t limit;
    ~RestoreLimit() {
        try {
            if(device && !device->isClosed()) device->setReplayCacheLimit(limit);
        } catch(const std::exception&) {
        }
    }
};

struct AbortUpload {
    std::shared_ptr<dai::Device> device;
    std::string hash;
    ~AbortUpload() {
        try {
            device->replayCacheAbort(hash);
        } catch(const std::exception&) {
        }
    }
};

std::uint64_t directoryBytes(const std::filesystem::path& path) {
    std::uint64_t bytes = 0;
    for(const auto& item : std::filesystem::directory_iterator(path)) {
        if(item.is_regular_file() && item.path().filename().string().front() != '.') bytes += item.file_size();
    }
    return bytes;
}

std::string hashText(const std::string& text) {
    const std::vector<std::uint8_t> bytes(text.begin(), text.end());
    return dai::replay::sha256OfBytes(bytes.data(), bytes.size());
}

void writeImages(const std::filesystem::path& root, const std::string& extension) {
    for(const auto number : {10, 30, 2}) {
        const int value = number == 2 ? 40 : number == 10 ? 100 : 160;
        const auto path = root / ("cam_" + std::to_string(number) + extension);
        if(extension == ".dai") {
            dai::ImgFrame frame;
            frame.setType(dai::ImgFrame::Type::NV12);
            frame.setSize(18, 10);
            frame.setSequenceNum(number);
            const auto time = std::chrono::steady_clock::time_point(std::chrono::milliseconds(number * 10));
            frame.setTimestamp(time);
            frame.setTimestampDevice(time);
            std::vector<std::uint8_t> bytes(18 * 10 * 3 / 2, 128);
            std::fill_n(bytes.begin(), 18 * 10, static_cast<std::uint8_t>(value));
            frame.setData(bytes);
            frame.save(path);
        } else {
            REQUIRE(cv::imwrite(path.string(), cv::Mat(10, 18, CV_8UC3, cv::Scalar(value, value, value))));
        }
    }
    std::ofstream(root / "readme.txt") << "included in the directory hash: " << root.string();
    std::ofstream(root / ".hidden") << "not uploaded";
    std::filesystem::create_directory(root / "nested");
    std::ofstream(root / "nested" / "another_1.png") << "not a direct frame";
}

void receive(const std::shared_ptr<dai::MessageQueue>& queue, bool recorded) {
    int previous = -100;
    const std::vector<std::int64_t> sequences = recorded ? std::vector<std::int64_t>{2, 10, 30} : std::vector<std::int64_t>{0, 1, 2};
    for(const auto sequence : sequences) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(5), timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        REQUIRE(frame->getWidth() == 18);
        REQUIRE(frame->getHeight() == 10);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(frame->getSequenceNum() == sequence);
        REQUIRE_FALSE(frame->getData().empty());
        const int value = frame->getData()[frame->fb.p1Offset];
        REQUIRE(value > previous + 20);
        previous = value;
    }
}

}  // namespace

TEST_CASE("RVC4 cache limit dispatches its uint64 argument and persists", "[directory][cache][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    const auto before = device->getReplayCacheInfo();
    RestoreLimit restore{device, before.limitBytes};
    for(const std::uint64_t limit : {std::uint64_t{0}, std::uint64_t{16} << 30}) {
        device->setReplayCacheLimit(limit);
        const auto after = device->getReplayCacheInfo();
        REQUIRE(after.limitBytes == limit);
        REQUIRE(after.usedBytes == before.usedBytes);
        REQUIRE(after.entries.size() == before.entries.size());
    }
    device->close();
    device = openRvc4();
    REQUIRE(device);
    REQUIRE(device->getReplayCacheInfo().limitBytes == (std::uint64_t{16} << 30));
}

TEST_CASE("RVC4 replays unpacked host directories and reuses their content hash", "[directory][cache][rvc4]") {
    const auto extension = GENERATE(".jpg", ".png", ".dai");
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    replaytest::Directory source;
    // A directory's name must not select archive decoding.
    const auto frames = source.path / "frames.tar";
    std::filesystem::create_directory(frames);
    writeImages(frames, extension);
    const auto hash = dai::replay::sha256OfDirectory(frames);
    const auto bytes = directoryBytes(frames);
    {
        auto device = openRvc4();
        if(!device) SKIP("No RVC4 connected");
        dai::Pipeline pipeline(device);
        std::vector<std::shared_ptr<dai::MessageQueue>> queues;
        for(int index = 0; index < 2; ++index) {
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setSource(frames).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setFps(30).setDecodeMode(mode);
            replay->setPersistentUpload(true).setOutFrameType(dai::ImgFrame::Type::NV12);
            queues.push_back(replay->out.createOutputQueue(8, true));
        }
        pipeline.start();
        for(const auto& queue : queues) receive(queue, std::string(extension) == ".dai");
        const auto info = device->getReplayCacheInfo();
        REQUIRE(std::count_if(info.entries.begin(),
                              info.entries.end(),
                              [&](const auto& entry) { return entry.kind == dai::ReplayCacheKind::SOURCE && entry.sourceHash == hash; })
                == 1);
        const auto entry = std::find_if(info.entries.begin(), info.entries.end(), [&](const auto& value) {
            return value.kind == dai::ReplayCacheKind::SOURCE && value.sourceHash == hash;
        });
        REQUIRE(entry != info.entries.end());
        REQUIRE(entry->sizeBytes == bytes);
        REQUIRE(entry->inUse);
        device->clearReplayCache();
        REQUIRE(std::get<0>(device->replayCacheQuery(hash)));
        pipeline.stop();
    }
    {
        auto device = openRvc4();
        REQUIRE(device);
        const auto present = device->replayCacheQuery(hash);
        REQUIRE(std::get<0>(present));
        REQUIRE(std::get<2>(present) == bytes);
        dai::Pipeline pipeline(device);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setSource(frames).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setFps(30).setDecodeMode(mode);
        replay->setOutFrameType(dai::ImgFrame::Type::NV12);
        auto queue = replay->out.createOutputQueue(8, true);
        pipeline.start();
        receive(queue, std::string(extension) == ".dai");
        pipeline.stop();
    }
    std::ofstream(frames / "readme.txt", std::ios::app) << "changed";
    const auto changed = dai::replay::sha256OfDirectory(frames);
    REQUIRE(changed != hash);
    {
        auto device = openRvc4();
        REQUIRE(device);
        dai::Pipeline pipeline(device);
        std::vector<std::shared_ptr<dai::MessageQueue>> queues;
        for(int index = 0; index < 2; ++index) {
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setSource(frames).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setFps(30).setDecodeMode(mode);
            replay->setOutFrameType(dai::ImgFrame::Type::NV12);
            queues.push_back(replay->out.createOutputQueue(8, true));
        }
        pipeline.start();
        for(const auto& queue : queues) receive(queue, std::string(extension) == ".dai");
        pipeline.stop();
    }
    auto device = openRvc4();
    REQUIRE(device);
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(changed)));
    REQUIRE(std::get<0>(device->replayCacheQuery(hash)));
    device->clearReplayCache();
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
}

TEST_CASE("RVC4 directory uploads reject invalid requests and discard partial data", "[directory][cache][rvc4]") {
    using Catch::Matchers::ContainsSubstring;
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory source;
    std::ofstream(source.path / "a", std::ios::binary) << "abc";
    std::ofstream(source.path / "b", std::ios::binary) << "de";
    const auto nonce = "nonce_" + source.path.filename().string();
    std::ofstream(source.path / nonce).close();
    const auto hash = dai::replay::sha256OfDirectory(source.path);
    AbortUpload abort{device, hash};
    const std::map<std::string, std::uint64_t> files{{"a", 3}, {"b", 2}, {nonce, 0}};
    REQUIRE_THROWS(device->replayCacheBeginDirectory(hash, {{"../outside", 1}}));
    REQUIRE_THROWS(device->replayCacheBeginDirectory(hash, {{"/absolute", 1}}));
    REQUIRE_THROWS(device->replayCacheBeginDirectory(hash, {{"nested/file", 1}}));
    REQUIRE_THROWS(device->replayCacheBeginDirectory(hash, {{".hidden", 1}}));
    const auto limit = device->getReplayCacheInfo().limitBytes;
    RestoreLimit restore{device, limit};
    const auto before = device->getReplayCacheInfo();
    device->setReplayCacheLimit(4);
    REQUIRE_THROWS(device->replayCacheBeginDirectory(hash, files));
    REQUIRE(device->getReplayCacheInfo().entries.size() == before.entries.size());
    device->setReplayCacheLimit(limit);
    device->replayCacheBeginDirectory(hash, files);
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
    REQUIRE_THROWS(device->replayCacheWriteFile(hash, "undeclared", 0, {1}));
    REQUIRE_THROWS(device->replayCacheWriteFile(hash, "a", 1, {'b'}));
    REQUIRE_THROWS(device->replayCacheWriteFile(hash, "a", 0, {'a', 'b', 'c', 'd'}));
    device->replayCacheWriteFile(hash, "a", 0, {'a', 'b', 'c'});
    REQUIRE_THROWS_WITH(device->replayCacheCommit(hash), ContainsSubstring("incomplete"));
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
    device->replayCacheBeginDirectory(hash, files);
    device->replayCacheWriteFile(hash, "a", 0, {'x', 'y', 'z'});
    device->replayCacheWriteFile(hash, "b", 0, {'d', 'e'});
    REQUIRE_THROWS_WITH(device->replayCacheCommit(hash), ContainsSubstring("hash mismatch"));
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
    device->replayCacheBeginDirectory(hash, files);
    device->replayCacheAbort(hash);
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
    device->replayCacheBeginDirectory(hash, files);
    device->replayCacheWriteFile(hash, "a", 0, {'a', 'b'});
    device->replayCacheWriteFile(hash, "a", 2, {'c'});
    device->replayCacheWriteFile(hash, "b", 0, {'d', 'e'});
    device->replayCacheCommit(hash);
    const auto committed = device->replayCacheQuery(hash);
    REQUIRE(std::get<0>(committed));
    REQUIRE(std::get<2>(committed) == 5);
    device->replayCacheRelease({hash});
    device->clearReplayCache();
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
}

TEST_CASE("RVC4 cache reserves pending uploads against its quota", "[directory][cache][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    RestoreLimit restore{device, device->getReplayCacheInfo().limitBytes};
    const auto nonce = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto first = hashText(nonce);
    const auto other = nonce + "second";
    const auto second = hashText(other);
    AbortUpload abortFirst{device, first};
    AbortUpload abortSecond{device, second};
    device->setReplayCacheLimit(10);
    REQUIRE_FALSE(std::get<0>(device->replayCacheBeginDirectory(first, {{"a", 8}})));
    REQUIRE_THROWS(device->replayCacheBeginDirectory(second, {{"b", 8}}));
    device->replayCacheAbort(first);
    REQUIRE_FALSE(std::get<0>(device->replayCacheBeginDirectory(second, {{"b", 8}})));
    device->replayCacheAbort(second);
    device->setReplayCacheLimit(200);
    const std::vector<std::uint8_t> data(nonce.begin(), nonce.end());
    const auto fileHash = dai::replay::sha256OfBytes(data.data(), data.size());
    AbortUpload abortFile{device, fileHash};
    REQUIRE_FALSE(std::get<0>(device->replayCachePrepareFile(fileHash, data.size())));
    device->replayCacheWrite(fileHash, 0, data);
    device->replayCacheCommit(fileHash);
    REQUIRE_THROWS(device->replayCacheBeginDirectory(first, {{"a", 200}}));
    REQUIRE(std::get<0>(device->replayCacheQuery(fileHash)));
    device->replayCacheRelease({fileHash});
    device->clearReplayCache();
}

TEST_CASE("RVC4 keeps prepared directory sources under quota pressure", "[directory][cache][rvc4]") {
    replaytest::Directory first;
    replaytest::Directory second;
    writeImages(first.path, ".png");
    writeImages(second.path, ".png");
    std::ofstream(second.path / "extra") << "different source";
    const auto firstBytes = directoryBytes(first.path);
    const auto secondBytes = directoryBytes(second.path);
    const auto hash = dai::replay::sha256OfDirectory(first.path);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    RestoreLimit restore{device, device->getReplayCacheInfo().limitBytes};
    device->setReplayCacheLimit(std::max(firstBytes, secondBytes));
    {
        dai::Pipeline pipeline(device);
        std::vector<std::shared_ptr<dai::MessageQueue>> queues;
        for(const auto& path : {first.path, second.path}) {
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setSource(path).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false);
            queues.push_back(replay->out.createOutputQueue(8, true));
        }
        REQUIRE_THROWS(pipeline.start());
    }
    device->close();
    device = openRvc4();
    REQUIRE(device);
    REQUIRE(std::get<0>(device->replayCacheQuery(hash)));
    const auto info = device->getReplayCacheInfo();
    const auto found = std::find_if(info.entries.begin(), info.entries.end(), [&](const auto& entry) { return entry.sourceHash == hash; });
    REQUIRE(found != info.entries.end());
    REQUIRE_FALSE(found->inUse);
    device->clearReplayCache();
}

TEST_CASE("RVC4 reconnect discards an interrupted directory upload", "[directory][cache][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    const auto nonce = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto hash = hashText(nonce);
    REQUIRE_FALSE(std::get<0>(device->replayCacheBeginDirectory(hash, {{"frame", 128}})));
    device->replayCacheWriteFile(hash, "frame", 0, {1, 2, 3});
    device->close();
    device = openRvc4();
    REQUIRE(device);
    AbortUpload abort{device, hash};
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
    REQUIRE_FALSE(std::get<0>(device->replayCacheBeginDirectory(hash, {{"frame", 128}})));
}

TEST_CASE("RVC4 directory replay uses automatic and explicit metadata sidecars", "[directory][metadata][rvc4]") {
    const bool explicitMetadata = GENERATE(false, true);
    const bool persistent = GENERATE(false, true);
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    replaytest::Directory source;
    const auto frames = source.path / "frames";
    std::filesystem::create_directory(frames);
    writeImages(frames, ".png");
    const auto writeMetadata = [](const std::filesystem::path& path, int firstSequence) {
        dai::utility::ByteRecorder recorder;
        recorder.init<dai::proto::img_frame::ImgFrame>(path.string(), dai::RecordConfig::CompressionLevel::NONE, "video");
        for(int index = 0; index < 3; ++index) {
            dai::ImgFrame frame;
            frame.setType(dai::ImgFrame::Type::NV12);
            frame.setSize(18, 10);
            frame.setSequenceNum(firstSequence + index);
            const auto time = std::chrono::steady_clock::time_point(std::chrono::milliseconds(1000 + index * 100));
            frame.setTimestamp(time);
            frame.setTimestampDevice(time);
            recorder.write(frame.serializeProto(true));
        }
        recorder.close();
    };
    writeMetadata(frames / "cam.mcap", 100);
    const auto external = source.path / "external.mcap";
    if(explicitMetadata) writeMetadata(external, 200);
    {
        auto device = openRvc4();
        if(!device) SKIP("No RVC4 connected");
        dai::Pipeline pipeline(device);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setSource(frames).setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setDecodeMode(mode);
        replay->setPersistentUpload(persistent);
        if(explicitMetadata) replay->setMetadataFile(external);
        auto queue = replay->out.createOutputQueue(8, true);
        pipeline.start();
        for(int index = 0; index < 3; ++index) {
            bool timeout = false;
            const auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(5), timeout);
            REQUIRE_FALSE(timeout);
            REQUIRE(frame);
            REQUIRE(frame->getSequenceNum() == (explicitMetadata ? 200 : 100) + index);
            REQUIRE(frame->getTimestampDevice() == std::chrono::steady_clock::time_point(std::chrono::milliseconds(1000 + index * 100)));
        }
        pipeline.stop();
    }
    auto device = openRvc4();
    REQUIRE(device);
    REQUIRE(std::get<0>(device->replayCacheQuery(dai::replay::sha256OfDirectory(frames))) == persistent);
    if(explicitMetadata) REQUIRE(std::get<0>(device->replayCacheQuery(dai::replay::sha256OfFile(external))) == persistent);
    device->clearReplayCache();
}
