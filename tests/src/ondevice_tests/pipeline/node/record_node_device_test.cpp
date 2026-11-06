extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <fstream>
#include <regex>
#include <unistd.h>

#include "../../../helpers/replay_test_support.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MessageDemux.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/utility/Compression.hpp"
#include "nlohmann/json.hpp"

using record_test::DeviceLogCapture;
using record_test::EnvGuard;
using record_test::LineLevel;
using record_test::LogCapture;
using record_test::RecordingGuard;
using record_test::TempDir;
using record_test::allFiles;
using record_test::deleteRecording;
using record_test::downloadRecording;
using record_test::fileCount;
using record_test::fileSizeSum;
using record_test::getStatus;
using record_test::listRecordings;
using record_test::loadImg;
using record_test::makeImg;
using record_test::openRvc2OrSkip;
using record_test::openRvc4OrSkip;
using record_test::sendFrame;
using record_test::isFrameFile;
using record_test::setFormat;
using record_test::setFps;
using record_test::setKeepOnDevice;
using record_test::setMaxPreTriggerBytes;
using record_test::setPreTriggerFrames;
using record_test::setOutputDirectory;
using record_test::setRecordMetadataFile;
using record_test::setRecordVideoFile;
using record_test::setRecordingName;
using record_test::setRunOnHost;
using record_test::setStreamName;
using record_test::setTransferToHost;
using record_test::setWaitForTrigger;
using record_test::startControl;
using record_test::stopControl;
using record_test::uniqueName;
using record_test::waitUntil;
using replay_test::makeDaiDir;
using replay_test::rangeSeq;
using replay_test::rangeTs;
using replay_test::setFast;
using replay_test::setLoop;
using replay_test::setMetadataFile;
using replay_test::setSource;
using replay_test::setStopPipelineAtEnd;
using Catch::Matchers::ContainsSubstring;

namespace {

struct Store {
    std::shared_ptr<dai::Device> device;
    EnvGuard unsetReplay;
    EnvGuard unsetRecord;
    TempDir dir;
    DeviceLogCapture devLog;
    LogCapture hostLog;
    std::unique_ptr<dai::Pipeline> pipeline;
    std::unique_ptr<RecordingGuard> guard;

    Store()
        : device(openRvc4OrSkip()),
          unsetReplay("DEPTHAI_REPLAY"),
          unsetRecord("DEPTHAI_RECORD"),
          devLog(*device, dai::LogLevel::DEBUG) {
        try {
            deleteRecording(*device, "test");
        } catch(const std::exception&) {
        }
    }

    dai::Pipeline& begin() {
        pipeline = std::make_unique<dai::Pipeline>(device);
        guard = std::make_unique<RecordingGuard>(*device, *pipeline);
        return *pipeline;
    }

    void restart() {
        if(pipeline && pipeline->isRunning()) pipeline->stop();
        guard.reset();
        pipeline.reset();
        begin();
    }

    dai::Pipeline& replacePipeline() {
        if(pipeline && pipeline->isRunning()) pipeline->stop();
        pipeline = std::make_unique<dai::Pipeline>(device);
        if(guard) {
            guard->retarget(*pipeline);
        } else {
            guard = std::make_unique<RecordingGuard>(*device, *pipeline);
        }
        return *pipeline;
    }
};

struct RawNode {
    std::shared_ptr<dai::node::RecordVideo> node;
    std::shared_ptr<dai::InputQueue> frames;
    std::shared_ptr<dai::MessageQueue> status;
};

RawNode addRaw(dai::Pipeline& pipeline, const std::filesystem::path& dir, const std::string& name, bool setTransfer, bool transfer, const std::string& stream = "cam") {
    RawNode raw;
    raw.node = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*raw.node);
    setFormat(*raw.node, record_test::Format::RAW_FRAMES);
    setRecordingName(*raw.node, name);
    setOutputDirectory(*raw.node, dir);
    setStreamName(*raw.node, stream);
    if(setTransfer) setTransferToHost(*raw.node, transfer);
    raw.frames = raw.node->input.createInputQueue();
    raw.status = raw.node->status.createOutputQueue(16, true);
    return raw;
}

const record_test::RecordingView* findRecording(const std::vector<record_test::RecordingView>& rows, const std::string& name) {
    for(const auto& row : rows) {
        if(row.name == name) return &row;
    }
    return nullptr;
}

void sendIdle(const std::shared_ptr<dai::InputQueue>& queue, DeviceLogCapture& log, int index) {
    sendFrame(queue, makeImg(64, 48, index, index));
    REQUIRE(log.waitFor("Record idle frame " + std::to_string(index), std::chrono::seconds(10)));
}

void sendWrote(const std::shared_ptr<dai::InputQueue>& queue, DeviceLogCapture& log, int index) {
    sendFrame(queue, makeImg(64, 48, index, index));
    REQUIRE(log.waitFor("Record wrote frame " + std::to_string(index), std::chrono::seconds(10)));
}

std::shared_ptr<dai::RecordStatus> takeStatus(const std::shared_ptr<dai::MessageQueue>& queue) {
    auto status = record_test::getFor<dai::RecordStatus>(*queue, std::chrono::seconds(15));
    REQUIRE(status != nullptr);
    return status;
}

void checkStopped(const dai::RecordStatus& status, std::int32_t segment, std::int64_t count, std::int64_t first, std::int64_t last, std::int64_t skipped) {
    CHECK(status.state == dai::RecordStatus::State::STOPPED);
    CHECK(status.segment == segment);
    CHECK(status.frameCount == count);
    CHECK(status.firstSequenceNum == first);
    CHECK(status.lastSequenceNum == last);
    CHECK(status.skippedFrames == skipped);
}

std::uint64_t streamFileBytes(const std::filesystem::path& dir, const std::string& stream = "cam") {
    std::uint64_t sum = 0;
    for(const auto& name : allFiles(dir)) {
        if(!isFrameFile(name, stream)) continue;
        sum += static_cast<std::uint64_t>(std::filesystem::file_size(dir / name));
    }
    return sum;
}

bool named(const std::vector<record_test::RecordingView>& rows, const std::string& name) {
    return findRecording(rows, name) != nullptr;
}

bool holisticNamed(const std::vector<record_test::RecordingView>& rows) {
    for(const auto& row : rows) {
        if(row.name.rfind("holistic_", 0) == 0) return true;
    }
    return false;
}

int warnHits(const Store& store, const std::string& text) {
    return store.devLog.count(LineLevel::WARN, text) + store.hostLog.count(LineLevel::WARN, text);
}

bool downloadFinished(const LogCapture& log) {
    std::string last;
    for(const auto& text : log.texts()) {
        if(text.find("Record download") != std::string::npos) last = text;
    }
    if(last.empty()) return false;
    std::smatch match;
    std::regex numbers("(\\d+)/(\\d+)");
    if(!std::regex_search(last, match, numbers)) return false;
    return match[1].str() == match[2].str();
}

void expectFiles(const std::filesystem::path& dir, const std::vector<std::string>& names) {
    CHECK(allFiles(dir) == names);
}

void skipUnlessThree(dai::Device& device) {
    if(device.getConnectedCameras().size() < 3) SKIP("fewer than three cameras");
}

struct CamOut {
    std::shared_ptr<dai::node::Camera> camera;
    dai::Node::Output* output = nullptr;
    std::shared_ptr<dai::MessageQueue> queue;
};

std::vector<CamOut> addCameras(dai::Pipeline& pipeline, std::uint32_t width, std::uint32_t height, float fps, unsigned queueSize, bool blocking) {
    std::vector<CamOut> cameras;
    for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        CamOut cam;
        cam.camera = pipeline.create<dai::node::Camera>()->build(socket);
        cam.output = cam.camera->requestOutput({width, height}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, fps);
        cam.queue = cam.output->createOutputQueue(queueSize, blocking);
        cameras.push_back(std::move(cam));
    }
    return cameras;
}

std::filesystem::path oneTar(const std::filesystem::path& out) {
    const std::regex name("^recording_\\d{4}-\\d{2}-\\d{2}_\\d{2}-\\d{2}-\\d{2}\\.tar\\.gz$");
    std::filesystem::path found;
    for(const auto& entry : std::filesystem::directory_iterator(out)) {
        if(!entry.is_regular_file()) continue;
        if(!std::regex_match(entry.path().filename().string(), name)) continue;
        REQUIRE(found.empty());
        found = entry.path();
    }
    REQUIRE(!found.empty());
    return found;
}

void checkGzip(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    unsigned char magic[2] = {0, 0};
    input.read(reinterpret_cast<char*>(magic), 2);
    CHECK(magic[0] == 0x1f);
    CHECK(magic[1] == 0x8b);
}

int countPrefixed(const std::vector<std::string>& names, const std::string& prefix) {
    int count = 0;
    for(const auto& name : names) {
        if(name.rfind(prefix, 0) != 0 || !record_test::endsWith(name, ".dai")) continue;
        const auto digits = name.substr(prefix.size(), name.size() - prefix.size() - 4);
        if(digits.size() != 8) continue;
        bool numeric = true;
        for(const char ch : digits) numeric = numeric && ch >= '0' && ch <= '9';
        if(numeric) ++count;
    }
    return count;
}

void writeRecordJson(const std::filesystem::path& path, const std::filesystem::path& out, const char* format, bool keep) {
    nlohmann::json body = {{"outputDir", out.string()},
                           {"videoEncoding", {{"enabled", true}, {"bitrate", 0}, {"profile", "H264_MAIN"}, {"lossless", false}, {"quality", 80}}},
                           {"compressionLevel", 3},
                           {"syncCameraOutputs", true},
                           {"keepOnDevice", keep}};
    if(format != nullptr) body["format"] = format;
    std::ofstream file(path);
    file << body.dump();
}

struct DirMode {
    std::filesystem::path path;
    std::filesystem::perms saved;
    explicit DirMode(std::filesystem::path directory) : path(std::move(directory)), saved(std::filesystem::status(path).permissions()) {}
    ~DirMode() {
        std::error_code error;
        std::filesystem::permissions(path, saved, error);
    }
    DirMode(const DirMode&) = delete;
    DirMode& operator=(const DirMode&) = delete;
};

std::filesystem::path recordFive(Store& store, const std::string& name, bool setTransfer, bool transfer) {
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, setTransfer, transfer);
    pipeline.start();
    auto started = takeStatus(raw.status);
    CHECK(started->state == dai::RecordStatus::State::STARTED);
    CHECK(started->segment == 0);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    pipeline.stop();
    auto stopped = getStatus(*raw.node);
    REQUIRE(stopped != nullptr);
    checkStopped(*stopped, 0, 5, 0, 4, 0);
    CHECK(record_test::endsWith(stopped->path, "/" + name));
    return std::filesystem::path(stopped->path);
}

}  // namespace

TEST_CASE("VR1 five raw frames stay on the device", "[record][device]") {
    Store store;
    const auto path = recordFive(store, "test", true, false);
    downloadRecording(*store.device, "test", store.dir.path);
    const auto stream = store.dir.path / "test" / "cam";
    CHECK(fileCount(stream) == 5);
    const auto listed = listRecordings(*store.device);
    const auto* info = findRecording(listed, "test");
    REQUIRE(info != nullptr);
    CHECK(info->complete);
    CHECK(info->sizeBytes == streamFileBytes(stream));
    CHECK(path.filename() == "test");
}

TEST_CASE("VR2 downloaded frames match the sent rows", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR2");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    std::vector<std::vector<std::uint8_t>> sent;
    for(int index = 0; index < 5; ++index) {
        auto frame = makeImg(64, 48, index, index);
        auto data = frame->getData();
        sent.emplace_back(data.begin(), data.end());
        sendFrame(raw.frames, frame);
        REQUIRE(store.devLog.waitFor("Record wrote frame " + std::to_string(index), std::chrono::seconds(10)));
    }
    pipeline.stop();
    downloadRecording(*store.device, name, store.dir.path);
    for(int index = 0; index < 5; ++index) {
        auto loaded = loadImg(store.dir.path / name / "cam" / record_test::frameFileName("cam", index));
        REQUIRE(loaded);
        CHECK(loaded->fb.width == 64);
        CHECK(loaded->fb.height == 48);
        CHECK(loaded->getType() == dai::ImgFrame::Type::NV12);
        CHECK(loaded->fb.stride >= 64);
        CHECK(loaded->getSequenceNum() == index);
        const auto got = loaded->getData();
        REQUIRE(got.size() == sent[static_cast<std::size_t>(index)].size());
        CHECK(std::equal(got.begin(), got.end(), sent[static_cast<std::size_t>(index)].begin()));
    }
}

#include "record_node_device_catalog.inc"
