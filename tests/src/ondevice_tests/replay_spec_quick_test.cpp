#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <atomic>
#include <chrono>
#include <fstream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "../ReplayArchiveFixture.hpp"
#include "depthai/depthai.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "opencv/ReplayInternal.hpp"
#include "utility/RecordReplayImpl.hpp"

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;

namespace {
std::shared_ptr<dai::Device> openRvc4() {
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(info.platform == X_LINK_RVC4) return std::make_shared<dai::Device>(info);
    }
    return nullptr;
}

void image(const std::filesystem::path& path, int value) {
    REQUIRE(cv::imwrite(path.string(), cv::Mat(16, 28, CV_8UC3, cv::Scalar(value, value, value))));
}

void metadata(const std::filesystem::path& path, int count) {
    dai::utility::ByteRecorder recorder;
    recorder.init<dai::proto::img_frame::ImgFrame>(path.string(), dai::RecordConfig::CompressionLevel::NONE, "video");
    for(int index = 0; index < count; ++index) {
        dai::ImgFrame frame;
        frame.setType(dai::ImgFrame::Type::NV12).setSize(28, 16);
        frame.setSequenceNum(10 + index * 10);
        const auto time = std::chrono::steady_clock::time_point(index * 100ms);
        frame.setTimestamp(time);
        frame.setTimestampDevice(time);
        frame.sourceFb.width = 1234;
        frame.sourceFb.height = 678;
        frame.cam.sensitivityIso = 432;
        frame.instanceNum = 42;
        frame.category = 7;
        frame.transformation = dai::ImgTransformation(28, 16);
        frame.setData(std::vector<std::uint8_t>(28 * 16 * 3 / 2, 100));
        recorder.write(frame.serializeProto());
    }
    recorder.close();
}

std::pair<std::string, std::string> upload(dai::Device& device, const std::filesystem::path& directory) {
    const auto hash = dai::replay::sha256OfDirectory(directory);
    std::map<std::string, std::uint64_t> files;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) files.emplace(entry.path().filename().string(), entry.file_size());
    const auto prepared = device.replayCacheBeginDirectory(hash, files);
    if(!std::get<0>(prepared)) {
        for(const auto& file : files) {
            std::ifstream input(directory / file.first, std::ios::binary);
            const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
            device.replayCacheWriteFile(hash, file.first, 0, bytes);
        }
        device.replayCacheCommit(hash);
    }
    return {hash, std::get<1>(prepared)};
}
}  // namespace

TEST_CASE("RVC4 quick fixes preserve metadata and strict numeric order", "[spec-quick][rvc4]") {
    const auto mode = GENERATE(dai::ReplayDecodeMode::STREAM, dai::ReplayDecodeMode::PRE_DECODE);
    replaytest::Directory files;
    image(files.path / "cam_2.png", 40);
    image(files.path / "cam_2147483648.png", 180);
    image(files.path / "cam_3abc.png", 100);
    metadata(files.path / "cam.mcap", 2);
    const auto hash = dai::replay::sha256OfDirectory(files.path);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    {
        dai::Pipeline pipeline(device);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setSource(files.path).setRunOnHost(false).setSize(14, 8).setOutFrameType(dai::ImgFrame::Type::NV12);
        replay->setDecodeMode(mode).setFps(999.95f).setLoop(true).setStopPipelineAtEnd(false);
        replay->setPersistentUpload(true).setPersistentUpload(false);
        auto queue = replay->out.createOutputQueue(8, true);
        pipeline.start();
        std::chrono::steady_clock::time_point previous;
        for(int index = 0; index < 3; ++index) {
            bool timeout = false;
            auto frame = queue->get<dai::ImgFrame>(3s, timeout);
            REQUIRE_FALSE(timeout);
            REQUIRE(frame);
            REQUIRE(frame->getWidth() == 14);
            REQUIRE(frame->getHeight() == 8);
            REQUIRE(frame->sourceFb.width == 1234);
            REQUIRE(frame->sourceFb.height == 678);
            REQUIRE(frame->cam.sensitivityIso == 432);
            REQUIRE(frame->instanceNum == 42);
            REQUIRE(frame->category == 7);
            REQUIRE(frame->transformation.getSize().first == 14);
            REQUIRE(frame->transformation.getSize().second == 8);
            REQUIRE(frame->getSequenceNum() == (index == 0 ? 10 : index == 1 ? 20 : 21));
            if(index == 1) REQUIRE(frame->getData()[frame->fb.p1Offset] > 140);
            if(index == 2) {
                const auto gap = std::chrono::duration<double>(frame->getTimestampDevice() - previous).count();
                REQUIRE(gap == Catch::Approx(1.0 / 999.95).margin(0.00001));
            }
            previous = frame->getTimestampDevice();
        }
        pipeline.stop();
    }
    device = openRvc4();
    REQUIRE(device);
    REQUIRE_FALSE(std::get<0>(device->replayCacheQuery(hash)));
}

TEST_CASE("RVC4 resize without metadata keeps an output-sized identity", "[spec-quick][rvc4]") {
    replaytest::Directory files;
    image(files.path / "cam_0.png", 80);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(files.path).setRunOnHost(false).setSize(14, 8).setLoop(false).setStopPipelineAtEnd(false);
    auto queue = replay->out.createOutputQueue();
    pipeline.start();
    bool timeout = false;
    auto frame = queue->get<dai::ImgFrame>(3s, timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(frame);
    REQUIRE(frame->transformation.getSize().first == 14);
    REQUIRE(frame->transformation.getSize().second == 8);
    pipeline.stop();
}

TEST_CASE("RVC4 rejects missing explicit metadata and names every stream", "[spec-quick][rvc4]") {
    const bool missingMetadata = GENERATE(false, true);
    replaytest::Directory files;
    image(files.path / "left_0.png", 80);
    if(!missingMetadata) image(files.path / "right_0.png", 140);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    const auto source = upload(*device, files.path);
    {
        dai::Pipeline pipeline(device);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setSource(source.second, dai::ReplaySourceLocation::DEVICE).setRunOnHost(false).setSize(28, 16);
        if(missingMetadata) replay->setMetadataFile("/tmp/absent-replay-spec-metadata.mcap");
        auto queue = replay->out.createOutputQueue();
        if(missingMetadata) REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("metadata file is missing"));
        else REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("left") && ContainsSubstring("right"));
    }
    if(!device->isClosed()) {
        device->replayCacheRelease({source.first});
        device->clearReplayCache();
    }
}

TEST_CASE("RVC4 metadata archive selection ignores a same-named directory", "[spec-quick][rvc4]") {
    replaytest::Directory files;
    const auto mcap = files.path / "cam.mcap";
    metadata(mcap, 2);
    std::ifstream input(mcap, std::ios::binary);
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
    const auto archive = files.path / "recording.tar";
    replaytest::writeArchive(archive, {{"cam/", {}, true}, {"cam.mcap", bytes}});
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
    replay->setSource(archive).setArchiveEntry("cam").setRunOnHost(false).setLoop(false).setStopPipelineAtEnd(false).setFps(30);
    auto queue = replay->out.createOutputQueue();
    pipeline.start();
    for(int index = 0; index < 2; ++index) {
        bool timeout = false;
        auto frame = queue->get<dai::ImgFrame>(3s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
        REQUIRE(frame->getSequenceNum() == 10 + index * 10);
    }
    pipeline.stop();
}

TEST_CASE("RVC4 duration limit retains frames received before START", "[spec-quick][rvc4]") {
    replaytest::Directory files;
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    dai::Pipeline pipeline(device);
    auto record = pipeline.create<dai::node::RecordVideo>();
    const auto name = "duration_" + dai::makeRecordingTimestamp();
    record->setRunOnHost(false).setFormat(dai::RecordFormat::RAW_FRAMES).setRecordingName(name).setOutputDirectory(files.path);
    record->setWaitForTrigger(true).setPreTriggerFrames(15);
    auto input = record->input.createInputQueue();
    auto controls = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, false);
    pipeline.start();
    for(int index = 0; index < 15; ++index) {
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setType(dai::ImgFrame::Type::NV12).setSize(64, 48).setSequenceNum(index);
        frame->setData(std::vector<std::uint8_t>(64 * 48 * 3 / 2, 100));
        input->send(frame);
        std::this_thread::sleep_for(15ms);
    }
    std::this_thread::sleep_for(200ms);
    controls->send(dai::RecordControl::start(-1, 1));
    std::shared_ptr<dai::RecordStatus> stopped;
    for(int index = 0; index < 2; ++index) {
        bool timeout = false;
        auto message = status->get<dai::RecordStatus>(5s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(message);
        if(message->state == dai::RecordStatus::State::STOPPED) stopped = message;
    }
    REQUIRE(stopped);
    REQUIRE(stopped->frameCount == 15);
    REQUIRE(stopped->firstSequenceNum == 0);
    REQUIRE(stopped->lastSequenceNum == 14);
    pipeline.stop();
    const auto recorded = files.path / name / "frames";
    REQUIRE(std::distance(std::filesystem::directory_iterator(recorded), std::filesystem::directory_iterator{}) == 15);
}

TEST_CASE("RVC4 predecode progress includes the final frame for odd totals", "[spec-quick][rvc4]") {
    replaytest::Directory files;
    for(int index = 0; index < 25; ++index) image(files.path / ("cam_" + std::to_string(index) + ".png"), 80);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    std::atomic<bool> finalProgress{false};
    const auto callback = device->addLogCallback([&](dai::LogMessage message) {
        if(message.payload.find("Replay decode ") != std::string::npos && message.payload.find("25/25 frames") != std::string::npos) finalProgress = true;
    });
    device->setLogLevel(dai::LogLevel::INFO);
    dai::Pipeline pipeline(device);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(files.path).setRunOnHost(false).setDecodeMode(dai::ReplayDecodeMode::PRE_DECODE).setLoop(false).setStopPipelineAtEnd(false);
    replay->setLogLevel(dai::LogLevel::INFO);
    auto queue = replay->out.createOutputQueue(32, false);
    pipeline.start();
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while(!finalProgress && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(5ms);
    pipeline.stop();
    device->removeLogCallback(callback);
    REQUIRE(finalProgress.load());
}

TEST_CASE("RVC4 does not duplicate the host's missing-trigger warning", "[spec-quick][rvc4]") {
    replaytest::Directory files;
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    std::atomic<int> deviceWarnings{0};
    const auto callback = device->addLogCallback([&](dai::LogMessage message) {
        if(message.payload.find("Record no trigger source") != std::string::npos) ++deviceWarnings;
    });
    dai::Pipeline pipeline(device);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(false).setFormat(dai::RecordFormat::RAW_FRAMES).setRecordingName("warning_" + dai::makeRecordingTimestamp());
    record->setOutputDirectory(files.path).setWaitForTrigger(true).setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    std::this_thread::sleep_for(100ms);
    pipeline.stop();
    device->removeLogCallback(callback);
    REQUIRE(deviceWarnings.load() == 0);
}
