extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <regex>

#include "../../../helpers/record_test_support.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"

using record_test::DeviceLogCapture;
using record_test::EnvGuard;
using record_test::LineLevel;
using record_test::LogCapture;
using record_test::RecordingGuard;
using record_test::TempDir;
using record_test::getStatus;
using record_test::openRvc4OrSkip;
using record_test::setFormat;
using record_test::setFps;
using record_test::setOutputDirectory;
using record_test::setPreTriggerFrames;
using record_test::setRecordingName;
using record_test::setStreamName;
using record_test::setTransferToHost;
using record_test::setWaitForTrigger;
using record_test::startControl;
using record_test::uniqueName;

namespace {

struct Perf {
    std::shared_ptr<dai::Device> device;
    EnvGuard unsetReplay;
    EnvGuard unsetRecord;
    TempDir dir;
    DeviceLogCapture devLog;
    LogCapture hostLog;
    std::unique_ptr<dai::Pipeline> pipeline;
    std::unique_ptr<RecordingGuard> guard;

    Perf()
        : device(openRvc4OrSkip()),
          unsetReplay("DEPTHAI_REPLAY"),
          unsetRecord("DEPTHAI_RECORD"),
          devLog(*device, dai::LogLevel::DEBUG) {}

    dai::Pipeline& begin() {
        pipeline = std::make_unique<dai::Pipeline>(device);
        guard = std::make_unique<RecordingGuard>(*device, *pipeline);
        return *pipeline;
    }
};

std::shared_ptr<dai::RecordStatus> takeStatus(const std::shared_ptr<dai::MessageQueue>& queue, std::chrono::milliseconds timeout) {
    auto status = record_test::getFor<dai::RecordStatus>(*queue, timeout);
    REQUIRE(status != nullptr);
    return status;
}

int rateInWarning(const std::string& text) {
    std::smatch match;
    std::regex pattern("Record expected write rate (\\d+) MB/s");
    if(!std::regex_search(text, match, pattern)) return -1;
    return std::stoi(match[1].str());
}

}  // namespace

TEST_CASE("VD1 raw 1080p keeps 600 frames", "[record][perf]") {
    Perf perf;
    auto& pipeline = perf.begin();
    const auto name = uniqueName("VD1");
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::RAW_FRAMES);
    setRecordingName(*record, name);
    setOutputDirectory(*record, perf.dir.path);
    setStreamName(*record, "cam");
    setTransferToHost(*record, true);
    setWaitForTrigger(*record, true);
    setPreTriggerFrames(*record, 0);
    output->link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    control->send(startControl(600, -1));
    auto started = takeStatus(status, std::chrono::seconds(15));
    auto stopped = takeStatus(status, std::chrono::seconds(60));
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    CHECK(stopped->segment == 0);
    CHECK(stopped->frameCount == 600);
    CHECK(stopped->skippedFrames == 0);
    CHECK(stopped->lastSequenceNum == stopped->firstSequenceNum + 599);
    const auto startedMs = record_test::timestampMs(started->getTimestamp());
    const auto stoppedMs = record_test::timestampMs(stopped->getTimestamp());
    const auto bytes = record_test::fileSizeSum(perf.dir.path / name);
    if(stoppedMs > startedMs && bytes > 0) {
        const double rate = static_cast<double>(bytes) / (static_cast<double>(stoppedMs - startedMs) / 1000.0);
        INFO("VD1 write bytes per second " << rate);
    }
}

TEST_CASE("VD2 encoded 4k keeps 600 frames", "[record][perf]") {
    Perf perf;
    auto& pipeline = perf.begin();
    const auto name = uniqueName("VD2");
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({3840, 2160}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(30.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    output->link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::VIDEO);
    setRecordingName(*record, name);
    setOutputDirectory(*record, perf.dir.path);
    setStreamName(*record, "enc");
    setFps(*record, 30u);
    setTransferToHost(*record, true);
    setWaitForTrigger(*record, true);
    setPreTriggerFrames(*record, 0);
    encoder->out.link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    control->send(startControl(600, -1));
    REQUIRE(takeStatus(status, std::chrono::seconds(15))->state == dai::RecordStatus::State::STARTED);
    auto stopped = takeStatus(status, std::chrono::seconds(60));
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    CHECK(stopped->frameCount == 600);
    CHECK(stopped->skippedFrames == 0);
}

TEST_CASE("VD3 a 12 megapixel frame warns at 540 MB/s or more", "[record][perf]") {
    Perf perf;
    auto& pipeline = perf.begin();
    const auto name = uniqueName("VD3");
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({4000, 3000}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::RAW_FRAMES);
    setRecordingName(*record, name);
    setOutputDirectory(*record, perf.dir.path);
    setStreamName(*record, "cam");
    setTransferToHost(*record, true);
    setWaitForTrigger(*record, true);
    setPreTriggerFrames(*record, 0);
    output->link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    control->send(startControl(-1, 20000));
    auto stopped = takeStatus(status, std::chrono::seconds(60));
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    int warnings = 0;
    int rate = -1;
    const auto harvest = [&](const std::vector<std::string>& lines) {
        for(const auto& text : lines) {
            const int parsed = rateInWarning(text);
            if(parsed < 0) continue;
            ++warnings;
            rate = parsed;
        }
    };
    harvest(perf.devLog.texts());
    if(warnings == 0) harvest(perf.hostLog.texts());
    CHECK(warnings == 1);
    CHECK(rate >= 540);
    INFO("VD3 skippedFrames " << stopped->skippedFrames << " frameCount " << stopped->frameCount << " rate " << rate);
}

TEST_CASE("VD4 the pre-trigger buffer is written without skips", "[record][perf]") {
    Perf perf;
    auto& pipeline = perf.begin();
    const auto name = uniqueName("VD4");
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::RAW_FRAMES);
    setRecordingName(*record, name);
    setOutputDirectory(*record, perf.dir.path);
    setStreamName(*record, "cam");
    setTransferToHost(*record, true);
    setWaitForTrigger(*record, true);
    output->link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    control->send(startControl(-1, 10000));
    auto started = takeStatus(status, std::chrono::seconds(15));
    CHECK(started->state == dai::RecordStatus::State::STARTED);
    const auto marked = std::chrono::steady_clock::now();
    const bool wrote = perf.devLog.waitFor("Record wrote frame", std::chrono::seconds(1)) || perf.hostLog.waitFor("Record wrote frame", std::chrono::milliseconds(1));
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - marked).count();
    CHECK(wrote);
    CHECK(elapsed <= 1.0);
    auto stopped = takeStatus(status, std::chrono::seconds(30));
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    CHECK(stopped->skippedFrames == 0);
    CHECK(stopped->frameCount >= 60 + 290);
}
