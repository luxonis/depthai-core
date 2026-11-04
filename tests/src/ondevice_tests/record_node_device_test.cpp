#include <catch2/catch_all.hpp>
#include <algorithm>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <filesystem>
#include <fstream>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/utility/RecordingStore.hpp"

namespace {

std::shared_ptr<dai::Device> openRvc4() {
    const auto [found, info] = dai::Device::getAnyAvailableDevice();
    if(!found) return nullptr;
    auto device = std::make_shared<dai::Device>(info);
    if(device->getPlatform() != dai::Platform::RVC4) return nullptr;
    return device;
}

std::shared_ptr<dai::RecordStatus> readStatus(const std::shared_ptr<dai::MessageQueue>& queue, dai::RecordStatus::State state) {
    bool timeout = false;
    auto message = queue->get<dai::RecordStatus>(std::chrono::seconds(5), timeout);
    REQUIRE_FALSE(timeout);
    REQUIRE(message);
    REQUIRE(message->state == state);
    return message;
}

std::shared_ptr<dai::ImgFrame> frame(std::int64_t sequence) {
    auto image = std::make_shared<dai::ImgFrame>();
    image->setType(dai::ImgFrame::Type::NV12);
    image->setWidth(64);
    image->setHeight(48);
    image->setSequenceNum(sequence);
    image->setTimestamp(std::chrono::steady_clock::now());
    image->setData(std::vector<std::uint8_t>(64 * 48 * 3 / 2, static_cast<std::uint8_t>(sequence)));
    return image;
}

struct OutputDirectory {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / ("dai-device-" + dai::makeRecordingTimestamp());
    ~OutputDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
};

}  // namespace

TEST_CASE("Device recorder writes raw frames and transfers at stop", "[record][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 device connected");
    OutputDirectory output;
    const auto name = "raw_" + dai::makeRecordingTimestamp();
    dai::Pipeline pipeline(device);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(false).setFormat(dai::RecordFormat::RAW_FRAMES).setRecordingName(name).setOutputDirectory(output.path);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    readStatus(status, dai::RecordStatus::State::STARTED);
    const auto original = frame(17);
    input->send(original);
    // STOP travels on a separate link. Wait until the frame has reached the writer.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    control->send(dai::RecordControl::stop());
    const auto stopped = readStatus(status, dai::RecordStatus::State::STOPPED);
    REQUIRE(stopped->frameCount == 1);
    REQUIRE(stopped->firstSequenceNum == 17);
    pipeline.stop();
    const auto path = output.path / name / "frames" / "frames_00000017.dai";
    REQUIRE(std::filesystem::is_regular_file(path));
    dai::ImgFrame restored;
    restored.load(path);
    REQUIRE(restored.getSequenceNum() == original->getSequenceNum());
    REQUIRE(restored.getTimestamp() == original->getTimestamp());
    REQUIRE(restored.getWidth() == 64);
    REQUIRE(restored.getHeight() == 48);
    const auto bytes = restored.getData();
    REQUIRE(bytes.size() == original->getData().size());
    REQUIRE(std::all_of(bytes.begin(), bytes.end(), [](auto value) { return value == 17; }));
    const auto last = record->getStatus();
    REQUIRE(last);
    REQUIRE(last->state == dai::RecordStatus::State::STOPPED);
}

TEST_CASE("Device recording store protects writers and transfers retained recordings", "[record][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 device connected");
    OutputDirectory output;
    const auto name = "retained_" + dai::makeRecordingTimestamp();
    dai::Pipeline pipeline(device);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(false).setFormat(dai::RecordFormat::RAW_FRAMES).setRecordingName(name).setTransferToHost(false).setWaitForTrigger(true).setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    REQUIRE_THROWS(device->deleteRecording(name));
    REQUIRE_THROWS(device->downloadRecording(name, output.path));
    control->send(dai::RecordControl::start(2, -1));
    readStatus(status, dai::RecordStatus::State::STARTED);
    input->send(frame(10));
    input->send(frame(13));
    auto stopped = readStatus(status, dai::RecordStatus::State::STOPPED);
    REQUIRE(stopped->frameCount == 2);
    REQUIRE(stopped->skippedFrames == 2);
    REQUIRE_THROWS(device->deleteRecording(name));
    device->prepareRecordingDownload();
    const auto recordings = device->listRecordings();
    const auto found = std::find_if(recordings.begin(), recordings.end(), [&](const auto& entry) { return entry.name == name; });
    REQUIRE(found != recordings.end());
    REQUIRE(found->complete);
    REQUIRE(found->sizeBytes > 0);
    const auto nested = output.path / "created" / "by" / "download";
    device->downloadRecording(name, nested);
    REQUIRE(std::filesystem::is_regular_file(nested / name / "frames" / "frames_00000010.dai"));
    REQUIRE(std::filesystem::is_regular_file(nested / name / "frames" / "frames_00000013.dai"));
    REQUIRE_THROWS(device->downloadRecording(name, nested));
    device->deleteRecording(name);
    pipeline.stop();
}

TEST_CASE("Device Script constructs record messages and triggers segments", "[record][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 device connected");
    OutputDirectory output;
    dai::Pipeline pipeline(device);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(false).setFormat(dai::RecordFormat::RAW_FRAMES).setRecordingName("script_" + dai::makeRecordingTimestamp()).setOutputDirectory(output.path);
    record->setWaitForTrigger(true).setPreTriggerFrames(0);
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
import time
RecordStatus()
time.sleep(0.1)
node.outputs['control'].send(RecordControl.start(1, -1))
)");
    script->outputs["control"].link(record->inputControl);
    auto input = record->input.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    readStatus(status, dai::RecordStatus::State::STARTED);
    input->send(frame(0));
    REQUIRE(readStatus(status, dai::RecordStatus::State::STOPPED)->frameCount == 1);
    pipeline.stop();
}

TEST_CASE("Device recorder muxes camera video without host frame transport", "[record][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 device connected");
    const auto profile = GENERATE(dai::VideoEncoderProperties::Profile::H264_MAIN, dai::VideoEncoderProperties::Profile::MJPEG);
    OutputDirectory output;
    const auto name = "video_" + dai::makeRecordingTimestamp();
    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setProfile(profile);
    camera->requestOutput({320, 240}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10)->link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(false).setRecordingName(name).setOutputDirectory(output.path).setFps(10);
    encoder->out.link(record->input);
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    readStatus(status, dai::RecordStatus::State::STARTED);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    control->send(dai::RecordControl::stop());
    const auto stopped = readStatus(status, dai::RecordStatus::State::STOPPED);
    REQUIRE(stopped->frameCount > 0);
    pipeline.stop();
    cv::VideoCapture video((output.path / name / "frames.mp4").string());
    REQUIRE(video.isOpened());
    cv::Mat decoded;
    int count = 0;
    while(video.read(decoded)) {
        REQUIRE(decoded.cols == 320);
        REQUIRE(decoded.rows == 240);
        ++count;
    }
    REQUIRE(count == stopped->frameCount);
    REQUIRE(std::filesystem::file_size(output.path / name / "frames.mcap") > 0);
}
