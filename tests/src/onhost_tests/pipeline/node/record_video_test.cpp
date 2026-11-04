#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/RecordControl.hpp"
#include "depthai/pipeline/datatype/RecordStatus.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        path = std::filesystem::temp_directory_path() / ("dai-record-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
};

std::shared_ptr<dai::ImgFrame> makeFrame(std::int64_t sequence, std::uint32_t width, std::uint32_t height, std::uint8_t fill) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(width);
    frame->setHeight(height);
    frame->setType(dai::ImgFrame::Type::NV12);
    const std::size_t bytes = static_cast<std::size_t>(width) * height * 3 / 2;
    frame->setData(std::vector<std::uint8_t>(bytes, fill));
    frame->setSequenceNum(sequence);
    frame->setTimestamp(std::chrono::steady_clock::now());
    return frame;
}

std::vector<std::uint8_t> copyData(const dai::ImgFrame& frame) {
    const auto data = frame.getData();
    return {data.begin(), data.end()};
}

std::vector<std::shared_ptr<dai::RecordStatus>> readStatuses(const std::shared_ptr<dai::MessageQueue>& queue, int count, std::chrono::milliseconds timeout) {
    std::vector<std::shared_ptr<dai::RecordStatus>> messages;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(static_cast<int>(messages.size()) < count && std::chrono::steady_clock::now() < deadline) {
        bool timedOut = false;
        auto message = queue->get<dai::RecordStatus>(std::chrono::milliseconds(20), timedOut);
        if(message) messages.push_back(std::move(message));
    }
    return messages;
}

std::optional<dai::RecordStatus> waitForState(dai::node::RecordVideo& node, dai::RecordStatus::State state, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        auto status = node.getStatus();
        if(status && status->state == state) return status;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return node.getStatus();
}

void waitForFile(const std::filesystem::path& path) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(std::chrono::steady_clock::now() < deadline) {
        if(std::filesystem::is_regular_file(path)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool fileContains(const std::filesystem::path& path, const std::string& text) {
    std::ifstream input(path, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    return contents.find(text) != std::string::npos;
}

}  // namespace

TEST_CASE("RecordControl and RecordStatus round-trip") {
    auto control = dai::RecordControl::start(4, 250);
    control->setSequenceNum(7);
    dai::RecordControl restoredControl;
    dai::utility::deserialize(dai::utility::serialize(*control), restoredControl);
    REQUIRE(restoredControl.command == dai::RecordControl::Command::START);
    REQUIRE(restoredControl.maxFrames == 4);
    REQUIRE(restoredControl.maxDurationMs == 250);
    REQUIRE(restoredControl.getSequenceNum() == 7);
    REQUIRE(restoredControl.getDatatype() == dai::DatatypeEnum::RecordControl);

    dai::RecordStatus status;
    status.state = dai::RecordStatus::State::ERROR;
    status.segment = 2;
    status.frameCount = 3;
    status.firstSequenceNum = 8;
    status.lastSequenceNum = 11;
    status.skippedFrames = 1;
    status.path = "recording/frames";
    status.error = "Record storage is full";
    status.setSequenceNum(1);
    dai::RecordStatus restoredStatus;
    dai::utility::deserialize(dai::utility::serialize(status), restoredStatus);
    REQUIRE(restoredStatus.state == dai::RecordStatus::State::ERROR);
    REQUIRE(restoredStatus.segment == 2);
    REQUIRE(restoredStatus.frameCount == 3);
    REQUIRE(restoredStatus.firstSequenceNum == 8);
    REQUIRE(restoredStatus.lastSequenceNum == 11);
    REQUIRE(restoredStatus.skippedFrames == 1);
    REQUIRE(restoredStatus.path == "recording/frames");
    REQUIRE(restoredStatus.error == "Record storage is full");
    REQUIRE(restoredStatus.getSequenceNum() == 1);
    REQUIRE(restoredStatus.getDatatype() == dai::DatatypeEnum::RecordStatus);
}

TEST_CASE("RecordConfig format JSON") {
    dai::RecordConfig config;
    config.outputDir = "/tmp/recordings";
    config.compressionLevel = dai::RecordConfig::CompressionLevel::NONE;
    config.videoEncoding.enabled = false;
    config.format = dai::RecordFormat::RAW_FRAMES;
    config.keepOnDevice = true;
    config.streams["CameraCAM_A"] = dai::RecordConfig::StreamConfig{30.5f};
    const nlohmann::json written = config;
    REQUIRE(written.at("format") == "RAW_FRAMES");
    REQUIRE(written.at("keepOnDevice") == true);
    REQUIRE(written.at("streams").at("CameraCAM_A").at("fps") == 30.5f);

    auto parsed = written.get<dai::RecordConfig>();
    REQUIRE(parsed.format == dai::RecordFormat::RAW_FRAMES);
    REQUIRE(parsed.keepOnDevice);
    REQUIRE(parsed.streams.at("CameraCAM_A").fps == 30.5f);

    auto unknown = written;
    unknown["format"] = "NOPE";
    REQUIRE_THROWS_WITH(unknown.get<dai::RecordConfig>(), Catch::Matchers::ContainsSubstring("Unknown record format: NOPE"));
}

TEST_CASE("RecordVideo rejects invalid configuration at start") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setOutputDirectory(dir.path);
    record->setRecordingName("clip");
    record->setFps(0);
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("Record invalid fps"));
}

TEST_CASE("RecordVideo rejects a stream name that ends with underscore digits") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setOutputDirectory(dir.path);
    record->setRecordingName("clip");
    record->setStreamName("cam_12");
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("Record invalid streamName: cam_12"));
}

TEST_CASE("RecordVideo accepts a stream name that ends with an underscore") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setOutputDirectory(dir.path);
    record->setRecordingName("clip_ok");
    record->setStreamName("cam_");
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setPreTriggerFrames(0);
    REQUIRE_NOTHROW(pipeline.start());
    pipeline.stop();
}

TEST_CASE("RecordVideo RAW_FRAMES cannot use the legacy file setters") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setRecordVideoFile(dir.path / "clip.avi");
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("Record RAW_FRAMES cannot be used with setRecordVideoFile"));
}

TEST_CASE("RecordVideo refuses setRunOnHost(false) without an RVC4 device") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setOutputDirectory(dir.path);
    record->setRecordingName("clip");
    record->setRunOnHost(false);
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("Record setRunOnHost(false) is not supported without an RVC4 device"));
}

TEST_CASE("RecordVideo refuses a recording directory that already exists") {
    TempDir dir;
    std::filesystem::create_directory(dir.path / "clip");
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setOutputDirectory(dir.path);
    record->setRecordingName("clip");
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("Record recording already exists: clip"));
}

TEST_CASE("RecordVideo raw frames round-trip through .dai files") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("rawclip");
    record->setStreamName("frames");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();

    std::vector<std::vector<std::uint8_t>> expected;
    for(int index = 0; index < 5; ++index) {
        auto frame = makeFrame(index, 64, 48, static_cast<std::uint8_t>(index + 1));
        expected.push_back(copyData(*frame));
        input->send(frame);
    }
    std::ostringstream lastName;
    lastName << "frames_" << std::setw(8) << std::setfill('0') << 4 << ".dai";
    waitForFile(dir.path / "rawclip" / "frames" / lastName.str());
    auto started = readStatuses(status, 1, std::chrono::seconds(2));
    REQUIRE(started.size() == 1);
    REQUIRE(started[0]->state == dai::RecordStatus::State::STARTED);
    REQUIRE(started[0]->frameCount == 0);
    REQUIRE(started[0]->getSequenceNum() == 0);

    pipeline.stop();
    auto stopped = record->getStatus();
    REQUIRE(stopped.has_value());
    REQUIRE(stopped->state == dai::RecordStatus::State::STOPPED);
    REQUIRE(stopped->frameCount == 5);
    REQUIRE(stopped->firstSequenceNum == 0);
    REQUIRE(stopped->lastSequenceNum == 4);
    REQUIRE(stopped->path == (dir.path / "rawclip").string());

    for(int index = 0; index < 5; ++index) {
        std::ostringstream name;
        name << "frames_" << std::setw(8) << std::setfill('0') << index << ".dai";
        const auto file = dir.path / "rawclip" / "frames" / name.str();
        REQUIRE(std::filesystem::is_regular_file(file));
        dai::ImgFrame loaded;
        loaded.load(file);
        REQUIRE(loaded.getWidth() == 64);
        REQUIRE(loaded.getHeight() == 48);
        REQUIRE(loaded.getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(loaded.getSequenceNum() == index);
        REQUIRE(copyData(loaded) == expected[static_cast<std::size_t>(index)]);
    }
}

TEST_CASE("RecordVideo idle stop leaves the closed marker open") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("idleclip");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(2);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(1, 16, 16, 3));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    pipeline.stop();
    REQUIRE_FALSE(record->getStatus().has_value());
    std::ifstream marker(dir.path / "idleclip" / ".dai_closed_frames");
    std::string contents;
    std::getline(marker, contents);
    REQUIRE(contents == "open");
    REQUIRE_FALSE(std::filesystem::exists(dir.path / "idleclip" / "frames"));
}

TEST_CASE("RecordVideo trigger writes the newest pre-trigger frames") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("trig");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(5);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    for(int index = 0; index < 5; ++index) input->send(makeFrame(index, 16, 16, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    control->send(dai::RecordControl::start(2, -1));
    auto stopped = waitForState(*record, dai::RecordStatus::State::STOPPED, std::chrono::seconds(2));
    REQUIRE(stopped.has_value());
    REQUIRE(stopped->frameCount == 2);
    REQUIRE(stopped->firstSequenceNum == 3);
    REQUIRE(stopped->lastSequenceNum == 4);
    REQUIRE(std::filesystem::is_regular_file(dir.path / "trig" / "frames" / "frames_00000003.dai"));
    REQUIRE(std::filesystem::is_regular_file(dir.path / "trig" / "frames" / "frames_00000004.dai"));
    REQUIRE_FALSE(std::filesystem::exists(dir.path / "trig" / "frames" / "frames_00000002.dai"));
    pipeline.stop();
}

TEST_CASE("RecordVideo closes on max duration") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("timed");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(0);
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    control->send(dai::RecordControl::start(-1, 200));
    auto messages = readStatuses(status, 2, std::chrono::seconds(2));
    REQUIRE(messages.size() == 2);
    REQUIRE(messages[0]->state == dai::RecordStatus::State::STARTED);
    REQUIRE(messages[1]->state == dai::RecordStatus::State::STOPPED);
    REQUIRE(messages[1]->frameCount == 0);
    const auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(messages[1]->getTimestamp() - messages[0]->getTimestamp());
    REQUIRE(delta.count() >= 200);
    REQUIRE(delta.count() <= 300);
    pipeline.stop();
}

TEST_CASE("RecordVideo limit of 0 ignores the whole start") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("zero");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    control->send(dai::RecordControl::start(0, -1));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    REQUIRE_FALSE(record->getStatus().has_value());
    control->send(dai::RecordControl::start(-1, -1));
    input->send(makeFrame(0, 16, 16, 2));
    waitForFile(dir.path / "zero" / "frames" / "frames_00000000.dai");
    control->send(dai::RecordControl::stop());
    auto messages = readStatuses(status, 2, std::chrono::seconds(2));
    REQUIRE(messages.size() == 2);
    REQUIRE(messages[0]->segment == 0);
    REQUIRE(messages[1]->state == dai::RecordStatus::State::STOPPED);
    REQUIRE(messages[1]->frameCount == 1);
    pipeline.stop();
}

TEST_CASE("RecordVideo ignores start while recording") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("busy");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    auto started = readStatuses(status, 1, std::chrono::seconds(2));
    REQUIRE(started.size() == 1);
    control->send(dai::RecordControl::start());
    input->send(makeFrame(0, 16, 16, 4));
    waitForFile(dir.path / "busy" / "frames" / "frames_00000000.dai");
    pipeline.stop();
    REQUIRE(record->getStatus()->state == dai::RecordStatus::State::STOPPED);
    REQUIRE(record->getStatus()->segment == 0);
    REQUIRE(std::filesystem::is_regular_file(dir.path / "busy" / "frames" / "frames_00000000.dai"));
    int daiCount = 0;
    for(const auto& entry : std::filesystem::directory_iterator(dir.path / "busy" / "frames")) {
        if(entry.path().extension() == ".dai") ++daiCount;
    }
    REQUIRE(daiCount == 1);
}

TEST_CASE("RecordVideo pre-trigger byte budget is checked once") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("budget");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(60);
    auto input = record->input.createInputQueue();
    pipeline.start();
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(1000);
    frame->setHeight(1000);
    frame->setType(dai::ImgFrame::Type::RAW8);
    frame->setData(std::vector<std::uint8_t>(5000000, 1));
    frame->setSequenceNum(0);
    input->send(frame);
    auto status = waitForState(*record, dai::RecordStatus::State::ERROR, std::chrono::seconds(2));
    REQUIRE(status.has_value());
    REQUIRE_THAT(status->error, Catch::Matchers::ContainsSubstring("300000000"));
    REQUIRE_THAT(status->error, Catch::Matchers::ContainsSubstring("268435456"));
    pipeline.stop();
}

TEST_CASE("RecordVideo keeps the pre-trigger buffer inside its byte limit") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("smallbuf");
    record->setWaitForTrigger(true);
    // N times the first frame must fit, or the node errors before the byte cap applies.
    record->setPreTriggerFrames(1);
    record->setMaxPreTriggerBytes(1000);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    input->send(makeFrame(0, 20, 20, 1));  // 600 bytes. 1 * 600 stays under the 1000 byte cap.
    input->send(makeFrame(9, 40, 40, 1));  // 2400 bytes. Older frames are dropped and this frame is not stored.
    for(int index = 1; index < 5; ++index) input->send(makeFrame(index, 20, 20, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    control->send(dai::RecordControl::start(-1, -1));
    control->send(dai::RecordControl::stop());
    auto stopped = waitForState(*record, dai::RecordStatus::State::STOPPED, std::chrono::seconds(2));
    REQUIRE(stopped.has_value());
    REQUIRE(stopped->frameCount == 1);
    REQUIRE(stopped->firstSequenceNum == 4);
    pipeline.stop();
}

TEST_CASE("RecordVideo legacy host path writes an avi") {
    TempDir dir;
    const auto video = dir.path / "legacy.avi";
    const auto metadata = dir.path / "legacy.mcap";
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRecordVideoFile(video);
    record->setRecordMetadataFile(metadata);
    record->setCompressionLevel(dai::RecordConfig::CompressionLevel::NONE);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(0, 32, 32, 8));
    waitForFile(video);
    pipeline.stop();
    REQUIRE(std::filesystem::is_regular_file(video));
    REQUIRE(std::filesystem::is_regular_file(metadata));
    REQUIRE(std::filesystem::file_size(video) > 0);
    REQUIRE_FALSE(fileContains(metadata, "lz4"));
}

TEST_CASE("RecordVideo default compression writes lz4 metadata") {
    TempDir dir;
    const auto metadata = dir.path / "default.mcap";
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRecordVideoFile(dir.path / "default.avi");
    record->setRecordMetadataFile(metadata);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(0, 32, 32, 8));
    waitForFile(metadata);
    pipeline.stop();
    REQUIRE(fileContains(metadata, "lz4"));
}

TEST_CASE("RecordVideo video segments insert the segment suffix") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::VIDEO);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("segments");
    record->setStreamName("frames");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    auto control = record->inputControl.createInputQueue();
    pipeline.start();
    control->send(dai::RecordControl::start(-1, -1));
    input->send(makeFrame(0, 32, 32, 1));
    waitForFile(dir.path / "segments" / "frames.avi");
    control->send(dai::RecordControl::stop());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    control->send(dai::RecordControl::start(-1, -1));
    input->send(makeFrame(1, 32, 32, 2));
    waitForFile(dir.path / "segments" / "frames_s1.avi");
    control->send(dai::RecordControl::stop());
    auto stopped = waitForState(*record, dai::RecordStatus::State::STOPPED, std::chrono::seconds(2));
    REQUIRE(stopped.has_value());
    REQUIRE(stopped->segment == 1);
    REQUIRE(std::filesystem::is_regular_file(dir.path / "segments" / "frames.avi"));
    REQUIRE(std::filesystem::is_regular_file(dir.path / "segments" / "frames_s1.avi"));
    pipeline.stop();
}

TEST_CASE("RecordVideo reports skipped frames and still writes a rewind") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("gaps");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(1, 16, 16, 1));
    input->send(makeFrame(4, 16, 16, 2));
    input->send(makeFrame(3, 16, 16, 3));
    waitForFile(dir.path / "gaps" / "frames" / "frames_00000003.dai");
    pipeline.stop();
    auto status = record->getStatus();
    REQUIRE(status.has_value());
    REQUIRE(status->state == dai::RecordStatus::State::STOPPED);
    REQUIRE(status->skippedFrames == 2);
    REQUIRE(std::filesystem::is_regular_file(dir.path / "gaps" / "frames" / "frames_00000001.dai"));
    REQUIRE(std::filesystem::is_regular_file(dir.path / "gaps" / "frames" / "frames_00000004.dai"));
    REQUIRE(std::filesystem::is_regular_file(dir.path / "gaps" / "frames" / "frames_00000003.dai"));
}

TEST_CASE("RecordVideo rejects a duplicate raw file name") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("dup");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(1, 16, 16, 1));
    input->send(makeFrame(1, 16, 16, 2));
    auto status = waitForState(*record, dai::RecordStatus::State::ERROR, std::chrono::seconds(2));
    REQUIRE(status.has_value());
    REQUIRE_THAT(status->error, Catch::Matchers::ContainsSubstring("Record file already exists: frames_00000001.dai"));
    pipeline.stop();
}

TEST_CASE("RecordVideo rejects a negative raw sequence number") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("neg");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    input->send(makeFrame(-1, 16, 16, 1));
    auto status = waitForState(*record, dai::RecordStatus::State::ERROR, std::chrono::seconds(2));
    REQUIRE(status.has_value());
    REQUIRE_THAT(status->error, Catch::Matchers::ContainsSubstring("Record sequenceNum -1 is below 0"));
    pipeline.stop();
}

TEST_CASE("RecordVideo rejects H265 in VIDEO format") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::VIDEO);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("hevc");
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    auto encoded = std::make_shared<dai::EncodedFrame>();
    encoded->setWidth(16);
    encoded->setHeight(16);
    encoded->setProfile(dai::EncodedFrame::Profile::HEVC);
    encoded->setData(std::vector<std::uint8_t>{0, 0, 0, 1});
    encoded->setSequenceNum(0);
    input->send(encoded);
    auto status = waitForState(*record, dai::RecordStatus::State::ERROR, std::chrono::seconds(2));
    REQUIRE(status.has_value());
    REQUIRE_THAT(status->error, Catch::Matchers::ContainsSubstring("Record H265 is not supported in VIDEO format"));
    pipeline.stop();
}

TEST_CASE("RecordVideo still records a frame above the expected write rate") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("rate");
    record->setFps(30);
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue();
    pipeline.start();
    // 2000 x 1500 NV12 is 4,500,000 bytes. At 30 fps that is 135 MB/s and must warn, not drop the frame.
    input->send(makeFrame(0, 2000, 1500, 7));
    waitForFile(dir.path / "rate" / "frames" / "frames_00000000.dai");
    pipeline.stop();
    REQUIRE(std::filesystem::is_regular_file(dir.path / "rate" / "frames" / "frames_00000000.dai"));
    REQUIRE(record->getStatus()->state == dai::RecordStatus::State::STOPPED);
}

TEST_CASE("RecordVideo device scenarios need RVC4 firmware") {
    bool rvc4 = false;
    for(const auto& device : dai::DeviceBase::getAllAvailableDevices()) {
        if(device.platform == X_LINK_RVC4) rvc4 = true;
    }
    if(!rvc4) SKIP("S7, S10, S11 and S15 need an RVC4 device and recording-store firmware");
    SKIP("S7, S10, S11 and S15 need recording-store firmware that is not in this repository");
}
