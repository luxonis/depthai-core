extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <regex>

#include "../../../helpers/record_test_support.hpp"

namespace {

using Catch::Matchers::ContainsSubstring;
using record_test::expectStatus;
using record_test::HostRecord;
using record_test::makeEnc;
using record_test::makeEnc265;
using record_test::makeEncJ;
using record_test::makeEncPad;
using record_test::makeImg;
using record_test::makeMcap;
using record_test::sendFrame;

constexpr auto kWait = std::chrono::seconds(10);

void requireStartError(record_test::HostRecord& fx, const char* text = nullptr) {
    if(text != nullptr) {
        REQUIRE_THROWS_WITH(fx.pipeline.start(), ContainsSubstring(text));
    } else {
        REQUIRE_THROWS(fx.pipeline.start());
    }
}

void wrote(record_test::HostRecord& fx, std::int64_t sequence) {
    REQUIRE(fx.log.waitFor("Record wrote frame " + std::to_string(sequence), kWait));
}

void idle(record_test::HostRecord& fx, std::int64_t sequence) {
    REQUIRE(fx.log.waitFor("Record idle frame " + std::to_string(sequence), kWait));
}

void sendRaw(record_test::HostRecord& fx, int n, std::int64_t sequence) {
    sendFrame(fx.frames, makeImg(64, 48, n, sequence));
}

void sendRawWrote(record_test::HostRecord& fx, int n) {
    sendRaw(fx, n, n);
    wrote(fx, n);
}

void armTrigger(record_test::HostRecord& fx, std::int32_t preTriggerFrames) {
    record_test::setWaitForTrigger(*fx.node, true);
    record_test::setPreTriggerFrames(*fx.node, preTriggerFrames);
    // inputControl must exist before start(); createInputQueue throws once the pipeline is built.
    fx.control();
}

int warnCount(const record_test::HostRecord& fx, const std::string& substring = "") {
    return fx.log.count(record_test::LineLevel::WARN, substring);
}

std::shared_ptr<dai::RecordStatus> readStopped(record_test::HostRecord& fx) {
    auto stopped = record_test::stopAndStatus(fx.pipeline, fx.status, *fx.node);
    REQUIRE(stopped != nullptr);
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    return stopped;
}

void expectFiles(const std::filesystem::path& dir, const std::vector<std::string>& names) {
    CHECK(record_test::allFiles(dir) == names);
}

void beginRecord(record_test::HostRecord& fx) {
    record_test::setFps(*fx.node, 30u);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
}

std::shared_ptr<dai::RecordStatus> stoppedOf(
    record_test::HostRecord& fx, std::int32_t segment, std::int32_t frames, std::int64_t first, std::int64_t last, std::int32_t skipped) {
    auto stopped = readStopped(fx);
    CHECK(stopped->segment == segment);
    CHECK(stopped->frameCount == frames);
    CHECK(stopped->firstSequenceNum == first);
    CHECK(stopped->lastSequenceNum == last);
    CHECK(stopped->skippedFrames == skipped);
    return stopped;
}

void sendControl(record_test::HostRecord& fx, const std::shared_ptr<dai::RecordControl>& control) {
    fx.control()->send(control);
}

}  // namespace

using Catch::Matchers::ContainsSubstring;
using record_test::expectStatus;
using record_test::HostRecord;
using record_test::makeEnc;
using record_test::makeEnc265;
using record_test::makeEncJ;
using record_test::makeEncPad;
using record_test::makeImg;
using record_test::makeMcap;
using record_test::sendFrame;

TEST_CASE("data record helpers", "[record][data]") {
    if(!record_test::recordDataAvailable()) SKIP("needs the generated data");
    const auto units = record_test::accessUnits(record_test::recordDataRoot() / "plain_h264_small.h264");
    REQUIRE(units.size() == 20);
    for(const auto& unit : units) {
        REQUIRE(unit.size() >= 5);
        CHECK(unit[0] == 0x00);
        CHECK(unit[1] == 0x00);
        CHECK(unit[2] == 0x00);
        CHECK(unit[3] == 0x01);
        CHECK(unit[4] == 0x67);
    }

    record_test::TempDir dir;
    std::vector<record_test::McapEntry> entries;
    for(int n = 0; n < 20; ++n) entries.push_back(record_test::McapEntry{n, 100 * n, 64, 48});
    const auto mcap = dir.path / "plain_h264_small.mcap";
    makeMcap(mcap, entries);
    CHECK(record_test::countVideoFrames(record_test::recordDataRoot() / "plain_h264_small.mp4", mcap) == 20);

    const auto imgPath = dir.path / "frame.dai";
    auto sent = makeImg(64, 48, 3);
    sent->save(imgPath);
    auto loaded = record_test::loadImg(imgPath);
    CHECK(loaded->getSequenceNum() == 3);
    CHECK(loaded->event == dai::FrameEvent::NONE);
    CHECK(loaded->getWidth() == 64);
    CHECK(loaded->getHeight() == 48);
    CHECK(loaded->cam.exposureTimeUs == 1003);
    CHECK(loaded->cam.sensitivityIso == 100);
    CHECK(loaded->category == 7);
    CHECK(loaded->instanceNum == 1);
    CHECK_FALSE(loaded->getTimestampSystem().has_value());
    CHECK(record_test::timestampMs(loaded->getTimestamp()) == 300);
    CHECK(record_test::timestampMs(loaded->getTimestampDevice()) == 300);

    const auto encPath = dir.path / "enc.dai";
    auto enc = makeEnc(3);
    const auto bits = enc->getData();
    enc->save(encPath);
    auto encLoaded = record_test::loadEnc(encPath);
    CHECK(encLoaded->getSequenceNum() == 3);
    CHECK(encLoaded->getProfile() == dai::EncodedFrame::Profile::AVC);
    CHECK(encLoaded->getWidth() == 64);
    CHECK(encLoaded->getQuality() == 80);
    CHECK(encLoaded->getData().size() == bits.size());

    const auto none = dir.path / "none.mcap";
    const auto def = dir.path / "default.mcap";
    makeMcap(none, {record_test::McapEntry{0, 0, 64, 48}}, dai::RecordConfig::CompressionLevel::NONE);
    makeMcap(def, {record_test::McapEntry{0, 0, 64, 48}}, dai::RecordConfig::CompressionLevel::DEFAULT);
    CHECK(record_test::mcapCompression(none) == "");
    CHECK(record_test::mcapCompression(def) == "lz4");
}

TEST_CASE("A15 chained defaults record two frames", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 30u);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(64, 48, 0));
    REQUIRE(fx.log.waitFor("Record wrote frame 0", std::chrono::seconds(10)));
    REQUIRE(record_test::waitUntil([&] { return record_test::fileCount(fx.streamDir()) == 1; }, std::chrono::seconds(10)));
    sendFrame(fx.frames, makeImg(64, 48, 1));
    REQUIRE(fx.log.waitFor("Record wrote frame 1", std::chrono::seconds(10)));
    REQUIRE(record_test::waitUntil([&] { return record_test::fileCount(fx.streamDir()) == 2; }, std::chrono::seconds(10)));
    auto stopped = record_test::stopAndStatus(fx.pipeline, fx.status, *fx.node);
    REQUIRE(stopped != nullptr);
    CHECK(stopped->state == dai::RecordStatus::State::STOPPED);
    CHECK(stopped->frameCount == 2);
    CHECK(stopped->skippedFrames == 0);
    auto names = record_test::allFiles(fx.streamDir());
    REQUIRE(names.size() == 2);
    CHECK(names[0] == "cam_00000000.dai");
    CHECK(names[1] == "cam_00000001.dai");
}

TEST_CASE("A1 fps zero errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 0.f);
    requireStartError(fx, "fps");
}

TEST_CASE("A2 fps negative errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, -1.f);
    requireStartError(fx, "fps");
}

TEST_CASE("A3 fps nan errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, NAN);
    requireStartError(fx, "fps");
}

TEST_CASE("A4 max pre-trigger bytes zero errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setMaxPreTriggerBytes(*fx.node, 0);
    requireStartError(fx);
}

TEST_CASE("A5 empty stream name errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "");
    requireStartError(fx);
}

TEST_CASE("A6 stream name with slash errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "a/b");
    requireStartError(fx);
}

TEST_CASE("A7 stream name ending in digits errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "cam_12");
    requireStartError(fx);
}

TEST_CASE("A8 stream name cam_ writes a doubled underscore", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "cam_");
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(std::filesystem::exists(fx.recDir() / "cam_" / "cam__00000000.dai"));
}

TEST_CASE("A9 stream name cam12 is a single directory", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "cam12");
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(std::filesystem::exists(fx.recDir() / "cam12" / "cam12_00000000.dai"));
}

TEST_CASE("A10 stream name cam_a is legal", "[record][host]") {
    HostRecord fx;
    record_test::setStreamName(*fx.node, "cam_a");
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(std::filesystem::exists(fx.recDir() / "cam_a" / "cam_a_00000000.dai"));
}

TEST_CASE("A11 recording name dotdot errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setRecordingName(*fx.node, "..");
    requireStartError(fx);
}

TEST_CASE("A12 empty recording name errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setRecordingName(*fx.node, "");
    requireStartError(fx);
}

TEST_CASE("A13 recording name with colon errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setRecordingName(*fx.node, "rec:1");
    requireStartError(fx);
}

TEST_CASE("A14 transfer to host is ignored on the host", "[record][host]") {
    HostRecord fx;
    record_test::setTransferToHost(*fx.node, false);
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(warnCount(fx, "Record setTransferToHost ignored on the host") == 1);
    CHECK(std::filesystem::exists(fx.streamDir() / "cam_00000000.dai"));
}

TEST_CASE("A16 wait for trigger without a control source warns", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, true);
    fx.pipeline.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    fx.pipeline.stop();
    CHECK(warnCount(fx, "trigger") == 1);
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    CHECK(record_test::getStatus(*fx.node) == nullptr);
}

TEST_CASE("A17 run on host false errors at start", "[record][host]") {
    HostRecord fx;
    record_test::setRunOnHost(*fx.node, false);
    requireStartError(fx);
}

TEST_CASE("A18 a second pipeline with the same name throws", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    const auto before = record_test::allFiles(fx.streamDir());
    dai::Pipeline second(false);
    auto node = second.create<dai::node::RecordVideo>();
    record_test::watchNode(*node);
    record_test::setFormat(*node, record_test::Format::RAW_FRAMES);
    record_test::setRecordingName(*node, "test");
    record_test::setOutputDirectory(*node, fx.dir.path);
    record_test::setStreamName(*node, "cam");
    REQUIRE_THROWS_WITH(second.start(), ContainsSubstring("test"));
    CHECK(record_test::allFiles(fx.streamDir()) == before);
}

TEST_CASE("A19 two nodes with the same name and stream error", "[record][host]") {
    HostRecord fx;
    auto other = fx.pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*other);
    record_test::setFormat(*other, record_test::Format::RAW_FRAMES);
    record_test::setRecordingName(*other, "test");
    record_test::setOutputDirectory(*other, fx.dir.path);
    record_test::setStreamName(*other, "cam");
    requireStartError(fx);
}

TEST_CASE("A20 two streams in one recording", "[record][host]") {
    HostRecord fx;
    auto right = fx.pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*right);
    record_test::setFormat(*right, record_test::Format::RAW_FRAMES);
    record_test::setRecordingName(*right, "test");
    record_test::setOutputDirectory(*right, fx.dir.path);
    record_test::setStreamName(*fx.node, "left");
    record_test::setStreamName(*right, "right");
    auto rightIn = right->input.createInputQueue();
    auto rightStatus = right->status.createOutputQueue(16, true);
    beginRecord(fx);
    expectStatus(rightStatus, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    for(int n = 0; n < 2; ++n) {
        sendFrame(fx.frames, makeImg(64, 48, n));
        sendFrame(rightIn, makeImg(64, 48, n));
        REQUIRE(record_test::waitUntil(
            [&] { return record_test::fileCount(fx.recDir() / "left", "left") == n + 1 && record_test::fileCount(fx.recDir() / "right", "right") == n + 1; },
            kWait));
    }
    fx.pipeline.stop();
    expectFiles(fx.recDir() / "left", {"left_00000000.dai", "left_00000001.dai"});
    expectFiles(fx.recDir() / "right", {"right_00000000.dai", "right_00000001.dai"});
    int entries = 0;
    for(const auto& entry : std::filesystem::directory_iterator(fx.recDir())) {
        (void)entry;
        ++entries;
    }
    CHECK(entries == 2);
}

TEST_CASE("A21 output directory is created", "[record][host]") {
    HostRecord fx;
    const auto nested = fx.dir.path / "a" / "b" / "c";
    record_test::setOutputDirectory(*fx.node, nested);
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(std::filesystem::exists(nested / "test" / "cam" / "cam_00000000.dai"));
}

TEST_CASE("A22 default recording name is a timestamp", "[record][host]") {
    HostRecord fx(false);
    record_test::setFormat(*fx.node, record_test::Format::RAW_FRAMES);
    record_test::setOutputDirectory(*fx.node, fx.dir.path);
    record_test::setStreamName(*fx.node, "cam");
    beginRecord(fx);
    sendRawWrote(fx, 0);
    readStopped(fx);
    const std::regex name("^recording_\\d{4}-\\d{2}-\\d{2}_\\d{2}-\\d{2}-\\d{2}-\\d{3}$");
    std::vector<std::string> dirs;
    for(const auto& entry : std::filesystem::directory_iterator(fx.dir.path)) {
        if(entry.is_directory()) dirs.push_back(entry.path().filename().string());
    }
    REQUIRE(dirs.size() == 1);
    CHECK(std::regex_match(dirs[0], name));
    CHECK(record_test::fileCount(fx.dir.path / dirs[0] / "cam") == 1);
}

TEST_CASE("A23 raw frames ignore compression none", "[record][host]") {
    HostRecord fx;
    record_test::setCompressionLevel(*fx.node, record_test::Compression::NONE);
    beginRecord(fx);
    sendRawWrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(warnCount(fx) == 0);
    expectFiles(fx.streamDir(), {"cam_00000000.dai"});
}

void legacyFiles(HostRecord& fx) {
    record_test::setRecordVideoFile(*fx.node, fx.dir.path / "v.avi");
    record_test::setRecordMetadataFile(*fx.node, fx.dir.path / "v.mcap");
}

TEST_CASE("A24 legacy video and metadata files", "[record][host]") {
    HostRecord fx(false);
    legacyFiles(fx);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    expectFiles(fx.dir.path, {"v.avi", "v.mcap"});
    CHECK(record_test::countVideoFrames(fx.dir.path / "v.avi", fx.dir.path / "v.mcap") == 2);
}

TEST_CASE("A25 legacy layout rejects setFormat", "[record][host]") {
    HostRecord fx(false);
    legacyFiles(fx);
    record_test::setFormat(*fx.node, record_test::Format::RAW_FRAMES);
    requireStartError(fx);
}

TEST_CASE("A26 legacy trigger writes one file per segment", "[record][host]") {
    HostRecord fx(false);
    legacyFiles(fx);
    record_test::setWaitForTrigger(*fx.node, true);
    fx.control();
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 1, 0, 0, 0);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendRawWrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 1, 1, 1, 1, 0);
    fx.pipeline.stop();
    expectFiles(fx.dir.path, {"v.avi", "v.mcap", "v_s1.avi", "v_s1.mcap"});
    CHECK(record_test::countVideoFrames(fx.dir.path / "v.avi", fx.dir.path / "v.mcap") == 1);
    CHECK(record_test::countVideoFrames(fx.dir.path / "v_s1.avi", fx.dir.path / "v_s1.mcap") == 1);
}

TEST_CASE("A27 legacy layout ignores setOutputDirectory", "[record][host]") {
    HostRecord fx(false);
    legacyFiles(fx);
    record_test::setOutputDirectory(*fx.node, fx.dir.path / "other");
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    CHECK(warnCount(fx, "Record legacy layout: setOutputDirectory ignored") == 1);
    expectFiles(fx.dir.path, {"v.avi", "v.mcap"});
    CHECK_FALSE(std::filesystem::exists(fx.dir.path / "other"));
}

TEST_CASE("R1 five raw frames and a stopped status", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    for(int n = 0; n < 5; ++n) sendRawWrote(fx, n);
    auto stopped = stoppedOf(fx, 0, 5, 0, 4, 0);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai", "cam_00000002.dai", "cam_00000003.dai", "cam_00000004.dai"});
    CHECK(record_test::endsWith(stopped->path, "/test"));
}

TEST_CASE("R2 loaded raw frames keep the sent fields", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    for(int n = 0; n < 5; ++n) sendRawWrote(fx, n);
    stoppedOf(fx, 0, 5, 0, 4, 0);
    for(int n = 0; n < 5; ++n) {
        auto loaded = record_test::loadImg(fx.streamDir() / record_test::frameFileName("cam", n));
        auto sent = makeImg(64, 48, n);
        CHECK(loaded->getSequenceNum() == n);
        CHECK(record_test::timestampMs(loaded->getTimestamp()) == n * 100);
        CHECK(record_test::timestampMs(loaded->getTimestampDevice()) == n * 100);
        CHECK_FALSE(loaded->getTimestampSystem().has_value());
        CHECK(loaded->getType() == dai::ImgFrame::Type::NV12);
        CHECK(loaded->getWidth() == 64);
        CHECK(loaded->getHeight() == 48);
        CHECK(loaded->sourceFb.width == 64);
        CHECK(loaded->cam.exposureTimeUs == 1000 + n);
        CHECK(loaded->cam.sensitivityIso == 100);
        CHECK(loaded->category == 7);
        CHECK(loaded->instanceNum == 1);
        CHECK(loaded->event == dai::FrameEvent::NONE);
        CHECK(loaded->transformation.getSize().first == 64);
        CHECK(loaded->transformation.getSize().second == 48);
        const auto intrinsics = loaded->transformation.getIntrinsicMatrix();
        CHECK(intrinsics[0][0] == 32.f);
        CHECK(std::equal(loaded->getData().begin(), loaded->getData().end(), sent->getData().begin()));
        CHECK(loaded->getData().size() == sent->getData().size());
    }
}

TEST_CASE("R3 encoded frames round trip in raw mode", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginRecord(fx);
    sendFrame(fx.frames, makeEnc(0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeEnc(1));
    wrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    CHECK(warnCount(fx) == 1);
    for(int n = 0; n < 2; ++n) {
        auto loaded = record_test::loadEnc(fx.streamDir() / record_test::frameFileName("cam", n));
        auto sent = makeEnc(n);
        CHECK(loaded->getProfile() == dai::EncodedFrame::Profile::AVC);
        CHECK(loaded->getSequenceNum() == n);
        CHECK(loaded->getWidth() == 64);
        CHECK(loaded->getHeight() == 48);
        CHECK(loaded->getQuality() == 80);
        CHECK(loaded->getData().size() == sent->getData().size());
        CHECK(std::equal(loaded->getData().begin(), loaded->getData().end(), sent->getData().begin()));
    }
}

TEST_CASE("R4 sequence numbers of eight and nine digits", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRaw(fx, 0, 99999999);
    wrote(fx, 99999999);
    sendRaw(fx, 1, 100000000);
    wrote(fx, 100000000);
    readStopped(fx);
    expectFiles(fx.streamDir(), {"cam_100000000.dai", "cam_99999999.dai"});
}

TEST_CASE("R5 a negative sequence is an error", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRaw(fx, 0, -1);
    auto err = expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 0, -1, -1, 0);
    CHECK(err->error.find("-1") != std::string::npos);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
    CHECK(record_test::allFiles(fx.recDir()).empty());
}

TEST_CASE("R6 a repeated sequence stops the pipeline", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendRaw(fx, 2, 1);
    auto err = expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 2, 0, 1, 0);
    CHECK(fx.status->tryGet<dai::RecordStatus>() == nullptr);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
    CHECK(record_test::fileCount(fx.streamDir()) == 2);
    CHECK(fx.log.countSubstring("Record sequence went back (1 -> 1)") == 1);
    (void)err;
}

TEST_CASE("R7 files appear under their final names", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    for(int n = 0; n < 3; ++n) sendRawWrote(fx, n);
    REQUIRE(record_test::fileCount(fx.streamDir()) == 3);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai", "cam_00000002.dai"});
    fx.pipeline.stop();
}

TEST_CASE("R8 h264 frames warn once", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginRecord(fx);
    sendFrame(fx.frames, makeEnc(0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeEnc(1));
    wrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    CHECK(warnCount(fx, "H264") == 1);
    CHECK(record_test::fileCount(fx.streamDir()) == 2);
}

TEST_CASE("R9 h265 frames warn once", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginRecord(fx);
    sendFrame(fx.frames, makeEnc265(0));
    wrote(fx, 0);
    stoppedOf(fx, 0, 1, 0, 0, 0);
    CHECK(warnCount(fx, "H265") == 1);
    CHECK(record_test::loadEnc(fx.streamDir() / "cam_00000000.dai")->getProfile() == dai::EncodedFrame::Profile::HEVC);
}

TEST_CASE("R10 mixing image and encoded frames errors", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginRecord(fx);
    sendRawWrote(fx, 0);
    sendFrame(fx.frames, makeEnc(1));
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 1, 0, 0, 0);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
    CHECK(record_test::fileCount(fx.streamDir()) == 1);
}

TEST_CASE("R11 a bitstream image frame errors", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    auto frame = makeImg(64, 48, 0);
    frame->setType(dai::ImgFrame::Type::BITSTREAM);
    frame->setData(std::vector<std::uint8_t>(100, 7));
    sendFrame(fx.frames, frame);
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 0, -1, -1, 0);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
}

TEST_CASE("R12 sequence going back still writes the files", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRaw(fx, 0, 3);
    wrote(fx, 3);
    sendRaw(fx, 1, 1);
    wrote(fx, 1);
    sendRaw(fx, 2, 2);
    wrote(fx, 2);
    stoppedOf(fx, 0, 3, 3, 2, 0);
    expectFiles(fx.streamDir(), {"cam_00000001.dai", "cam_00000002.dai", "cam_00000003.dai"});
    CHECK(warnCount(fx, "Record sequence went back (3 -> 1)") == 1);
    CHECK(fx.log.countSubstring("Record skipped") == 0);
}

TEST_CASE("R13 getStatus follows the queue", "[record][host]") {
    HostRecord fx;
    CHECK(record_test::getStatus(*fx.node) == nullptr);
    beginRecord(fx);
    for(int n = 0; n < 3; ++n) sendRawWrote(fx, n);
    auto during = record_test::getStatus(*fx.node);
    REQUIRE(during != nullptr);
    CHECK(during->state == dai::RecordStatus::State::STARTED);
    CHECK(during->segment == 0);
    for(int n = 3; n < 5; ++n) sendRawWrote(fx, n);
    auto stopped = stoppedOf(fx, 0, 5, 0, 4, 0);
    auto after = record_test::getStatus(*fx.node);
    REQUIRE(after != nullptr);
    CHECK(after->state == stopped->state);
    CHECK(after->frameCount == 5);
}

TEST_CASE("T1 idle frames write nothing", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    for(int n = 0; n < 3; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    // pipeline.stop() destroys the status queue, and tryGet/get then throw "MessageQueue was closed".
    // Probe while the queue is still open, then confirm the node published nothing.
    bool timedOut = false;
    CHECK(fx.status->get<dai::RecordStatus>(std::chrono::milliseconds(500), timedOut) == nullptr);
    fx.pipeline.stop();
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    CHECK(record_test::getStatus(*fx.node) == nullptr);
    CHECK(fx.log.countSubstring("Record idle frame 0") == 1);
    CHECK(fx.log.countSubstring("Record idle frame 1") == 1);
    CHECK(fx.log.countSubstring("Record idle frame 2") == 1);
}

TEST_CASE("T2 two triggered segments", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    for(int n = 0; n < 3; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    for(int n = 3; n <= 5; ++n) sendRawWrote(fx, n);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 3, 5, 0);
    for(int n = 6; n <= 7; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendRawWrote(fx, 8);
    sendRawWrote(fx, 9);
    stoppedOf(fx, 1, 2, 8, 9, 0);
    expectFiles(fx.streamDir(), {"cam_00000003.dai", "cam_00000004.dai", "cam_00000005.dai", "cam_00000008.dai", "cam_00000009.dai"});
}

TEST_CASE("T3 stop and a second start are ignored", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::stopControl());
    REQUIRE(fx.log.waitFor("Record ignored stop", kWait));
    CHECK(warnCount(fx) == 0);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendControl(fx, record_test::startControl());
    REQUIRE(fx.log.waitFor("Record start ignored", kWait));
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 0, 1, 0);
    fx.pipeline.stop();
    CHECK(warnCount(fx, "Record start ignored") == 1);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
}

TEST_CASE("T4 a frame limit closes the segment", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendRaw(fx, 0, 0);
    idle(fx, 0);
    sendRaw(fx, 1, 1);
    idle(fx, 1);
    sendControl(fx, record_test::startControl(2, -1));
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 2);
    sendRawWrote(fx, 3);
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 2, 3, 0);
    for(int n = 4; n <= 6; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    expectFiles(fx.streamDir(), {"cam_00000002.dai", "cam_00000003.dai"});
    sendControl(fx, record_test::stopControl());
    REQUIRE(fx.log.waitFor("Record ignored stop", kWait));
    CHECK(warnCount(fx) == 0);
    fx.pipeline.stop();
}

TEST_CASE("T5 a frame limit of zero is ignored", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(0, -1));
    REQUIRE(fx.log.waitFor("Record limit of 0 ignored", kWait));
    sendRaw(fx, 0, 0);
    idle(fx, 0);
    sendRaw(fx, 1, 1);
    idle(fx, 1);
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    bool timedOut = false;
    CHECK(fx.status->get<dai::RecordStatus>(std::chrono::milliseconds(300), timedOut) == nullptr);
    CHECK(warnCount(fx) == 1);
    fx.pipeline.stop();
}

TEST_CASE("T6 negative limits record until stop", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(-5, -5));
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 0, 1, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
}

void sendAt(HostRecord& fx, int n, const std::chrono::steady_clock::time_point& origin, int offsetMs) {
    std::this_thread::sleep_until(origin + std::chrono::milliseconds(offsetMs));
    sendRawWrote(fx, n);
}

TEST_CASE("T7 a duration limit closes the segment", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(-1, 1000));
    auto started = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    const auto origin = started->getTimestamp();
    sendAt(fx, 10, origin, 200);
    sendAt(fx, 11, origin, 600);
    sendAt(fx, 12, origin, 900);
    auto stopped = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 10, 12, 0);
    const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(stopped->getTimestamp() - started->getTimestamp()).count();
    CHECK(span >= 1000);
    CHECK(span <= 1100);
    sendRaw(fx, 13, 13);
    idle(fx, 13);
    expectFiles(fx.streamDir(), {"cam_00000010.dai", "cam_00000011.dai", "cam_00000012.dai"});
    fx.pipeline.stop();
}

TEST_CASE("T8 a duration with no frames still stops", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(-1, 1000));
    auto started = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    auto stopped = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 0, -1, -1, 0);
    const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(stopped->getTimestamp() - started->getTimestamp()).count();
    CHECK(span >= 1000);
    CHECK(span <= 1100);
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    fx.pipeline.stop();
}

TEST_CASE("T9 the frame limit wins over the duration", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(2, 1000));
    auto started = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    const auto origin = started->getTimestamp();
    sendAt(fx, 0, origin, 200);
    sendAt(fx, 1, origin, 600);
    auto stopped = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 0, 1, 0);
    const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(stopped->getTimestamp() - started->getTimestamp()).count();
    CHECK(span < 900);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
    fx.pipeline.stop();
}

TEST_CASE("T10 the duration wins when the frame limit is higher", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(10, 1000));
    auto started = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    const auto origin = started->getTimestamp();
    sendAt(fx, 0, origin, 200);
    sendAt(fx, 1, origin, 600);
    sendAt(fx, 2, origin, 900);
    auto stopped = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 0, 2, 0);
    const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(stopped->getTimestamp() - started->getTimestamp()).count();
    CHECK(span >= 1000);
    CHECK(span <= 1100);
    sendRaw(fx, 3, 3);
    idle(fx, 3);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai", "cam_00000002.dai"});
    fx.pipeline.stop();
}

TEST_CASE("T11 stop may cut a burst", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    for(int n = 0; n < 5; ++n) sendFrame(fx.frames, makeImg(64, 48, n));
    sendControl(fx, record_test::stopControl());
    std::shared_ptr<dai::RecordStatus> stopped;
    const auto deadline = std::chrono::steady_clock::now() + kWait;
    while(std::chrono::steady_clock::now() < deadline) {
        auto status = record_test::getFor<dai::RecordStatus>(*fx.status, std::chrono::milliseconds(200));
        if(status && status->state == dai::RecordStatus::State::STOPPED) {
            stopped = status;
            break;
        }
    }
    REQUIRE(stopped != nullptr);
    const int k = stopped->frameCount;
    CHECK(k >= 0);
    CHECK(k <= 5);
    if(k == 0) {
        CHECK(stopped->firstSequenceNum == -1);
        CHECK(stopped->lastSequenceNum == -1);
    } else {
        CHECK(stopped->firstSequenceNum == 0);
        CHECK(stopped->lastSequenceNum == k - 1);
    }
    CHECK(stopped->skippedFrames == 0);
    fx.pipeline.stop();
    std::vector<std::string> names;
    for(int n = 0; n < k; ++n) names.push_back(record_test::frameFileName("cam", n));
    expectFiles(fx.streamDir(), names);
}

TEST_CASE("T12 three controls queue in order", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    sendControl(fx, record_test::stopControl());
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 0, -1, -1, 0);
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 1, 2, 0, 1, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
}

TEST_CASE("T13 pipeline stop publishes STOPPED", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    auto stopped = stoppedOf(fx, 0, 2, 0, 1, 0);
    auto again = record_test::getStatus(*fx.node);
    REQUIRE(again != nullptr);
    CHECK(again->state == stopped->state);
    CHECK(again->frameCount == 2);
}

TEST_CASE("T14 idle pre-trigger frames are dropped on pipeline stop", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 2);
    fx.pipeline.start();
    for(int n = 0; n < 4; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    fx.pipeline.stop();
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    CHECK(record_test::getStatus(*fx.node) == nullptr);
}

TEST_CASE("T15 start then stop with no frames", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 0, -1, -1, 0);
    CHECK(record_test::fileCount(fx.streamDir()) == 0);
    fx.pipeline.stop();
}

TEST_CASE("T16 a repeated sequence during a trigger is an error", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendRaw(fx, 2, 1);
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 2, 0, 1, 0);
    CHECK(fx.status->tryGet<dai::RecordStatus>() == nullptr);
    auto latest = record_test::getStatus(*fx.node);
    REQUIRE(latest != nullptr);
    CHECK(latest->state == dai::RecordStatus::State::ERROR);
    CHECK(fx.log.countSubstring("Record sequence went back (1 -> 1)") == 1);
    fx.pipeline.stop();
}

TEST_CASE("T17 status sequence numbers increase", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    for(int n = 0; n < 3; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    auto s0 = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    for(int n = 3; n <= 5; ++n) sendRawWrote(fx, n);
    sendControl(fx, record_test::stopControl());
    auto s1 = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 3, 5, 0);
    for(int n = 6; n <= 7; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    auto s2 = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendRawWrote(fx, 8);
    sendRawWrote(fx, 9);
    sendControl(fx, record_test::stopControl());
    auto s3 = expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 1, 2, 8, 9, 0);
    CHECK(s0->getSequenceNum() == 0);
    CHECK(s1->getSequenceNum() == 1);
    CHECK(s2->getSequenceNum() == 2);
    CHECK(s3->getSequenceNum() == 3);
    fx.pipeline.stop();
}

TEST_CASE("T18 start is ignored when the trigger is off", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, false);
    fx.control();
    beginRecord(fx);
    sendControl(fx, record_test::startControl(1, -1));
    REQUIRE(fx.log.waitFor("Record start ignored", kWait));
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendRawWrote(fx, 2);
    stoppedOf(fx, 0, 3, 0, 2, 0);
    CHECK(warnCount(fx, "Record start ignored") == 1);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai", "cam_00000002.dai"});
}

TEST_CASE("T19 a duration of zero is ignored", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl(-1, 0));
    REQUIRE(fx.log.waitFor("Record limit of 0 ignored", kWait));
    bool timedOut = false;
    CHECK(fx.status->get<dai::RecordStatus>(std::chrono::milliseconds(300), timedOut) == nullptr);
    CHECK(warnCount(fx, "Record limit of 0 ignored") == 1);
    fx.pipeline.stop();
}

TEST_CASE("P1 pre-trigger keeps the newest frames", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 2);
    fx.pipeline.start();
    for(int n = 0; n <= 4; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 5);
    sendRawWrote(fx, 6);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 4, 3, 6, 0);
    for(int n = 7; n <= 9; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl(3, -1));
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendRawWrote(fx, 10);
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 1, 3, 8, 10, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(),
                {"cam_00000003.dai", "cam_00000004.dai", "cam_00000005.dai", "cam_00000006.dai", "cam_00000008.dai", "cam_00000009.dai", "cam_00000010.dai"});
}

TEST_CASE("P2 a frame limit consumes the pre-trigger buffer", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 5);
    fx.pipeline.start();
    for(int n = 0; n <= 6; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl(2, -1));
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 5, 6, 0);
    sendRaw(fx, 7, 7);
    idle(fx, 7);
    CHECK(record_test::fileCount(fx.streamDir()) == 2);
    expectFiles(fx.streamDir(), {"cam_00000005.dai", "cam_00000006.dai"});
    fx.pipeline.stop();
}

TEST_CASE("P3 the limit stops before the next frame", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 2);
    fx.pipeline.start();
    for(int n = 0; n <= 4; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl(3, -1));
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 5);
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 3, 5, 0);
    sendRaw(fx, 6, 6);
    idle(fx, 6);
    expectFiles(fx.streamDir(), {"cam_00000003.dai", "cam_00000004.dai", "cam_00000005.dai"});
    fx.pipeline.stop();
}

TEST_CASE("P4 a zero pre-trigger buffer keeps nothing", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    for(int n = 0; n <= 4; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 5);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 1, 5, 5, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(), {"cam_00000005.dai"});
}

TEST_CASE("P4b the default buffer keeps a short burst", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, true);
    fx.control();
    fx.pipeline.start();
    for(int n = 0; n <= 4; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 5);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 6, 0, 5, 0);
    fx.pipeline.stop();
    CHECK(record_test::fileCount(fx.streamDir()) == 6);
}

TEST_CASE("P4c the default buffer keeps the newest sixty", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, true);
    fx.control();
    fx.pipeline.start();
    for(int n = 0; n <= 64; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 60, 5, 64, 0);
    fx.pipeline.stop();
    CHECK(record_test::fileCount(fx.streamDir()) == 60);
    CHECK(std::filesystem::exists(fx.streamDir() / "cam_00000005.dai"));
    CHECK(std::filesystem::exists(fx.streamDir() / "cam_00000064.dai"));
    CHECK_FALSE(std::filesystem::exists(fx.streamDir() / "cam_00000004.dai"));
}

TEST_CASE("P5 start with an empty pre-trigger buffer", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 2);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRawWrote(fx, 0);
    sendRawWrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 0, 1, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
}

TEST_CASE("P6 one huge frame overflows the default byte cap", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 100);
    fx.pipeline.start();
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    auto err = expectStatus(fx.status, dai::RecordStatus::State::ERROR, -1, 0, -1, -1, 0);
    CHECK(err->error.find("1800000000") != std::string::npos);
    CHECK(err->error.find("268435456") != std::string::npos);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
}

TEST_CASE("P7 ten huge slots still fit the default cap", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 10);
    fx.pipeline.start();
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    idle(fx, 0);
    bool timedOut = false;
    CHECK(fx.status->get<dai::RecordStatus>(std::chrono::milliseconds(300), timedOut) == nullptr);
    fx.pipeline.stop();
}

TEST_CASE("P7b the default sixty slots overflow", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, true);
    fx.pipeline.start();
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    auto err = expectStatus(fx.status, dai::RecordStatus::State::ERROR, -1, 0, -1, -1, 0);
    CHECK(err->error.find("1080000000") != std::string::npos);
    CHECK(err->error.find("268435456") != std::string::npos);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
}

TEST_CASE("P8 a custom byte cap rejects the buffer", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 3);
    record_test::setMaxPreTriggerBytes(*fx.node, 50000000);
    fx.pipeline.start();
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    auto err = expectStatus(fx.status, dai::RecordStatus::State::ERROR, -1, 0, -1, -1, 0);
    CHECK(err->error.find("54000000") != std::string::npos);
    CHECK(err->error.find("50000000") != std::string::npos);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
}

TEST_CASE("P9 a byte cap that fits does not error", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 3);
    record_test::setMaxPreTriggerBytes(*fx.node, 60000000);
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 1));
    wrote(fx, 1);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 0, 1, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 2));
    idle(fx, 2);
    CHECK(fx.log.count(record_test::LineLevel::ERROR, "") == 0);
    expectFiles(fx.streamDir(), {"cam_00000000.dai", "cam_00000001.dai"});
    fx.pipeline.stop();
}

TEST_CASE("P10 the pre-trigger byte cap drops the oldest encoded frames", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    armTrigger(fx, 3);
    record_test::setMaxPreTriggerBytes(*fx.node, 40000000);
    fx.pipeline.start();
    sendFrame(fx.frames, makeEncPad(0, 10000000));
    idle(fx, 0);
    sendFrame(fx.frames, makeEncPad(1, 18000000));
    idle(fx, 1);
    sendFrame(fx.frames, makeEncPad(2, 18000000));
    idle(fx, 2);
    sendFrame(fx.frames, makeEncPad(3, 18000000));
    idle(fx, 3);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 2, 2, 3, 0);
    CHECK(warnCount(fx, "Record pre-trigger buffer full") == 1);
    expectFiles(fx.streamDir(), {"cam_00000002.dai", "cam_00000003.dai"});
    fx.pipeline.stop();
}

TEST_CASE("P11 pre-trigger gaps are skipped frames", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 2);
    fx.pipeline.start();
    sendRaw(fx, 0, 0);
    idle(fx, 0);
    sendRaw(fx, 1, 1);
    idle(fx, 1);
    sendRaw(fx, 5, 5);
    idle(fx, 5);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendRaw(fx, 9, 9);
    wrote(fx, 9);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 1, 9, 6);
    CHECK(warnCount(fx, "Record skipped") == 2);
    expectFiles(fx.streamDir(), {"cam_00000001.dai", "cam_00000005.dai", "cam_00000009.dai"});
    fx.pipeline.stop();
}

TEST_CASE("P12 one pre-trigger slot keeps the last idle frame", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 1);
    fx.pipeline.start();
    for(int n = 0; n < 3; ++n) {
        sendRaw(fx, n, n);
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 1, 2, 2, 0);
    fx.pipeline.stop();
    expectFiles(fx.streamDir(), {"cam_00000002.dai"});
}

TEST_CASE("P13 the pre-trigger flush does not skip frames", "[record][host]") {
    HostRecord fx;
    record_test::setWaitForTrigger(*fx.node, true);
    record_test::setRunOnHost(*fx.node, true);
    fx.control();
    fx.pipeline.start();
    for(int n = 0; n < 60; ++n) {
        sendFrame(fx.frames, makeImg(640, 480, n));
        idle(fx, n);
    }
    sendControl(fx, record_test::startControl());
    auto started = expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    const auto origin = std::chrono::steady_clock::now();
    std::thread sender([&] {
        std::vector<std::shared_ptr<dai::ADatatype>> frames;
        frames.reserve(300);
        for(int n = 60; n < 360; ++n) frames.push_back(makeImg(640, 480, n));
        record_test::sendPaced(fx.frames, frames, 33, &fx.pipeline);
    });
    REQUIRE(fx.log.waitFor("Record wrote frame 60", std::chrono::seconds(1)));
    CHECK(std::chrono::steady_clock::now() - origin < std::chrono::seconds(1));
    sender.join();
    REQUIRE(fx.log.waitFor("Record wrote frame 359", std::chrono::seconds(20)));
    (void)started;
    stoppedOf(fx, 0, 360, 0, 359, 0);
    CHECK(record_test::fileCount(fx.streamDir()) == 360);
}

TEST_CASE("K1 gaps warn and add to skippedFrames", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    for(int seq : {0, 1, 5, 6, 10}) {
        sendRaw(fx, seq, seq);
        wrote(fx, seq);
    }
    stoppedOf(fx, 0, 5, 0, 10, 6);
    CHECK(warnCount(fx, "Record skipped 3 frames (1 -> 5)") == 1);
    CHECK(warnCount(fx, "Record skipped 3 frames (6 -> 10)") == 1);
    CHECK(record_test::fileCount(fx.streamDir()) == 5);
}

TEST_CASE("K2 a gap after the trigger is one warning", "[record][host]") {
    HostRecord fx;
    armTrigger(fx, 0);
    fx.pipeline.start();
    sendRaw(fx, 0, 0);
    idle(fx, 0);
    sendRaw(fx, 1, 1);
    idle(fx, 1);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    for(int seq : {5, 6, 10}) {
        sendRaw(fx, seq, seq);
        wrote(fx, seq);
    }
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 3, 5, 10, 3);
    CHECK(warnCount(fx, "6 -> 10") == 1);
    fx.pipeline.stop();
}

TEST_CASE("K3 a backward sequence is not a second skip", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    for(int seq : {0, 1, 5, 2, 3}) {
        sendRaw(fx, seq, seq);
        wrote(fx, seq);
    }
    stoppedOf(fx, 0, 5, 0, 3, 3);
    CHECK(warnCount(fx, "Record skipped 3 frames (1 -> 5)") == 1);
    CHECK(warnCount(fx, "Record sequence went back (5 -> 2)") == 1);
    CHECK(std::filesystem::exists(fx.streamDir() / "cam_00000002.dai"));
}

TEST_CASE("K4 a one-frame gap", "[record][host]") {
    HostRecord fx;
    beginRecord(fx);
    sendRawWrote(fx, 0);
    sendRaw(fx, 2, 2);
    wrote(fx, 2);
    stoppedOf(fx, 0, 2, 0, 2, 1);
    CHECK(warnCount(fx, "Record skipped 1 frames (0 -> 2)") == 1);
}

TEST_CASE("K5 a 12 megapixel frame warns at 30 fps", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    wrote(fx, 0);
    CHECK(warnCount(fx, "Record expected write rate 540 MB/s") == 1);
    fx.pipeline.stop();
}

TEST_CASE("K6 vga at 30 fps has no rate warning", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    beginRecord(fx);
    sendFrame(fx.frames, makeImg(640, 480, 0));
    wrote(fx, 0);
    CHECK(warnCount(fx) == 0);
    fx.pipeline.stop();
}

TEST_CASE("K7 1080p at 30 fps has no rate warning", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    beginRecord(fx);
    sendFrame(fx.frames, makeImg(1920, 1080, 0));
    wrote(fx, 0);
    CHECK(warnCount(fx) == 0);
    fx.pipeline.stop();
}

TEST_CASE("K8 1080p at 60 fps warns", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 60.f);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(1920, 1080, 0));
    wrote(fx, 0);
    CHECK(warnCount(fx, "187 MB/s") == 1);
    fx.pipeline.stop();
}

TEST_CASE("K9 each segment warns once", "[record][host]") {
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    record_test::setWaitForTrigger(*fx.node, true);
    record_test::setPreTriggerFrames(*fx.node, 0);
    fx.control();
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 0));
    wrote(fx, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 1, 0, 0, 0);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendFrame(fx.frames, makeImg(4000, 3000, 1));
    wrote(fx, 1);
    CHECK(warnCount(fx, "expected write rate") == 2);
    fx.pipeline.stop();
}

TEST_CASE("K10 a small encoded frame has no rate warning", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    beginRecord(fx);
    sendFrame(fx.frames, makeEncPad(0, 500000));
    wrote(fx, 0);
    CHECK(fx.log.countSubstring("expected write rate") == 0);
    CHECK(warnCount(fx, "H264") == 1);
    fx.pipeline.stop();
}

TEST_CASE("K11 just under the rate cap does not warn", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    beginRecord(fx);
    sendFrame(fx.frames, makeEncPad(0, 3333333));
    wrote(fx, 0);
    CHECK(fx.log.countSubstring("expected write rate") == 0);
    CHECK(warnCount(fx, "H264") == 1);
    fx.pipeline.stop();
}

TEST_CASE("K12 one byte over the rate cap warns", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    record_test::setFps(*fx.node, 30.f);
    beginRecord(fx);
    sendFrame(fx.frames, makeEncPad(0, 3333334));
    wrote(fx, 0);
    CHECK(warnCount(fx, "Record expected write rate 100 MB/s") == 1);
    fx.pipeline.stop();
}

void beginVideo(HostRecord& fx, float fps) {
    record_test::setFormat(*fx.node, record_test::Format::VIDEO);
    record_test::setStreamName(*fx.node, "enc");
    record_test::setFps(*fx.node, fps);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
}

TEST_CASE("V1 twenty encoded frames become one mp4", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 25.f);
    std::vector<std::shared_ptr<dai::ADatatype>> frames;
    for(int n = 0; n < 20; ++n) frames.push_back(makeEnc(n));
    record_test::sendPaced(fx.frames, frames, 20, &fx.pipeline);
    REQUIRE(fx.log.waitFor("Record wrote frame 19", kWait));
    stoppedOf(fx, 0, 20, 0, 19, 0);
    expectFiles(fx.recDir(), {"enc.mcap", "enc.mp4"});
    CHECK(record_test::countVideoFrames(fx.recDir() / "enc.mp4", fx.recDir() / "enc.mcap") == 20);
    const auto messages = record_test::readMcap(fx.recDir() / "enc.mcap");
    REQUIRE(messages.size() == 20);
    for(int n = 0; n < 20; ++n) {
        CHECK(messages[static_cast<std::size_t>(n)].sequenceNum == n);
        CHECK(messages[static_cast<std::size_t>(n)].tsMs == 100 * n);
    }
    CHECK(record_test::videoDurationS(fx.recDir() / "enc.mp4") == Catch::Approx(0.8).margin(0.04));
}

TEST_CASE("V2 video segments and an empty third segment", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    record_test::setFormat(*fx.node, record_test::Format::VIDEO);
    record_test::setStreamName(*fx.node, "enc");
    record_test::setWaitForTrigger(*fx.node, true);
    record_test::setPreTriggerFrames(*fx.node, 0);
    fx.control();
    fx.pipeline.start();
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    for(int n = 0; n <= 3; ++n) {
        sendFrame(fx.frames, makeEnc(n));
        wrote(fx, n);
    }
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 0, 4, 0, 3, 0);
    sendFrame(fx.frames, makeEnc(4));
    idle(fx, 4);
    sendFrame(fx.frames, makeEnc(5));
    idle(fx, 5);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 1, 0, -1, -1, 0);
    sendFrame(fx.frames, makeEnc(6));
    wrote(fx, 6);
    sendFrame(fx.frames, makeEnc(7));
    wrote(fx, 7);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 1, 2, 6, 7, 0);
    sendControl(fx, record_test::startControl());
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 2, 0, -1, -1, 0);
    sendControl(fx, record_test::stopControl());
    expectStatus(fx.status, dai::RecordStatus::State::STOPPED, 2, 0, -1, -1, 0);
    fx.pipeline.stop();
    expectFiles(fx.recDir(), {"enc.mcap", "enc.mp4", "enc_s1.mcap", "enc_s1.mp4"});
    CHECK(record_test::countVideoFrames(fx.recDir() / "enc.mp4", fx.recDir() / "enc.mcap") == 4);
    CHECK(record_test::countVideoFrames(fx.recDir() / "enc_s1.mp4", fx.recDir() / "enc_s1.mcap") == 2);
}

TEST_CASE("V3 hevc is rejected in video mode", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 30.f);
    sendFrame(fx.frames, makeEnc265(0));
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 0, -1, -1, 0);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
    CHECK(record_test::allFiles(fx.recDir()).empty());
}

TEST_CASE("V4 a profile change is an error", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 30.f);
    sendFrame(fx.frames, makeEnc(0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeEncJ(1));
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 1, 0, 0, 0);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
}

TEST_CASE("V5 a size change is an error", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 30.f);
    sendFrame(fx.frames, makeEnc(0));
    wrote(fx, 0);
    auto second = makeEnc(1);
    second->setWidth(128);
    sendFrame(fx.frames, second);
    expectStatus(fx.status, dai::RecordStatus::State::ERROR, 0, 1, 0, 0, 0);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, kWait));
}

TEST_CASE("V6 raw frames in video mode become an avi", "[record][host]") {
    HostRecord fx;
    beginVideo(fx, 30.f);
    sendFrame(fx.frames, makeImg(64, 48, 0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeImg(64, 48, 1));
    wrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    expectFiles(fx.recDir(), {"enc.avi", "enc.mcap"});
    CHECK(record_test::countVideoFrames(fx.recDir() / "enc.avi", fx.recDir() / "enc.mcap") == 2);
}

TEST_CASE("V7 mjpeg frames become an mp4", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 30.f);
    sendFrame(fx.frames, makeEncJ(0));
    wrote(fx, 0);
    sendFrame(fx.frames, makeEncJ(1));
    wrote(fx, 1);
    stoppedOf(fx, 0, 2, 0, 1, 0);
    expectFiles(fx.recDir(), {"enc.mcap", "enc.mp4"});
    CHECK(record_test::countVideoFrames(fx.recDir() / "enc.mp4", fx.recDir() / "enc.mcap") == 2);
}

TEST_CASE("V8 compression none writes an uncompressed mcap", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    record_test::setCompressionLevel(*fx.node, record_test::Compression::NONE);
    beginVideo(fx, 25.f);
    std::vector<std::shared_ptr<dai::ADatatype>> frames;
    for(int n = 0; n < 20; ++n) frames.push_back(makeEnc(n));
    record_test::sendPaced(fx.frames, frames, 20, &fx.pipeline);
    REQUIRE(fx.log.waitFor("Record wrote frame 19", kWait));
    stoppedOf(fx, 0, 20, 0, 19, 0);
    CHECK(record_test::mcapCompression(fx.recDir() / "enc.mcap") == "");
}

TEST_CASE("V9 fps sets the video duration", "[record][host]") {
    record_test::requireRecordData();
    HostRecord fx;
    beginVideo(fx, 50.f);
    std::vector<std::shared_ptr<dai::ADatatype>> frames;
    for(int n = 0; n < 20; ++n) frames.push_back(makeEnc(n));
    record_test::sendPaced(fx.frames, frames, 20, &fx.pipeline);
    REQUIRE(fx.log.waitFor("Record wrote frame 19", kWait));
    stoppedOf(fx, 0, 20, 0, 19, 0);
    CHECK(record_test::videoDurationS(fx.recDir() / "enc.mp4") == Catch::Approx(0.4).margin(0.02));
}

TEST_CASE("H1 a full filesystem stops the recording", "[record][host]") {
    if(geteuid() != 0) SKIP("needs root for a 1 MiB tmpfs");
    HostRecord fx;
    const auto mountPoint = fx.dir.path / "fs";
    std::filesystem::create_directories(mountPoint);
    const std::string mountCmd = "mount -t tmpfs -o size=1m tmpfs " + mountPoint.string();
    REQUIRE(std::system(mountCmd.c_str()) == 0);
    struct Unmount {
        std::filesystem::path path;
        ~Unmount() {
            const std::string command = "umount " + path.string();
            std::system(command.c_str());
        }
    } unmount{mountPoint};
    record_test::setOutputDirectory(*fx.node, mountPoint);
    fx.pipeline.start();
    expectStatus(fx.status, dai::RecordStatus::State::STARTED, 0, 0, -1, -1, 0);
    record_test::sendPacedUntilStop(fx.frames, [](int i) { return makeImg(640, 480, i); }, 100, &fx.pipeline);
    REQUIRE(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, std::chrono::seconds(30)));
    auto err = record_test::getStatus(*fx.node);
    REQUIRE(err != nullptr);
    CHECK(err->state == dai::RecordStatus::State::ERROR);
    CHECK(err->frameCount == 2);
    CHECK(err->firstSequenceNum == 0);
    CHECK(err->lastSequenceNum == 1);
    CHECK(fx.log.count(record_test::LineLevel::ERROR, "") >= 1);
    const auto stream = mountPoint / "test" / "cam";
    expectFiles(stream, {"cam_00000000.dai", "cam_00000001.dai"});
    const auto first = record_test::loadImg(stream / "cam_00000000.dai")->getData();
    const auto second = record_test::loadImg(stream / "cam_00000001.dai")->getData();
    const auto sent0 = makeImg(640, 480, 0)->getData();
    const auto sent1 = makeImg(640, 480, 1)->getData();
    CHECK(first.size() == sent0.size());
    CHECK(second.size() == sent1.size());
    CHECK(std::equal(first.begin(), first.end(), sent0.begin()));
    CHECK(std::equal(second.begin(), second.end(), sent1.begin()));
}

TEST_CASE("H2 a short status queue still delivers the last status", "[record][host]") {
    record_test::TempDir dir;
    record_test::LogCapture log;
    dai::Pipeline pipeline(false);
    auto node = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*node);
    record_test::setFormat(*node, record_test::Format::RAW_FRAMES);
    record_test::setRecordingName(*node, "test");
    record_test::setOutputDirectory(*node, dir.path);
    record_test::setStreamName(*node, "cam");
    record_test::setFps(*node, 30u);
    auto frames = node->input.createInputQueue();
    auto status = node->status.createOutputQueue(1, false);
    pipeline.start();
    for(int n = 0; n < 5; ++n) {
        sendFrame(frames, makeImg(64, 48, n));
        REQUIRE(log.waitFor("Record wrote frame " + std::to_string(n), kWait));
    }
    pipeline.stop();
    // stop() closes the status queue. A closed queue makes tryGet throw, so fall back to getStatus.
    std::shared_ptr<dai::RecordStatus> last;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(std::chrono::steady_clock::now() < deadline) {
        try {
            while(auto message = status->tryGet<dai::RecordStatus>()) last = message;
        } catch(const std::exception&) {
            break;
        }
        if(last && last->state == dai::RecordStatus::State::STOPPED) break;
        auto latest = record_test::getStatus(*node);
        if(latest && latest->state == dai::RecordStatus::State::STOPPED) {
            last = latest;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if(!last) last = record_test::getStatus(*node);
    REQUIRE(last != nullptr);
    CHECK(last->state == dai::RecordStatus::State::STOPPED);
    CHECK(last->frameCount == 5);
    CHECK(last->firstSequenceNum == 0);
    CHECK(last->lastSequenceNum == 4);
    CHECK(last->skippedFrames == 0);

    HostRecord blocking;
    beginRecord(blocking);
    for(int n = 0; n < 5; ++n) sendRawWrote(blocking, n);
    auto started = record_test::getStatus(*blocking.node);
    REQUIRE(started != nullptr);
    CHECK(started->state == dai::RecordStatus::State::STARTED);
    stoppedOf(blocking, 0, 5, 0, 4, 0);
}
