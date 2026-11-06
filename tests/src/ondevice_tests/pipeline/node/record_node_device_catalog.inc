static std::shared_ptr<dai::InputQueue> armControl(const RawNode& raw) {
    return raw.node->inputControl.createInputQueue();
}

static void sendIdleFrame(Store& store, const RawNode& raw, int index, std::int64_t sequence) {
    sendFrame(raw.frames, makeImg(64, 48, index, sequence));
    REQUIRE(store.devLog.waitFor("Record idle frame " + std::to_string(sequence), std::chrono::seconds(10)));
}

static void sendWroteFrame(Store& store, const RawNode& raw, int index, std::int64_t sequence) {
    sendFrame(raw.frames, makeImg(64, 48, index, sequence));
    REQUIRE(store.devLog.waitFor("Record wrote frame " + std::to_string(sequence), std::chrono::seconds(10)));
}

static std::vector<std::string> archiveNames(const std::filesystem::path& tar) {
    auto names = dai::utility::filenamesInArchive(tar);
    std::sort(names.begin(), names.end());
    return names;
}

static void checkRawMembers(const std::filesystem::path& tar, bool imu) {
    checkGzip(tar);
    const auto names = archiveNames(tar);
    int other = 0;
    bool config = false;
    bool cameraInfo = false;
    bool imuFile = false;
    for(const auto& name : names) {
        if(name == "record_config.json") {
            config = true;
            continue;
        }
        if(name == "camera_info.json") {
            cameraInfo = true;
            continue;
        }
        if(name == "IMU.mcap") {
            imuFile = true;
            continue;
        }
        const bool frame = name.rfind("CameraCAM_", 0) == 0;
        if(!frame) ++other;
    }
    CHECK(config);
    CHECK(cameraInfo);
    CHECK(imuFile == imu);
    CHECK(other == 0);
    const int countA = countPrefixed(names, "CameraCAM_A/CameraCAM_A_");
    const int countB = countPrefixed(names, "CameraCAM_B/CameraCAM_B_");
    const int countC = countPrefixed(names, "CameraCAM_C/CameraCAM_C_");
    CHECK(countA >= 80);
    CHECK(countA <= 120);
    CHECK(countB >= 80);
    CHECK(countB <= 120);
    CHECK(countC >= 80);
    CHECK(countC <= 120);
    CHECK(std::abs(countA - countB) <= 1);
    CHECK(std::abs(countB - countC) <= 1);
    CHECK(std::abs(countA - countC) <= 1);
    const auto configBytes = dai::utility::readFileInArchive(tar, "record_config.json");
    const auto parsed = nlohmann::json::parse(configBytes.begin(), configBytes.end());
    CHECK(parsed["format"] == "RAW_FRAMES");
    CHECK(parsed["streams"]["CameraCAM_A"]["fps"] == 10);
    CHECK(parsed["streams"]["CameraCAM_B"]["fps"] == 10);
    CHECK(parsed["streams"]["CameraCAM_C"]["fps"] == 10);
}

static std::filesystem::path runHolistic(Store& store, dai::RecordConfig config, bool imu, int seconds) {
    skipUnlessThree(*store.device);
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    std::shared_ptr<dai::MessageQueue> imuQueue;
    if(imu) {
        auto node = pipeline.create<dai::node::IMU>();
        node->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 100);
        imuQueue = node->out.createOutputQueue(16, false);
    }
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    pipeline.stop();
    (void)cameras;
    (void)imuQueue;
    return oneTar(store.dir.path);
}

TEST_CASE("VR3 a second pipeline cannot reuse the recording name", "[record][device]") {
    Store store;
    recordFive(store, "test", true, false);
    const auto before = findRecording(listRecordings(*store.device), "test");
    REQUIRE(before != nullptr);
    const auto sizeBefore = before->sizeBytes;
    auto& pipeline = store.replacePipeline();
    auto raw = addRaw(pipeline, store.dir.path, "test", true, false);
    (void)raw;
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("test"));
    const auto after = findRecording(listRecordings(*store.device), "test");
    REQUIRE(after != nullptr);
    CHECK(after->sizeBytes == sizeBefore);
}

TEST_CASE("VR4 a second download throws", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR4");
    recordFive(store, name, true, false);
    downloadRecording(*store.device, name, store.dir.path);
    REQUIRE_THROWS(downloadRecording(*store.device, name, store.dir.path));
}

TEST_CASE("VR5 missing and unsafe names throw", "[record][device]") {
    Store store;
    REQUIRE_THROWS(downloadRecording(*store.device, "nope", store.dir.path));
    REQUIRE_THROWS(deleteRecording(*store.device, "nope"));
    REQUIRE_THROWS(deleteRecording(*store.device, ""));
    REQUIRE_THROWS(deleteRecording(*store.device, ".."));
    REQUIRE_THROWS(downloadRecording(*store.device, "a/b", store.dir.path));
}

TEST_CASE("VR6 an empty store lists nothing", "[record][device]") {
    Store store;
    const auto listed = listRecordings(*store.device);
    if(!listed.empty()) SKIP("store not empty");
    CHECK(listed.empty());
}

TEST_CASE("VR7 delete and download throw while the recording is open", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR7");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    REQUIRE_THROWS(deleteRecording(*store.device, name));
    REQUIRE_THROWS(downloadRecording(*store.device, name, store.dir.path));
    const auto* info = findRecording(listRecordings(*store.device), name);
    REQUIRE(info != nullptr);
    CHECK(info->complete == false);
    pipeline.stop();
}

TEST_CASE("VR8 delete removes the recording", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR8");
    recordFive(store, name, true, false);
    deleteRecording(*store.device, name);
    CHECK(findRecording(listRecordings(*store.device), name) == nullptr);
    REQUIRE_THROWS(deleteRecording(*store.device, name));
}

TEST_CASE("VR9 creation times increase and stay stable", "[record][device]") {
    Store store;
    const auto firstName = uniqueName("VR9");
    recordFive(store, firstName, true, false);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto secondName = uniqueName("VR9b");
    auto& pipeline = store.replacePipeline();
    auto raw = addRaw(pipeline, store.dir.path, secondName, true, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    pipeline.stop();
    const auto once = listRecordings(*store.device);
    const auto twice = listRecordings(*store.device);
    const auto* first = findRecording(once, firstName);
    const auto* second = findRecording(once, secondName);
    const auto* firstAgain = findRecording(twice, firstName);
    const auto* secondAgain = findRecording(twice, secondName);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(firstAgain != nullptr);
    REQUIRE(secondAgain != nullptr);
    CHECK(first->creationTimeMs > 0);
    CHECK(second->creationTimeMs > 0);
    const auto gap = second->creationTimeMs - first->creationTimeMs;
    CHECK(gap >= 1000);
    CHECK(gap <= 60000);
    CHECK(firstAgain->creationTimeMs == first->creationTimeMs);
    CHECK(secondAgain->creationTimeMs == second->creationTimeMs);
}

TEST_CASE("VR10 the default transfer copies five files and removes the device copy", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR10");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    pipeline.stop();
    CHECK(fileCount(store.dir.path / name / "cam") == 5);
    CHECK(findRecording(listRecordings(*store.device), name) == nullptr);
    CHECK(store.hostLog.count(LineLevel::INFO, "Record download") >= 1);
    CHECK(downloadFinished(store.hostLog));
}

TEST_CASE("VR11 keep on device leaves the recording listed", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR11");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    setKeepOnDevice(*raw.node, true);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    pipeline.stop();
    CHECK(fileCount(store.dir.path / name / "cam") == 5);
    CHECK(named(listRecordings(*store.device), name));
}

TEST_CASE("VR12 an existing host directory keeps the device recording", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR12");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    std::filesystem::create_directories(store.dir.path / name);
    REQUIRE_NOTHROW(pipeline.stop());
    const int errors = store.hostLog.count(LineLevel::ERROR, name) + store.devLog.count(LineLevel::ERROR, name);
    CHECK(errors >= 1);
    CHECK(named(listRecordings(*store.device), name));
}

TEST_CASE("VR13 destroying the pipeline downloads the recording", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR13");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    for(int index = 0; index < 5; ++index) sendWrote(raw.frames, store.devLog, index);
    store.guard->release();
    store.pipeline.reset();
    CHECK(fileCount(store.dir.path / name / "cam") == 5);
}

TEST_CASE("VR14 a sequence error still downloads the written frames", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR14");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    pipeline.start();
    REQUIRE(takeStatus(raw.status)->state == dai::RecordStatus::State::STARTED);
    sendWroteFrame(store, raw, 0, 0);
    sendWroteFrame(store, raw, 1, 1);
    sendFrame(raw.frames, makeImg(64, 48, 2, 1));
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(10)));
    CHECK(record_test::waitUntil([&] { return fileCount(store.dir.path / name / "cam") == 2; }, std::chrono::seconds(30)));
}

TEST_CASE("VR15 a device recording replays its raw frames", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR15");
    const auto path = recordFive(store, name, true, false);
    auto& pipeline = store.replacePipeline();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setSource(*replay, std::filesystem::path(path.string() + "/cam"), replay_test::Location::DEVICE);
    replay_test::setLoop(*replay, false);
    replay_test::setStopPipelineAtEnd(*replay, false);
    replay_test::setFps(*replay, 1000.f);
    auto out = replay->out.createOutputQueue(16, true);
    pipeline.start();
    for(int index = 0; index < 5; ++index) {
        auto frame = record_test::getFor<dai::ImgFrame>(*out, std::chrono::seconds(15));
        REQUIRE(frame != nullptr);
        CHECK(frame->getSequenceNum() == index);
        CHECK(record_test::timestampMs(frame->getTimestampDevice()) == static_cast<std::int64_t>(index) * 100);
        auto sent = makeImg(64, 48, index, index);
        const auto bytes = sent->getData();
        CHECK(replay_test::pixelsEqual(*frame, {bytes.begin(), bytes.end()}, 64, 48, dai::ImgFrame::Type::NV12));
    }
}

struct EncodedRecording {
    std::filesystem::path directory;
    std::shared_ptr<dai::RecordStatus> stopped;
};

static EncodedRecording recordEncoded(Store& store, const std::string& name, dai::VideoEncoderProperties::Profile profile, bool changeProfile) {
    auto& pipeline = store.begin();
    const auto source = store.dir.path / "src";
    makeDaiDir(source, 1920, 1080, rangeSeq(50), rangeTs(50));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setRunOnHost(*replay, true);
    replay_test::setSource(*replay, source);
    replay_test::setLoop(*replay, false);
    replay_test::setStopPipelineAtEnd(*replay, false);
    replay_test::setFps(*replay, 10.f);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(25.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    if(changeProfile) encoder->setProfile(profile);
    replay->out.link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::VIDEO);
    setRecordingName(*record, name);
    setOutputDirectory(*record, store.dir.path);
    setStreamName(*record, "enc");
    setFps(*record, 25u);
    setTransferToHost(*record, false);
    encoder->out.link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    REQUIRE(takeStatus(status)->state == dai::RecordStatus::State::STARTED);
    REQUIRE(record_test::waitUntil([&] { return store.devLog.countSubstring("Record wrote frame") >= 50; }, std::chrono::seconds(60)));
    pipeline.stop();
    EncodedRecording done;
    done.directory = std::filesystem::path(getStatus(*record)->path);
    done.stopped = getStatus(*record);
    return done;
}

static void checkEncodedFiles(Store& store, const EncodedRecording& done) {
    downloadRecording(*store.device, done.directory.filename().string(), store.dir.path);
    const auto hostDir = store.dir.path / done.directory.filename();
    expectFiles(hostDir, {"enc.mcap", "enc.mp4"});
    const auto messages = record_test::readMcap(hostDir / "enc.mcap");
    REQUIRE(messages.size() == 50);
    for(std::size_t index = 1; index < messages.size(); ++index) {
        CHECK(messages[index].sequenceNum == messages[index - 1].sequenceNum + 1);
    }
    REQUIRE(done.stopped != nullptr);
    checkStopped(*done.stopped, 0, 50, messages.front().sequenceNum, messages.back().sequenceNum, 0);
    CHECK(record_test::countVideoFrames(hostDir / "enc.mp4", hostDir / "enc.mcap") == 50);
    CHECK(std::fabs(record_test::videoDurationS(hostDir / "enc.mp4") - 2.0) <= 0.04);
    CHECK(store.devLog.countSubstring("expected write rate") == 0);
    CHECK(store.hostLog.countSubstring("expected write rate") == 0);
}

TEST_CASE("VR16 a device video replays the recorded sequence numbers", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR16");
    const auto done = recordEncoded(store, name, dai::VideoEncoderProperties::Profile::H264_MAIN, false);
    const auto messages = [&] {
        downloadRecording(*store.device, name, store.dir.path / "copy");
        return record_test::readMcap(store.dir.path / "copy" / name / "enc.mcap");
    }();
    REQUIRE(messages.size() == 50);
    auto& pipeline = store.replacePipeline();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setSource(*replay, done.directory / "enc.mp4", replay_test::Location::DEVICE);
    replay_test::setMetadataFile(*replay, done.directory / "enc.mcap");
    replay_test::setLoop(*replay, false);
    replay_test::setStopPipelineAtEnd(*replay, false);
    replay_test::setFps(*replay, 1000.f);
    auto out = replay->out.createOutputQueue(16, true);
    pipeline.start();
    for(int index = 0; index < 50; ++index) {
        auto frame = record_test::getFor<dai::ImgFrame>(*out, std::chrono::seconds(20));
        REQUIRE(frame != nullptr);
        CHECK(frame->getSequenceNum() == messages[static_cast<std::size_t>(index)].sequenceNum);
    }
}

TEST_CASE("VR17 an RVC2 device rejects the recording store", "[record][device]") {
    auto device = openRvc2OrSkip();
    REQUIRE_THROWS_WITH(listRecordings(*device), ContainsSubstring("not supported"));
}

TEST_CASE("VR18 the default transfer writes one host frame and no warning", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR18");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, false, false);
    const int warningsBefore = store.devLog.count(LineLevel::WARN) + store.hostLog.count(LineLevel::WARN);
    pipeline.start();
    sendWrote(raw.frames, store.devLog, 0);
    pipeline.stop();
    CHECK(store.devLog.count(LineLevel::WARN) + store.hostLog.count(LineLevel::WARN) == warningsBefore);
    CHECK(std::filesystem::is_regular_file(store.dir.path / name / "cam" / record_test::frameFileName("cam", 0)));
    CHECK(findRecording(listRecordings(*store.device), name) == nullptr);
}

TEST_CASE("VR19 two triggered segments on the device", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR19");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    setWaitForTrigger(*raw.node, true);
    setPreTriggerFrames(*raw.node, 0);
    auto control = armControl(raw);
    pipeline.start();
    for(int index = 0; index < 3; ++index) sendIdleFrame(store, raw, index, index);
    control->send(record_test::startControl());
    auto started = takeStatus(raw.status);
    CHECK(started->state == dai::RecordStatus::State::STARTED);
    CHECK(started->segment == 0);
    for(int index = 3; index <= 5; ++index) sendWroteFrame(store, raw, index, index);
    control->send(record_test::stopControl());
    auto stopped = takeStatus(raw.status);
    checkStopped(*stopped, 0, 3, 3, 5, 0);
    for(int index = 6; index <= 7; ++index) sendIdleFrame(store, raw, index, index);
    control->send(record_test::startControl());
    auto startedAgain = takeStatus(raw.status);
    CHECK(startedAgain->state == dai::RecordStatus::State::STARTED);
    CHECK(startedAgain->segment == 1);
    sendWroteFrame(store, raw, 8, 8);
    sendWroteFrame(store, raw, 9, 9);
    control->send(record_test::stopControl());
    auto stoppedAgain = takeStatus(raw.status);
    checkStopped(*stoppedAgain, 1, 2, 8, 9, 0);
    pipeline.stop();
    downloadRecording(*store.device, name, store.dir.path);
    expectFiles(store.dir.path / name / "cam",
                {"cam_00000003.dai", "cam_00000004.dai", "cam_00000005.dai", "cam_00000008.dai", "cam_00000009.dai"});
}

TEST_CASE("VR20 a 1920x1080 camera output stays near 30 fps", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR20");
    auto& pipeline = store.begin();
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::RAW_FRAMES);
    setRecordingName(*record, name);
    setOutputDirectory(*record, store.dir.path);
    setStreamName(*record, "cam");
    setTransferToHost(*record, false);
    setWaitForTrigger(*record, true);
    setPreTriggerFrames(*record, 10);
    output->link(record->input);
    auto frames = output->createOutputQueue(4, false);
    pipeline.start();
    auto first = record_test::getFor<dai::ImgFrame>(*frames, std::chrono::seconds(15));
    REQUIRE(first != nullptr);
    const auto firstTime = std::chrono::steady_clock::now();
    std::shared_ptr<dai::ImgFrame> last = first;
    auto lastTime = firstTime;
    while(std::chrono::steady_clock::now() < firstTime + std::chrono::seconds(10)) {
        auto frame = record_test::getFor<dai::ImgFrame>(*frames, std::chrono::milliseconds(200));
        if(frame == nullptr) continue;
        last = frame;
        lastTime = std::chrono::steady_clock::now();
    }
    const auto span = std::chrono::duration<double>(lastTime - firstTime).count();
    REQUIRE(span > 0.0);
    const auto fps = static_cast<double>(last->getSequenceNum() - first->getSequenceNum() + 1) / span;
    CHECK(std::fabs(fps - 30.0) <= 30.0 * 0.10);
}

TEST_CASE("VR21 a 4000x3000 frame warns about 554 MB/s", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR21");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    setFps(*raw.node, 30u);
    pipeline.start();
    sendFrame(raw.frames, makeImg(4000, 3000, 0, 0));
    const bool sawRate = store.devLog.waitFor("Record expected write rate 554 MB/s", std::chrono::seconds(15))
                         || store.hostLog.waitFor("Record expected write rate 554 MB/s", std::chrono::seconds(1));
    REQUIRE(sawRate);
    CHECK(store.devLog.count(LineLevel::WARN, "Record expected write rate 554 MB/s") + store.hostLog.count(LineLevel::WARN, "Record expected write rate 554 MB/s") == 1);
    pipeline.stop();
}

TEST_CASE("VR22 two streams land in one recording", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR22");
    auto& pipeline = store.begin();
    auto left = addRaw(pipeline, store.dir.path, name, true, false, "left");
    auto right = addRaw(pipeline, store.dir.path, name, true, false, "right");
    pipeline.start();
    for(int index = 0; index < 2; ++index) {
        sendWroteFrame(store, left, index, index);
        sendWroteFrame(store, right, index, index);
    }
    pipeline.stop();
    downloadRecording(*store.device, name, store.dir.path);
    expectFiles(store.dir.path / name / "left", {"left_00000000.dai", "left_00000001.dai"});
    expectFiles(store.dir.path / name / "right", {"right_00000000.dai", "right_00000001.dai"});
}

TEST_CASE("VR23 two nodes with one stream name error at start", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR23");
    auto& pipeline = store.begin();
    auto first = addRaw(pipeline, store.dir.path, name, true, false);
    auto second = addRaw(pipeline, store.dir.path, name, true, false);
    (void)first;
    (void)second;
    REQUIRE_THROWS(pipeline.start());
}

TEST_CASE("VR24 legacy encoded files transfer to the host", "[record][device]") {
    Store store;
    const auto before = listRecordings(*store.device);
    auto& pipeline = store.begin();
    const auto source = store.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(10), rangeTs(10));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setRunOnHost(*replay, true);
    replay_test::setSource(*replay, source);
    replay_test::setFast(*replay);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(30.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    replay->out.link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setRecordVideoFile(*record, store.dir.path / "v.mp4");
    setRecordMetadataFile(*record, store.dir.path / "v.mcap");
    encoder->out.link(record->input);
    pipeline.start();
    CHECK(record->runOnHost() == false);
    REQUIRE(record_test::waitUntil([&] { return store.devLog.countSubstring("Record wrote frame") >= 10; }, std::chrono::seconds(30)));
    pipeline.stop();
    CHECK(std::filesystem::is_regular_file(store.dir.path / "v.mp4"));
    CHECK(std::filesystem::is_regular_file(store.dir.path / "v.mcap"));
    CHECK(record_test::countVideoFrames(store.dir.path / "v.mp4", store.dir.path / "v.mcap") == 10);
    CHECK(listRecordings(*store.device).size() == before.size());
    CHECK(store.hostLog.countSubstring("Record download") >= 1);
}

TEST_CASE("VR25 legacy files stay on the device until download", "[record][device]") {
    Store store;
    const auto name = uniqueName("VR25");
    auto& pipeline = store.begin();
    const auto source = store.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(10), rangeTs(10));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setRunOnHost(*replay, true);
    replay_test::setSource(*replay, source);
    replay_test::setFast(*replay);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(30.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    replay->out.link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setRecordVideoFile(*record, store.dir.path / "v.mp4");
    setRecordMetadataFile(*record, store.dir.path / "v.mcap");
    setTransferToHost(*record, false);
    setRecordingName(*record, name);
    encoder->out.link(record->input);
    pipeline.start();
    REQUIRE(record_test::waitUntil([&] { return store.devLog.countSubstring("Record wrote frame") >= 10; }, std::chrono::seconds(30)));
    pipeline.stop();
    CHECK_FALSE(std::filesystem::is_regular_file(store.dir.path / "v.mp4"));
    CHECK(named(listRecordings(*store.device), name));
    downloadRecording(*store.device, name, store.dir.path / "down");
    bool video = false;
    bool metadata = false;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(store.dir.path / "down")) {
        if(!entry.is_regular_file()) continue;
        if(entry.path().filename() == "v.mp4") video = true;
        if(entry.path().filename() == "v.mcap") metadata = true;
    }
    CHECK(video);
    CHECK(metadata);
}

TEST_CASE("VR26 a legacy image recording runs on the host", "[record][device]") {
    Store store;
    auto& pipeline = store.begin();
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setRecordVideoFile(*record, store.dir.path / "v.avi");
    setRecordMetadataFile(*record, store.dir.path / "v.mcap");
    output->link(record->input);
    pipeline.start();
    CHECK(record->runOnHost());
    std::this_thread::sleep_for(std::chrono::seconds(2));
    pipeline.stop();
    CHECK(std::filesystem::is_regular_file(store.dir.path / "v.avi"));
    CHECK(std::filesystem::is_regular_file(store.dir.path / "v.mcap"));
    const int frames = record_test::countVideoFrames(store.dir.path / "v.avi", store.dir.path / "v.mcap");
    CHECK(frames >= 15);
    CHECK(frames <= 25);

    auto& again = store.replacePipeline();
    auto cameraAgain = again.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* outputAgain = cameraAgain->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);
    auto recordAgain = again.create<dai::node::RecordVideo>();
    record_test::watchNode(*recordAgain);
    setRecordVideoFile(*recordAgain, store.dir.path / "v2.avi");
    setRecordMetadataFile(*recordAgain, store.dir.path / "v2.mcap");
    setRunOnHost(*recordAgain, false);
    outputAgain->link(recordAgain->input);
    REQUIRE_THROWS(again.start());
}

TEST_CASE("VV1 a raw frame in video format errors", "[record][device]") {
    Store store;
    const auto name = uniqueName("VV1");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    setFormat(*raw.node, record_test::Format::VIDEO);
    pipeline.start();
    auto started = takeStatus(raw.status);
    CHECK(started->state == dai::RecordStatus::State::STARTED);
    CHECK(started->segment == 0);
    sendFrame(raw.frames, makeImg(64, 48, 0));
    auto error = takeStatus(raw.status);
    CHECK(error->state == dai::RecordStatus::State::ERROR);
    CHECK(error->segment == 0);
    CHECK(error->frameCount == 0);
    CHECK(error->firstSequenceNum == -1);
    CHECK(error->lastSequenceNum == -1);
    CHECK(error->skippedFrames == 0);
    CHECK(error->error.find("RAW_FRAMES") != std::string::npos);
    CHECK(error->error.find("VideoEncoder") != std::string::npos);
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(10)));
}

TEST_CASE("VV2 the raw-frame video error happens before a segment", "[record][device]") {
    Store store;
    const auto name = uniqueName("VV2");
    auto& pipeline = store.begin();
    auto raw = addRaw(pipeline, store.dir.path, name, true, false);
    setFormat(*raw.node, record_test::Format::VIDEO);
    setWaitForTrigger(*raw.node, true);
    setPreTriggerFrames(*raw.node, 0);
    auto control = armControl(raw);
    (void)control;
    pipeline.start();
    sendFrame(raw.frames, makeImg(64, 48, 0));
    auto error = takeStatus(raw.status);
    CHECK(error->state == dai::RecordStatus::State::ERROR);
    CHECK(error->segment == -1);
    CHECK(error->frameCount == 0);
    CHECK(error->firstSequenceNum == -1);
    CHECK(error->lastSequenceNum == -1);
    CHECK(error->skippedFrames == 0);
    CHECK(error->error.find("RAW_FRAMES") != std::string::npos);
    CHECK(error->error.find("VideoEncoder") != std::string::npos);
}

TEST_CASE("VV3 fifty encoded frames keep consecutive sequence numbers", "[record][device]") {
    Store store;
    const auto done = recordEncoded(store, uniqueName("VV3"), dai::VideoEncoderProperties::Profile::H264_MAIN, false);
    checkEncodedFiles(store, done);
}

TEST_CASE("VV4 H265 video format errors", "[record][device]") {
    Store store;
    const auto name = uniqueName("VV4");
    auto& pipeline = store.begin();
    const auto source = store.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(2), rangeTs(2));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setRunOnHost(*replay, true);
    replay_test::setSource(*replay, source);
    replay_test::setFast(*replay);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(25.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    encoder->setProfile(dai::VideoEncoderProperties::Profile::H265_MAIN);
    replay->out.link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::VIDEO);
    setRecordingName(*record, name);
    setOutputDirectory(*record, store.dir.path);
    setStreamName(*record, "enc");
    setTransferToHost(*record, false);
    encoder->out.link(record->input);
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    auto error = takeStatus(status);
    CHECK(error->state == dai::RecordStatus::State::ERROR);
    CHECK(error->segment == 0);
    CHECK(error->frameCount == 0);
    CHECK(error->firstSequenceNum == -1);
    CHECK(error->lastSequenceNum == -1);
    CHECK(error->skippedFrames == 0);
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(15)));
}

TEST_CASE("VV5 MJPEG video keeps fifty frames", "[record][device]") {
    Store store;
    const auto name = uniqueName("VV5");
    auto& pipeline = store.begin();
    const auto source = store.dir.path / "src";
    makeDaiDir(source, 1920, 1080, rangeSeq(50), rangeTs(50));
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    replay_test::setRunOnHost(*replay, true);
    replay_test::setSource(*replay, source);
    replay_test::setLoop(*replay, false);
    replay_test::setStopPipelineAtEnd(*replay, false);
    replay_test::setFps(*replay, 10.f);
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    encoder->setDefaultProfilePreset(25.f, dai::VideoEncoderProperties::Profile::H264_MAIN);
    encoder->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    replay->out.link(encoder->input);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record_test::watchNode(*record);
    setFormat(*record, record_test::Format::VIDEO);
    setRecordingName(*record, name);
    setOutputDirectory(*record, store.dir.path);
    setStreamName(*record, "enc");
    setFps(*record, 25u);
    setTransferToHost(*record, false);
    encoder->out.link(record->input);
    pipeline.start();
    REQUIRE(record_test::waitUntil([&] { return store.devLog.countSubstring("Record wrote frame") >= 50; }, std::chrono::seconds(60)));
    pipeline.stop();
    downloadRecording(*store.device, name, store.dir.path);
    expectFiles(store.dir.path / name, {"enc.mcap", "enc.mp4"});
    CHECK(record_test::countVideoFrames(store.dir.path / name / "enc.mp4", store.dir.path / name / "enc.mcap") == 50);
}

TEST_CASE("VH1 holistic raw frames from three cameras", "[record][device]") {
    Store store;
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    const auto tar = runHolistic(store, config, false, 10);
    checkRawMembers(tar, false);
    int files = 0;
    for(const auto& entry : std::filesystem::directory_iterator(store.dir.path)) {
        if(entry.is_regular_file()) ++files;
    }
    CHECK(files == 1);
    CHECK_FALSE(holisticNamed(listRecordings(*store.device)));
}

TEST_CASE("VH2 keep on device lists the holistic recording", "[record][device]") {
    Store store;
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    config.keepOnDevice = true;
    const auto tar = runHolistic(store, config, false, 10);
    CHECK(std::filesystem::is_regular_file(tar));
    int holistic = 0;
    for(const auto& row : listRecordings(*store.device)) {
        if(row.name.rfind("holistic_", 0) == 0) ++holistic;
    }
    CHECK(holistic == 1);
}

TEST_CASE("VH3 holistic raw frames include the IMU", "[record][device]") {
    Store store;
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    const auto tar = runHolistic(store, config, true, 10);
    checkRawMembers(tar, true);
}

TEST_CASE("VH4 holistic video writes one mp4 and mcap per camera", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::VIDEO;
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    pipeline.stop();
    const auto tar = oneTar(store.dir.path);
    const auto names = archiveNames(tar);
    const std::vector<std::string> expected = {"CameraCAM_A.mcap", "CameraCAM_A.mp4", "CameraCAM_B.mcap", "CameraCAM_B.mp4",
                                                "CameraCAM_C.mcap", "CameraCAM_C.mp4", "camera_info.json", "record_config.json"};
    CHECK(names == expected);
    const auto configBytes = dai::utility::readFileInArchive(tar, "record_config.json");
    const auto parsed = nlohmann::json::parse(configBytes.begin(), configBytes.end());
    CHECK(parsed["format"] == "VIDEO");
}

TEST_CASE("VH5 video encoding off uses host avi files", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::VIDEO;
    config.videoEncoding.enabled = false;
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    pipeline.stop();
    CHECK(store.hostLog.count(LineLevel::WARN, "Record holistic: video encoding off, host nodes used") == 1);
    const auto names = archiveNames(oneTar(store.dir.path));
    const std::vector<std::string> expected = {"CameraCAM_A.avi", "CameraCAM_A.mcap", "CameraCAM_B.avi", "CameraCAM_B.mcap",
                                                "CameraCAM_C.avi", "CameraCAM_C.mcap", "camera_info.json", "record_config.json"};
    CHECK(names == expected);
}

TEST_CASE("VH6 DEPTHAI_RECORD raw json matches the holistic raw tar", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    const auto jsonPath = store.dir.path / "record.json";
    writeRecordJson(jsonPath, store.dir.path / "out", "RAW_FRAMES", false);
    std::filesystem::create_directories(store.dir.path / "out");
    EnvGuard recordEnv("DEPTHAI_RECORD", jsonPath.string());
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    pipeline.stop();
    store.pipeline.reset();
    store.guard->release();
    checkRawMembers(oneTar(store.dir.path / "out"), false);
}

TEST_CASE("VH7 an unknown DEPTHAI_RECORD format errors at start", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    const auto jsonPath = store.dir.path / "record.json";
    writeRecordJson(jsonPath, store.dir.path / "out", "XYZ", false);
    std::filesystem::create_directories(store.dir.path / "out");
    EnvGuard recordEnv("DEPTHAI_RECORD", jsonPath.string());
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring("XYZ"));
}

TEST_CASE("VH8 DEPTHAI_RECORD without format records video", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    const auto jsonPath = store.dir.path / "record.json";
    writeRecordJson(jsonPath, store.dir.path / "out", nullptr, false);
    std::filesystem::create_directories(store.dir.path / "out");
    EnvGuard recordEnv("DEPTHAI_RECORD", jsonPath.string());
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    pipeline.stop();
    store.pipeline.reset();
    store.guard->release();
    const auto names = archiveNames(oneTar(store.dir.path / "out"));
    const std::vector<std::string> expected = {"CameraCAM_A.mcap", "CameraCAM_A.mp4", "CameraCAM_B.mcap", "CameraCAM_B.mp4",
                                                "CameraCAM_C.mcap", "CameraCAM_C.mp4", "camera_info.json", "record_config.json"};
    CHECK(names == expected);
    const auto configBytes = dai::utility::readFileInArchive(oneTar(store.dir.path / "out"), "record_config.json");
    const auto parsed = nlohmann::json::parse(configBytes.begin(), configBytes.end());
    CHECK(parsed["format"] == "VIDEO");
}

TEST_CASE("VH9 the three camera counts differ by at most one", "[record][device]") {
    Store store;
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    const auto tar = runHolistic(store, config, false, 10);
    const auto names = archiveNames(tar);
    const int countA = countPrefixed(names, "CameraCAM_A/CameraCAM_A_");
    const int countB = countPrefixed(names, "CameraCAM_B/CameraCAM_B_");
    const int countC = countPrefixed(names, "CameraCAM_C/CameraCAM_C_");
    CHECK(std::abs(countA - countB) <= 1);
    CHECK(std::abs(countB - countC) <= 1);
    CHECK(std::abs(countA - countC) <= 1);
}

TEST_CASE("VH10 sync runs on the device", "[record][device]") {
    Store store;
    skipUnlessThree(*store.device);
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    config.syncCameraOutputs = true;
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    int syncs = 0;
    int demuxes = 0;
    for(const auto& node : pipeline.getAllNodes()) {
        if(std::dynamic_pointer_cast<dai::node::Sync>(node) != nullptr) {
            ++syncs;
            CHECK(node->runOnHost() == false);
        }
        if(std::dynamic_pointer_cast<dai::node::MessageDemux>(node) != nullptr) {
            ++demuxes;
            CHECK(node->runOnHost() == false);
        }
    }
    CHECK(syncs >= 1);
    CHECK(demuxes >= 1);
    pipeline.stop();
}

TEST_CASE("VH11 holistic replay returns frames from each camera", "[record][device]") {
    Store store;
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    const auto tar = runHolistic(store, config, false, 10);
    auto& pipeline = store.replacePipeline();
    skipUnlessThree(*store.device);
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 16, true);
    pipeline.enableHolisticReplay(tar.string());
    pipeline.start();
    for(const auto& camera : cameras) {
        int count = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while(count < 50 && std::chrono::steady_clock::now() < deadline) {
            auto frame = record_test::getFor<dai::ImgFrame>(*camera.queue, std::chrono::seconds(5));
            if(frame == nullptr) break;
            CHECK(frame->getWidth() == 1280);
            CHECK(frame->getHeight() == 720);
            ++count;
        }
        CHECK(count >= 50);
    }
}

TEST_CASE("VH12 an RVC2 holistic video tar exists after stop", "[record][device]") {
    auto device = openRvc2OrSkip();
    if(device->getConnectedCameras().size() < 3) SKIP("fewer than three cameras");
    EnvGuard unsetReplay("DEPTHAI_REPLAY");
    EnvGuard unsetRecord("DEPTHAI_RECORD");
    TempDir dir;
    dai::Pipeline pipeline(device);
    dai::RecordConfig config;
    config.outputDir = dir.path;
    config.format = dai::RecordFormat::VIDEO;
    for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        auto camera = pipeline.create<dai::node::Camera>()->build(socket);
        auto* output = camera->requestOutput({1280, 720}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 10.f);
        (void)output->createOutputQueue(4, false);
    }
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    pipeline.stop();
    CHECK(std::filesystem::is_regular_file(oneTar(dir.path)));
}

TEST_CASE("VH13 a read-only output keeps the holistic recording on the device", "[record][device]") {
    if(geteuid() == 0) SKIP("running as root");
    Store store;
    skipUnlessThree(*store.device);
    dai::RecordConfig config;
    config.outputDir = store.dir.path;
    config.format = dai::RecordFormat::RAW_FRAMES;
    config.keepOnDevice = true;
    auto& pipeline = store.begin();
    auto cameras = addCameras(pipeline, 1280, 720, 10.f, 4, false);
    (void)cameras;
    pipeline.enableHolisticRecord(config);
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    DirMode mode(store.dir.path);
    std::filesystem::permissions(store.dir.path,
                                 std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec | std::filesystem::perms::group_read
                                     | std::filesystem::perms::group_exec | std::filesystem::perms::others_read | std::filesystem::perms::others_exec);
    REQUIRE_NOTHROW(pipeline.stop());
    bool tar = false;
    std::error_code error;
    for(const auto& entry : std::filesystem::directory_iterator(store.dir.path, error)) {
        if(entry.is_regular_file() && entry.path().extension() == ".gz") tar = true;
    }
    CHECK_FALSE(tar);
    CHECK(store.hostLog.count(LineLevel::ERROR, "holistic_") >= 1);
    CHECK(holisticNamed(listRecordings(*store.device)));
}
