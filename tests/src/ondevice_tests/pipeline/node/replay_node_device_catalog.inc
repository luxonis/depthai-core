using replay_test::frameBytes;
using replay_test::near;
using replay_test::pixelsWithin;
using replay_test::writeMeta;
using record_test::loadImg;

namespace {

int needleCount(const DeviceReplay& fx, const std::string& needle) {
    const int onDevice = fx.devLog.countSubstring(needle);
    const int onHost = fx.hostLog.countSubstring(needle);
    if(onDevice == 0) return onHost;
    if(onHost == 0) return onDevice;
    return std::max(onDevice, onHost);
}

std::vector<std::string> progressLines(const DeviceReplay& fx, const std::string& needle) {
    const auto deviceLines = fx.devLog.texts();
    const auto hostLines = fx.hostLog.texts();
    const auto& source = fx.devLog.countSubstring(needle) > 0 ? deviceLines : hostLines;
    std::vector<std::string> lines;
    for(const auto& line : source) {
        if(line.find(needle) != std::string::npos) lines.push_back(line);
    }
    return lines;
}

void dropPipeline(DeviceReplay& fx) {
    if(fx.pipeline == nullptr) return;
    try {
        if(fx.pipeline->isRunning()) fx.pipeline->stop();
    } catch(const std::exception&) {
    }
    fx.recordings.reset();
    fx.pipeline.reset();
}

std::filesystem::path videoFile(const char* name) {
    requireData();
    return replayDataRoot() / name;
}

std::shared_ptr<dai::ImgFrame> cloneFrame(const dai::ImgFrame& frame) {
    auto copy = std::make_shared<dai::ImgFrame>();
    const auto data = frame.getData();
    copy->setData(std::vector<std::uint8_t>(data.begin(), data.end()));
    copy->fb = frame.fb;
    copy->transformation = frame.transformation;
    copy->setSequenceNum(frame.getSequenceNum());
    copy->setTimestamp(frame.getTimestamp());
    copy->setTimestampDevice(frame.getTimestampDevice());
    return copy;
}

std::vector<std::shared_ptr<dai::ImgFrame>> playSource(DeviceReplay& fx, const std::filesystem::path& source, bool onHost, int count, bool preDecode) {
    dropPipeline(fx);
    auto& pipeline = fx.begin();
    auto node = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*node);
    setSource(*node, source);
    setFast(*node);
    if(preDecode) setDecodeMode(*node, DecodeMode::PRE_DECODE);
    if(onHost) setRunOnHost(*node, true);
    auto out = node->out.createOutputQueue(16, true);
    pipeline.start();
    auto frames = takeFrames(out, count);
    pipeline.stop();
    std::vector<std::shared_ptr<dai::ImgFrame>> clones;
    clones.reserve(frames.size());
    for(const auto& frame : frames) clones.push_back(cloneFrame(*frame));
    return clones;
}

void checkMeanY(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames, double tolerance) {
    REQUIRE(frames.size() == 30);
    for(int index = 0; index < 30; ++index) {
        CHECK(near(meanY(*frames[static_cast<std::size_t>(index)]), 16.0 + 7.0 * index, tolerance));
    }
}

void compareSides(DeviceReplay& fx, const std::filesystem::path& source, int count, int tolerance) {
    const auto onDevice = playSource(fx, source, false, count, false);
    const auto onHost = playSource(fx, source, true, count, false);
    REQUIRE(onDevice.size() == static_cast<std::size_t>(count));
    REQUIRE(onHost.size() == static_cast<std::size_t>(count));
    for(int index = 0; index < count; ++index) {
        const auto& left = *onDevice[static_cast<std::size_t>(index)];
        const auto& right = *onHost[static_cast<std::size_t>(index)];
        CHECK(left.getSequenceNum() == right.getSequenceNum());
        CHECK(left.getWidth() == right.getWidth());
        CHECK(left.getHeight() == right.getHeight());
        CHECK(left.getType() == right.getType());
        CHECK(pixelsWithin(left, right, tolerance));
    }
}

std::filesystem::path defectiveFrames(const std::filesystem::path& dir, int which) {
    makeDaiDir(dir, 64, 48, rangeSeq(3), rangeTs(3));
    saveShortFrame(dir / ("cam_" + std::to_string(which) + ".dai"), which);
    return dir;
}

bool planeBytesAre(const dai::ImgFrame& frame, std::uint8_t value, bool chroma) {
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const int stride = std::max(static_cast<int>(frame.getStride()), width);
    const auto data = frame.getData();
    const int rows = chroma ? height / 2 : height;
    const int offset = chroma ? (frame.fb.p2Offset > 0 ? static_cast<int>(frame.fb.p2Offset) : height * stride) : 0;
    for(int row = 0; row < rows; ++row) {
        for(int col = 0; col < width; ++col) {
            const auto at = static_cast<std::size_t>(offset + row * stride + col);
            if(at >= data.size() || data[at] != value) return false;
        }
    }
    return true;
}

std::optional<dai::CameraBoardSocket> unusedSocket(dai::Device& device) {
    const auto connected = device.getConnectedCameras();
    const dai::CameraBoardSocket sockets[] = {dai::CameraBoardSocket::CAM_A,
                                              dai::CameraBoardSocket::CAM_B,
                                              dai::CameraBoardSocket::CAM_C,
                                              dai::CameraBoardSocket::CAM_D,
                                              dai::CameraBoardSocket::CAM_E,
                                              dai::CameraBoardSocket::CAM_F,
                                              dai::CameraBoardSocket::CAM_G,
                                              dai::CameraBoardSocket::CAM_H,
                                              dai::CameraBoardSocket::CAM_I,
                                              dai::CameraBoardSocket::CAM_J};
    for(const auto socket : sockets) {
        if(std::find(connected.begin(), connected.end(), socket) == connected.end()) return socket;
    }
    return std::nullopt;
}

void skipUnlessThree(dai::Device& device) {
    if(device.getConnectedCameras().size() < 3) SKIP("fewer than three cameras");
}

std::filesystem::path makeHolistic(DeviceReplay& fx, const std::string& config) {
    skipUnlessThree(*fx.device);
    requireData();
    const auto info = cameraInfoFor(fx.device);
    if(info.empty()) SKIP("fewer than three cameras");
    const auto dir = fx.dir.path / "holistic";
    makeHolisticDir(dir, config, info);
    return dir;
}

std::shared_ptr<dai::MessageQueue> addCamera(dai::Pipeline& pipeline, dai::CameraBoardSocket socket, std::pair<std::uint32_t, std::uint32_t> size, dai::ImgFrame::Type type) {
    auto camera = pipeline.create<dai::node::Camera>()->build(socket);
    auto* output = camera->requestOutput(size, type, dai::ImgResizeMode::CROP, 30.f);
    return output->createOutputQueue(100, true);
}

std::vector<std::shared_ptr<dai::ImgFrame>> takeFor(dai::MessageQueue& queue, int count, std::chrono::milliseconds timeout) {
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    frames.reserve(static_cast<std::size_t>(count));
    for(int index = 0; index < count; ++index) {
        auto frame = record_test::getFor<dai::ImgFrame>(queue, timeout);
        REQUIRE(frame != nullptr);
        frames.push_back(frame);
    }
    return frames;
}

void checkMeta60(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames) {
    REQUIRE(frames.size() == 60);
    for(int index = 0; index < 60; ++index) {
        const auto expected = static_cast<std::int64_t>(1000 + 33 * index);
        CHECK(frames[static_cast<std::size_t>(index)]->getSequenceNum() == index);
        CHECK(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestamp()) == expected);
        CHECK(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestampDevice()) == expected);
    }
}

void checkSizeAndGap(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames) {
    REQUIRE(frames.size() == 60);
    CHECK(frames[0]->getWidth() == 1920);
    CHECK(frames[0]->getHeight() == 1080);
    double sum = 0.0;
    for(int index = 1; index < 60; ++index) {
        sum += static_cast<double>(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestampDevice())
                                   - timestampMs(frames[static_cast<std::size_t>(index - 1)]->getTimestampDevice()));
    }
    CHECK(std::fabs(sum / 59.0 - 33.0) <= 2.0);
}

void checkExactB(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames) {
    REQUIRE(frames.size() == 60);
    const auto expected = dai::ImgTransformation(1920, 1080, record_test::intrinsicsT(1920, 1080));
    for(int index = 0; index < 60; ++index) {
        const auto& frame = *frames[static_cast<std::size_t>(index)];
        const auto packed = frameBytes(1920, 1080, index, dai::ImgFrame::Type::NV12, FrameFill::plainYuv(80, 128, 128));
        CHECK(frame.getSequenceNum() == index);
        const auto ts = static_cast<std::int64_t>(1000 + 33 * index);
        CHECK(timestampMs(frame.getTimestamp()) == ts);
        CHECK(timestampMs(frame.getTimestampDevice()) == ts);
        CHECK(pixelsEqual(frame, packed, 1920, 1080, dai::ImgFrame::Type::NV12));
        CHECK(frame.transformation.getSourceIntrinsicMatrix() == expected.getSourceIntrinsicMatrix());
        CHECK(frame.transformation.getTransformationMatrix() == expected.getTransformationMatrix());
    }
}

void checkImu(dai::MessageQueue& queue) {
    for(int index = 0; index < 10; ++index) {
        auto imu = record_test::getFor<dai::IMUData>(queue, std::chrono::seconds(60));
        REQUIRE(imu != nullptr);
        const auto expected = static_cast<std::int64_t>(index) * 100;
        CHECK(imu->getSequenceNum() == index);
        CHECK(timestampMs(imu->getTimestamp()) == expected);
        CHECK(timestampMs(imu->getTimestampDevice()) == expected);
        REQUIRE(!imu->packets.empty());
        const auto& report = imu->packets.front().acceleroMeter;
        CHECK(report.getSequenceNum() == index);
        CHECK(report.timestamp.sec == expected / 1000);
        CHECK(report.timestamp.nsec == (expected % 1000) * 1000000);
        CHECK(report.tsDevice.sec == expected / 1000);
        CHECK(report.tsDevice.nsec == (expected % 1000) * 1000000);
    }
}

struct HolisticOptions {
    bool camA = false;
    bool camB = false;
    bool camC = false;
    bool imu = false;
    bool meanB = false;
    bool exactB = false;
    bool sizeAndGapC = false;
    bool cacheStream = false;
    bool cachePredecode = false;
    bool cachePersistent = false;
    bool oneReplayVideo = false;
    bool metadataOnDevice = false;
};

void runHolistic(DeviceReplay& fx, const std::filesystem::path& recording, const HolisticOptions& options) {
    dropPipeline(fx);
    auto& pipeline = fx.begin();
    std::shared_ptr<dai::MessageQueue> queueA;
    std::shared_ptr<dai::MessageQueue> queueB;
    std::shared_ptr<dai::MessageQueue> queueC;
    std::shared_ptr<dai::MessageQueue> queueImu;
    if(options.camA) queueA = addCamera(pipeline, dai::CameraBoardSocket::CAM_A, {1920, 1080}, dai::ImgFrame::Type::NV12);
    if(options.camB) queueB = addCamera(pipeline, dai::CameraBoardSocket::CAM_B, {1920, 1080}, dai::ImgFrame::Type::NV12);
    if(options.camC) queueC = addCamera(pipeline, dai::CameraBoardSocket::CAM_C, {1920, 1080}, dai::ImgFrame::Type::NV12);
    if(options.imu) {
        auto imu = pipeline.create<dai::node::IMU>();
        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 100);
        queueImu = imu->out.createOutputQueue(100, true);
    }
    pipeline.enableHolisticReplay(recording.string());
    pipeline.start();
    if(queueA) checkMeta60(takeFor(*queueA, 60, std::chrono::seconds(60)));
    if(queueB) {
        auto frames = takeFor(*queueB, 60, std::chrono::seconds(60));
        checkMeta60(frames);
        if(options.meanB) {
            for(const auto& frame : frames) CHECK(near(meanY(*frame), 80.0, 1.0));
        }
        if(options.exactB) checkExactB(frames);
    }
    if(queueC) {
        auto frames = takeFor(*queueC, 60, std::chrono::seconds(60));
        checkMeta60(frames);
        if(options.sizeAndGapC) checkSizeAndGap(frames);
    }
    if(queueImu) checkImu(*queueImu);
    if(options.oneReplayVideo) {
        int videos = 0;
        for(const auto& node : pipeline.getAllNodes()) {
            if(std::dynamic_pointer_cast<dai::node::ReplayVideo>(node)) ++videos;
        }
        CHECK(videos == 1);
    }
    if(options.metadataOnDevice) {
        int found = 0;
        bool onHost = true;
        for(const auto& node : pipeline.getAllNodes()) {
            if(auto meta = std::dynamic_pointer_cast<dai::node::ReplayMetadataOnly>(node)) {
                ++found;
                onHost = meta->runOnHost();
            }
        }
        CHECK(found == 1);
        CHECK(onHost == false);
    }
    if(options.cacheStream) {
        const auto during = getReplayCacheInfo(*fx.device);
        CHECK(countKind(during, CacheKind::SOURCE) == 1);
        CHECK(countKind(during, CacheKind::DECODED) == 0);
    }
    if(options.cachePredecode) CHECK(countKind(getReplayCacheInfo(*fx.device), CacheKind::DECODED) == 2);
    pipeline.stop();
    if(options.cacheStream) CHECK(getReplayCacheInfo(*fx.device).entries.empty());
    if(options.cachePersistent) CHECK(countKind(getReplayCacheInfo(*fx.device), CacheKind::SOURCE) == 1);
    if(options.cacheStream || options.cachePersistent) CHECK(needleCount(fx, "Replay cache upload") == 1);
}

HolisticOptions vc1Options() {
    HolisticOptions options;
    options.camA = true;
    options.camB = true;
    options.camC = true;
    options.imu = true;
    options.meanB = true;
    options.cacheStream = true;
    return options;
}

}  // namespace

TEST_CASE("VB1 h264 mean Y follows 16 plus 7n", "[replay][device]") {
    const auto source = videoFile("plain_h264.mp4");
    DeviceReplay fx;
    checkMeanY(playSource(fx, source, false, 30, false), 2.0);
}

TEST_CASE("VB2 h265 mean Y follows 16 plus 7n", "[replay][device]") {
    const auto source = videoFile("plain_h265.mp4");
    DeviceReplay fx;
    checkMeanY(playSource(fx, source, false, 30, false), 2.0);
}

TEST_CASE("VB3 ffv1 mean Y is exact", "[replay][device]") {
    const auto source = videoFile("plain_ffv1.avi");
    DeviceReplay fx;
    checkMeanY(playSource(fx, source, false, 30, false), 0.01);
}

TEST_CASE("VB4 mjpeg mean Y follows 16 plus 7n", "[replay][device]") {
    const auto source = videoFile("plain_mjpeg.mp4");
    DeviceReplay fx;
    checkMeanY(playSource(fx, source, false, 30, false), 2.0);
}

TEST_CASE("VB5 predecode h264 matches stream", "[replay][device]") {
    const auto source = videoFile("plain_h264.mp4");
    DeviceReplay fx;
    const auto streamed = playSource(fx, source, false, 30, false);
    const auto decoded = playSource(fx, source, false, 30, true);
    checkMeanY(streamed, 2.0);
    REQUIRE(decoded.size() == 30);
    for(int index = 0; index < 30; ++index) {
        CHECK(pixelsWithin(*streamed[static_cast<std::size_t>(index)], *decoded[static_cast<std::size_t>(index)], 0));
    }
}

TEST_CASE("VB6 predecode logs ten upload and decode lines", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 20);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    CHECK(takeFrames(play.out, 20).size() == 20);
    pipeline.stop();
    const auto uploads = progressLines(fx, "Replay upload");
    const auto decodes = progressLines(fx, "Replay decode");
    REQUIRE(uploads.size() == 10);
    REQUIRE(decodes.size() == 10);
    const auto total = std::to_string(directoryBytes(dir));
    CHECK(uploads.back().find(total + "/" + total + " bytes") != std::string::npos);
    CHECK(decodes.back().find("20/20 frames") != std::string::npos);
}

TEST_CASE("VB7 three frames log three decode lines", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    CHECK(takeFrames(play.out, 3).size() == 3);
    pipeline.stop();
    const auto decodes = progressLines(fx, "Replay decode");
    REQUIRE(decodes.size() == 3);
    CHECK(decodes[0].find("1/3") != std::string::npos);
    CHECK(decodes[1].find("2/3") != std::string::npos);
    CHECK(decodes[2].find("3/3") != std::string::npos);
}

TEST_CASE("VB8 predecode returns after the last decode line", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 87);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    pipeline.start();
    const auto decodes = progressLines(fx, "Replay decode");
    REQUIRE(!decodes.empty());
    CHECK(decodes.back().find("87/87 frames") != std::string::npos);
    const auto marked = std::chrono::steady_clock::now();
    auto first = record_test::getFor<dai::ImgFrame>(*play.out, std::chrono::seconds(2));
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - marked).count();
    CHECK(first != nullptr);
    CHECK(elapsed <= 2.0);
}

TEST_CASE("VB9 stop during predecode returns an error", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 640, 480, 87);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    std::atomic<bool> returned{false};
    std::exception_ptr error;
    std::thread worker([&] {
        try {
            pipeline.start();
        } catch(...) {
            error = std::current_exception();
        }
        returned = true;
    });
    const bool sawDecode = fx.devLog.waitFor("Replay decode", std::chrono::seconds(120)) || fx.hostLog.waitFor("Replay decode", std::chrono::milliseconds(1));
    if(returned.load()) {
        worker.join();
        SKIP("start was too fast");
    }
    if(!sawDecode) {
        pipeline.stop();
        worker.join();
        FAIL("Replay decode was not logged");
    }
    const auto marked = std::chrono::steady_clock::now();
    pipeline.stop();
    worker.join();
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - marked).count();
    CHECK(elapsed <= 5.0);
    REQUIRE(error != nullptr);
}

TEST_CASE("VB10 a truncated frame errors at start", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeFrames(fx.dir.path / "src", 64, 48, 3);
    const auto file = dir / "cam_2.dai";
    std::filesystem::resize_file(file, std::filesystem::file_size(file) / 2);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::STREAM);
    requireStart(pipeline, "cam_2.dai");
}

TEST_CASE("VB11 a short payload stops after two frames", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = defectiveFrames(fx.dir.path / "src", 2);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::STREAM);
    pipeline.start();
    CHECK(takeFrames(play.out, 2).size() == 2);
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(10)));
    CHECK(fx.devLog.count(LineLevel::ERROR, "cam_2.dai") + fx.hostLog.count(LineLevel::ERROR, "cam_2.dai") >= 1);
}

TEST_CASE("VB12 predecode rejects a short payload at start", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = defectiveFrames(fx.dir.path / "src", 2);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::PRE_DECODE);
    requireStart(pipeline, "cam_2.dai");
}

TEST_CASE("VB13 a short first frame errors at start", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = defectiveFrames(fx.dir.path / "src", 0);
    auto& pipeline = fx.begin();
    auto play = addPlay(pipeline, dir, false);
    setDecodeMode(*play.node, DecodeMode::STREAM);
    requireStart(pipeline, "cam_0.dai");
}

TEST_CASE("VB14 host and device h264 frames match", "[replay][device]") {
    const auto source = videoFile("plain_h264.mp4");
    DeviceReplay fx;
    compareSides(fx, source, 30, 0);
}

TEST_CASE("VB15 host and device ffv1 frames match", "[replay][device]") {
    const auto source = videoFile("plain_ffv1.avi");
    DeviceReplay fx;
    compareSides(fx, source, 30, 0);
}

TEST_CASE("VB16 host and device mjpeg frames match within 2", "[replay][device]") {
    const auto source = videoFile("plain_mjpeg.mp4");
    DeviceReplay fx;
    compareSides(fx, source, 30, 2);
}

TEST_CASE("VB17 host and device dai frames match", "[replay][device]") {
    DeviceReplay fx;
    const auto source = makeFrames(fx.dir.path / "src", 64, 48, 3);
    compareSides(fx, source, 3, 0);
}

TEST_CASE("VB18 device frames keep stride and pixels", "[replay][device]") {
    DeviceReplay fx;
    const auto source = makeFrames(fx.dir.path / "src", 64, 48, 3);
    const auto frames = playSource(fx, source, false, 3, false);
    REQUIRE(frames.size() == 3);
    for(int index = 0; index < 3; ++index) {
        const auto& frame = *frames[static_cast<std::size_t>(index)];
        CHECK(frame.getStride() >= 64);
        auto stored = loadImg(source / ("cam_" + std::to_string(index) + ".dai"));
        REQUIRE(stored != nullptr);
        const auto packed = std::vector<std::uint8_t>(stored->getData().begin(), stored->getData().end());
        CHECK(pixelsEqual(frame, packed, 64, 48, dai::ImgFrame::Type::NV12));
    }
}

TEST_CASE("VB19 unset fps keeps a 33.3 ms device interval", "[replay][device]") {
    const auto source = videoFile("plain_h264.mp4");
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    auto node = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*node);
    setSource(*node, source);
    setLoop(*node, false);
    setStopPipelineAtEnd(*node, false);
    auto out = node->out.createOutputQueue(4, false);
    pipeline.start();
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    for(int index = 0; index < 30; ++index) {
        auto frame = record_test::getFor<dai::ImgFrame>(*out, std::chrono::seconds(5));
        REQUIRE(frame != nullptr);
        frames.push_back(frame);
    }
    pipeline.stop();
    double sum = 0.0;
    for(int index = 1; index < 30; ++index) {
        sum += static_cast<double>(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestampDevice())
                                   - timestampMs(frames[static_cast<std::size_t>(index - 1)]->getTimestampDevice()));
    }
    CHECK(std::fabs(sum / 29.0 - 33.3) <= 2.0);
    double lowest = 0.0;
    double highest = 0.0;
    for(int index = 0; index < 30; ++index) {
        const double delta = static_cast<double>(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestamp())
                                                 - timestampMs(frames[static_cast<std::size_t>(index)]->getTimestampDevice()));
        if(index == 0 || delta < lowest) lowest = delta;
        if(index == 0 || delta > highest) highest = delta;
    }
    CHECK(highest - lowest <= 20.0);
}

TEST_CASE("VC1 holistic tar replays cameras and imu", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    runHolistic(fx, tar, vc1Options());
}

TEST_CASE("VC2 one camera builds one replay node", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    HolisticOptions options;
    options.camA = true;
    options.oneReplayVideo = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC3 an unconnected extra camera errors at start", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    const auto extra = unusedSocket(*fx.device);
    if(!extra) SKIP("fewer than three cameras");
    auto& pipeline = fx.begin();
    addCamera(pipeline, dai::CameraBoardSocket::CAM_A, {1920, 1080}, dai::ImgFrame::Type::NV12);
    addCamera(pipeline, dai::CameraBoardSocket::CAM_B, {1920, 1080}, dai::ImgFrame::Type::NV12);
    addCamera(pipeline, dai::CameraBoardSocket::CAM_C, {1920, 1080}, dai::ImgFrame::Type::NV12);
    addCamera(pipeline, *extra, {1920, 1080}, dai::ImgFrame::Type::NV12);
    pipeline.enableHolisticReplay(tar.string());
    requireStart(pipeline);
}

TEST_CASE("VC4 predecode keeps two decoded entries", "[replay][device]") {
    DeviceReplay fx;
    record_test::EnvGuard decodeMode("DEPTHAI_REPLAY_DECODE_MODE", "PRE_DECODE");
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.cacheStream = false;
    options.cachePredecode = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC15 persistent holistic upload is reused", "[replay][device]") {
    DeviceReplay fx;
    record_test::EnvGuard persistent("DEPTHAI_REPLAY_PERSISTENT_UPLOAD", "1");
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.cacheStream = false;
    options.cachePersistent = true;
    runHolistic(fx, tar, options);
    const int uploads = needleCount(fx, "Replay cache upload");
    const int hits = needleCount(fx, "Replay cache hit");
    runHolistic(fx, tar, options);
    CHECK(needleCount(fx, "Replay cache hit") == hits + 1);
    CHECK(needleCount(fx, "Replay cache upload") == uploads);
}

TEST_CASE("VC16 cam B matches the recorded frames", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.exactB = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC17 an archive entry replays cam A metadata", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto& pipeline = fx.begin();
    auto node = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*node);
    setSource(*node, tar);
    setArchiveEntry(*node, "CameraCAM_A");
    setLoop(*node, false);
    setStopPipelineAtEnd(*node, false);
    setFps(*node, 1000.f);
    auto out = node->out.createOutputQueue(64, true);
    pipeline.start();
    checkMeta60(takeFor(*out, 60, std::chrono::seconds(60)));
    pipeline.stop();
    dropPipeline(fx);
    auto& again = fx.begin();
    auto missing = again.create<dai::node::ReplayVideo>();
    record_test::watchNode(*missing);
    setSource(*missing, tar);
    setArchiveEntry(*missing, "CameraCAM_X");
    setLoop(*missing, false);
    setStopPipelineAtEnd(*missing, false);
    setFps(*missing, 1000.f);
    missing->out.createOutputQueue(4, true);
    requireStart(again, "CameraCAM_X");
}

TEST_CASE("VC18 metadata-only replays imu and cam B", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    {
        auto& pipeline = fx.begin();
        auto node = pipeline.create<dai::node::ReplayMetadataOnly>();
        record_test::watchNode(*node);
        setSource(*node, tar);
        setArchiveEntry(*node, "IMU");
        setLoop(*node, false);
        setFps(*node, 1000.f);
        auto out = node->out.createOutputQueue(16, true);
        pipeline.start();
        checkImu(*out);
        pipeline.stop();
        dropPipeline(fx);
    }
    {
        auto& pipeline = fx.begin();
        auto node = pipeline.create<dai::node::ReplayMetadataOnly>();
        record_test::watchNode(*node);
        setSource(*node, tar);
        setArchiveEntry(*node, "CameraCAM_B");
        setLoop(*node, false);
        setFps(*node, 1000.f);
        auto out = node->out.createOutputQueue(64, true);
        pipeline.start();
        checkExactB(takeFor(*out, 60, std::chrono::seconds(60)));
        pipeline.stop();
        dropPipeline(fx);
    }
    replay_test::makeImuMcap(fx.dir.path / "empty.mcap", 0);
    auto& pipeline = fx.begin();
    auto node = pipeline.create<dai::node::ReplayMetadataOnly>();
    record_test::watchNode(*node);
    setReplayFile(*node, fx.dir.path / "empty.mcap");
    setLoop(*node, false);
    setFps(*node, 1000.f);
    node->out.createOutputQueue(4, true);
    requireStart(pipeline);
}

TEST_CASE("VC19 a directory and a flat tar match the gzip recording", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    runHolistic(fx, dir, vc1Options());
    const auto tar = makeTar(fx.dir.path / "holistic.tar", dir, TarKind::FLAT);
    runHolistic(fx, tar, vc1Options());
}

TEST_CASE("VC5 cam C is 1920x1080 at 33 ms", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.sizeAndGapC = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC6 a record config without streams still sizes cam C", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfigNoStreams);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.sizeAndGapC = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC7 metadata replay runs on the device", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    auto options = vc1Options();
    options.metadataOnDevice = true;
    runHolistic(fx, tar, options);
}

TEST_CASE("VC8 a camera built on replay keeps dai time", "[replay][device]") {
    DeviceReplay fx;
    const auto source = makeFrames(fx.dir.path / "src", 1920, 1080, 10);
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setFast(*replay);
    auto replayOut = replay->out.createOutputQueue(8, true);
    auto camera = buildCamera(pipeline, dai::CameraBoardSocket::CAM_A, *replay);
    auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto cameraOut = output->createOutputQueue(8, true);
    pipeline.start();
    for(int index = 0; index < 10; ++index) {
        auto replayed = record_test::getFor<dai::ImgFrame>(*replayOut, std::chrono::seconds(10));
        auto frame = record_test::getFor<dai::ImgFrame>(*cameraOut, std::chrono::seconds(10));
        REQUIRE(replayed != nullptr);
        REQUIRE(frame != nullptr);
        CHECK(replayed->getType() == dai::ImgFrame::Type::NV12);
        CHECK(timestampMs(frame->getTimestampDevice()) == static_cast<std::int64_t>(index) * 100);
    }
}

TEST_CASE("VC9 a device source builds the camera at 1920x1080", "[replay][device]") {
    DeviceReplay fx;
    const auto host = makeFrames(fx.dir.path / "src", 1920, 1080, 10);
    const auto devicePath = makeDeviceSource(fx.device, host);
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, devicePath, Location::DEVICE);
    setFast(*replay);
    auto camera = buildCamera(pipeline, dai::CameraBoardSocket::CAM_A, *replay);
    auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto cameraOut = output->createOutputQueue(8, true);
    pipeline.start();
    auto frames = takeFor(*cameraOut, 10, std::chrono::seconds(30));
    pipeline.stop();
    REQUIRE(frames.size() == 10);
    CHECK(frames[0]->getWidth() == 1920);
    CHECK(frames[0]->getHeight() == 1080);
    deleteRecording(*fx.device, std::filesystem::path(devicePath).parent_path().filename().string());
}

TEST_CASE("VC10 neural build resizes replay to the model input", "[replay][device]") {
    dai::NNModelDescription description;
    description.model = "yolov6-nano";
    std::filesystem::path modelPath;
    try {
        modelPath = dai::getModelFromZoo(description, true, "", "", "none");
    } catch(const std::exception&) {
        SKIP("needs yolov6-nano");
    }
    std::optional<std::pair<std::uint32_t, std::uint32_t>> inputSize;
    try {
        inputSize = dai::NNArchive(modelPath).getInputSize();
    } catch(const std::exception&) {
        SKIP("needs yolov6-nano");
    }
    if(!inputSize) SKIP("needs yolov6-nano");
    DeviceReplay fx;
    const auto source = makeFrames(fx.dir.path / "src", 1920, 1080, 10);
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setFast(*replay);
    auto network = pipeline.create<dai::node::NeuralNetwork>();
    try {
        network->build(replay, description);
    } catch(const std::exception& ex) {
        const std::string text = ex.what();
        const bool namesType = text.find("NV12") != std::string::npos || text.find("YUV") != std::string::npos || text.find("GRAY") != std::string::npos
                               || text.find("RGB") != std::string::npos || text.find("BGR") != std::string::npos;
        if(namesType) SKIP(text.c_str());
        throw;
    }
    auto out = replay->out.createOutputQueue(8, true);
    pipeline.start();
    auto frames = takeFor(*out, 10, std::chrono::seconds(60));
    pipeline.stop();
    for(const auto& frame : frames) {
        CHECK(frame->getWidth() == inputSize->first);
        CHECK(frame->getHeight() == inputSize->second);
        const auto type = frame->getType();
        const bool allowed = type == dai::ImgFrame::Type::NV12 || type == dai::ImgFrame::Type::YUV420p || type == dai::ImgFrame::Type::GRAY8
                             || type == dai::ImgFrame::Type::RGB888i || type == dai::ImgFrame::Type::BGR888i;
        CHECK(allowed);
    }
}

TEST_CASE("VC11 a changed mock size stops the pipeline", "[replay][device]") {
    DeviceReplay fx;
    const auto wide = makeFrames(fx.dir.path / "wide", 1920, 1080, 3);
    const auto small = makeFrames(fx.dir.path / "small", 1280, 720, 3);
    auto& pipeline = fx.begin();
    auto replayA = pipeline.create<dai::node::ReplayVideo>();
    auto replayB = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replayA);
    record_test::watchNode(*replayB);
    setSource(*replayA, wide);
    setSource(*replayB, small);
    setFast(*replayA);
    setFast(*replayB);
    auto camera = buildCamera(pipeline, dai::CameraBoardSocket::CAM_A, *replayA);
    replayA->out.unlink(camera->mockIsp);
    replayB->out.link(camera->mockIsp);
    pipeline.start();
    CHECK(record_test::waitUntil([&] { return !pipeline.isRunning(); }, std::chrono::seconds(10)));
    bool saw = false;
    for(const auto& line : fx.devLog.textsAt(LineLevel::ERROR)) {
        if(line.find("1280") != std::string::npos && line.find("1920") != std::string::npos) saw = true;
    }
    for(const auto& line : fx.hostLog.textsAt(LineLevel::ERROR)) {
        if(line.find("1280") != std::string::npos && line.find("1920") != std::string::npos) saw = true;
    }
    CHECK(saw);
}

TEST_CASE("VC12 camera outputs keep the plain color", "[replay][device]") {
    DeviceReplay fx;
    const auto source = makeFrames(fx.dir.path / "src", 64, 48, 3, FrameFill::plainYuv(100, 128, 128));
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setFast(*replay);
    auto camera = buildCamera(pipeline, dai::CameraBoardSocket::CAM_A, *replay);
    auto* nv12 = camera->requestOutput({64, 48}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    auto* gray = camera->requestOutput({64, 48}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f);
    auto nvQueue = nv12->createOutputQueue(8, true);
    auto grayQueue = gray->createOutputQueue(8, true);
    pipeline.start();
    for(int index = 0; index < 3; ++index) {
        auto color = record_test::getFor<dai::ImgFrame>(*nvQueue, std::chrono::seconds(10));
        auto mono = record_test::getFor<dai::ImgFrame>(*grayQueue, std::chrono::seconds(10));
        REQUIRE(color != nullptr);
        REQUIRE(mono != nullptr);
        CHECK(planeBytesAre(*color, 128, true));
        CHECK(planeBytesAre(*mono, 100, false));
    }
}

TEST_CASE("VC13 an unconnected socket errors at start", "[replay][device]") {
    DeviceReplay fx;
    const auto extra = unusedSocket(*fx.device);
    if(!extra) SKIP("fewer than three cameras");
    const auto source = makeFrames(fx.dir.path / "src", 1920, 1080, 3);
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setFast(*replay);
    buildCamera(pipeline, *extra, *replay);
    requireStart(pipeline, dai::toString(*extra).c_str());
}

TEST_CASE("VC14 the dai directory wins over the sibling video", "[replay][device]") {
    DeviceReplay fx;
    const auto dir = makeHolistic(fx, kHolisticConfig);
    replay_test::copyFile(replayDataRoot() / "plain60_h264.mp4", dir / "CameraCAM_C.mp4");
    writeMeta(dir / "CameraCAM_C.mcap", 60, 0, 1000, 1920, 1080, 33);
    const auto tar = makeTar(fx.dir.path / "holistic.tar.gz", dir, TarKind::GZ);
    dropPipeline(fx);
    auto& pipeline = fx.begin();
    auto queue = addCamera(pipeline, dai::CameraBoardSocket::CAM_C, {1920, 1080}, dai::ImgFrame::Type::NV12);
    pipeline.enableHolisticReplay(tar.string());
    pipeline.start();
    auto frames = takeFor(*queue, 60, std::chrono::seconds(60));
    pipeline.stop();
    for(const auto& frame : frames) CHECK(near(meanY(*frame), 50.0, 1.0));
}
