extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/videoio.hpp>

#include "../../../helpers/replay_test_support.hpp"

TEST_CASE("data replay helpers", "[replay][data]") {
    const auto abc = test_hash::sha256Hex(std::vector<std::uint8_t>{'a', 'b', 'c'});
    CHECK(abc == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

    replay_test::TempDir dir;
    std::ofstream(dir.path / "a", std::ios::binary) << "1";
    std::ofstream(dir.path / "b", std::ios::binary) << "2";
    const std::vector<std::uint8_t> blob{'a', 0, '1', 'b', 0, '2'};
    CHECK(replay_test::hashDir(dir.path) == test_hash::sha256Hex(blob));

    const auto bytes = replay_test::makeDaiFrame(dir.path / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    auto loaded = record_test::loadImg(dir.path / "cam_0.dai");
    CHECK(loaded->getSequenceNum() == 0);
    CHECK(loaded->getWidth() == 64);
    CHECK(loaded->getHeight() == 48);
    CHECK(replay_test::pixelsEqual(*loaded, bytes, 64, 48, dai::ImgFrame::Type::NV12));

    replay_test::makeDaiDir(dir.path / "flat", 64, 48, replay_test::rangeSeq(3), replay_test::rangeTs(3));
    const auto archive = replay_test::makeTar(dir.path / "flat.tar", dir.path / "flat", replay_test::TarKind::FLAT);
    const auto names = dai::utility::filenamesInArchive(archive);
    CHECK(std::find(names.begin(), names.end(), "./cam_0.dai") != names.end());
    CHECK(std::find(names.begin(), names.end(), "./cam_1.dai") != names.end());
    CHECK(std::find(names.begin(), names.end(), "./cam_2.dai") != names.end());

    if(!replay_test::replayDataAvailable()) SKIP("needs the generated data");
    std::ifstream input(replay_test::replayDataRoot() / "expected_meanY.json");
    REQUIRE(input);
    nlohmann::json means;
    input >> means;
    for(const char* name : {"plain_h264.mp4", "plain_h265.mp4", "plain_ffv1.avi", "plain_mjpeg.mp4"}) {
        cv::VideoCapture capture((replay_test::replayDataRoot() / name).string());
        REQUIRE(capture.isOpened());
        CHECK(static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT)) == 30);
        const auto listed = means.at(name).get<std::vector<double>>();
        REQUIRE(listed.size() == 30);
        const double tol = std::string(name) == "plain_ffv1.avi" ? 0.01 : 2.0;
        for(int n = 0; n < 30; ++n) {
            CHECK(std::fabs(listed[static_cast<std::size_t>(n)] - (16.0 + 7.0 * n)) <= tol);
        }
    }
}

#include <cctype>
#include <limits>

using replay_test::CacheKind;
using replay_test::DecodeMode;
using replay_test::FrameFill;
using replay_test::HostCacheGuard;
using replay_test::HostReplay;
using replay_test::Location;
using replay_test::OtherKind;
using replay_test::TarKind;
using replay_test::checkArrivals;
using replay_test::clearHostCache;
using replay_test::copyFile;
using replay_test::getHostCacheInfo;
using replay_test::hashDir;
using replay_test::makeDaiDir;
using replay_test::makeDaiFrame;
using replay_test::makeDaiOther;
using replay_test::makeMcap;
using replay_test::makeS1;
using replay_test::makeTar;
using replay_test::meanY;
using replay_test::near;
using replay_test::pixelsEqual;
using replay_test::rangeSeq;
using replay_test::rangeTs;
using replay_test::replayDataRoot;
using replay_test::requireReplayData;
using replay_test::setArchiveEntry;
using replay_test::setDecodeMode;
using replay_test::setFast;
using replay_test::setFps;
using replay_test::setHostCacheLimit;
using replay_test::setLoop;
using replay_test::setMetadataFile;
using replay_test::setOutFrameType;
using replay_test::setPersistentUpload;
using replay_test::setReplayMetadataFile;
using replay_test::setReplayVideoFile;
using replay_test::setRunOnHost;
using replay_test::setSize;
using replay_test::setSource;
using replay_test::setStopPipelineAtEnd;
using replay_test::takeFrames;
using replay_test::takeTimed;
using replay_test::writeMeta;
using Catch::Matchers::ContainsSubstring;
using record_test::LineLevel;
using record_test::loadImg;
using record_test::timestampMs;

namespace {

void requireStartError(HostReplay& fx, const char* text = nullptr) {
    if(text == nullptr) {
        REQUIRE_THROWS(fx.pipeline.start());
    } else {
        REQUIRE_THROWS_WITH(fx.pipeline.start(), ContainsSubstring(text));
    }
}

void requireStartErrorEither(HostReplay& fx, const char* first, const char* second) {
    try {
        fx.pipeline.start();
        FAIL("expected an error at the start");
    } catch(const std::exception& error) {
        const std::string text = error.what();
        INFO(text);
        CHECK((text.find(first) != std::string::npos || text.find(second) != std::string::npos));
    }
}

std::vector<std::uint8_t> packedOf(const std::filesystem::path& path) {
    auto loaded = loadImg(path);
    REQUIRE(loaded != nullptr);
    const auto data = loaded->getData();
    return {data.begin(), data.end()};
}

void checkTs(const dai::ImgFrame& frame, std::int64_t sequence, std::int64_t tsMs) {
    CHECK(frame.getSequenceNum() == sequence);
    CHECK(timestampMs(frame.getTimestampDevice()) == tsMs);
    CHECK(frame.getTimestamp() == frame.getTimestampDevice());
}

void checkDefault(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames, const std::filesystem::path& source, int count, int width = 64, int height = 48) {
    REQUIRE(static_cast<int>(frames.size()) == count);
    for(int index = 0; index < count; ++index) {
        CHECK(frames[static_cast<std::size_t>(index)]->getWidth() == static_cast<unsigned>(width));
        CHECK(frames[static_cast<std::size_t>(index)]->getHeight() == static_cast<unsigned>(height));
        CHECK(frames[static_cast<std::size_t>(index)]->getType() == dai::ImgFrame::Type::NV12);
        checkTs(*frames[static_cast<std::size_t>(index)], index, static_cast<std::int64_t>(index) * 100);
        const auto packed = packedOf(source / ("cam_" + std::to_string(index) + ".dai"));
        CHECK(pixelsEqual(*frames[static_cast<std::size_t>(index)], packed, width, height, dai::ImgFrame::Type::NV12));
    }
}

void playDefault(HostReplay& fx) {
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    checkDefault(frames, fx.source, 3);
    REQUIRE(fx.log.waitFor("Replay finished", std::chrono::seconds(10)));
    CHECK(fx.out->tryGet<dai::ImgFrame>() == nullptr);
}

std::filesystem::path videoOf(HostReplay& fx, const char* name) {
    requireReplayData();
    return copyFile(replayDataRoot() / name, fx.dir.path / name);
}

std::filesystem::path metaFile(const std::filesystem::path& dir, const char* name, int count, std::int64_t seq0, std::int64_t ts0, int width, int height, std::int64_t step) {
    const auto path = dir / name;
    writeMeta(path, count, seq0, ts0, width, height, step);
    return path;
}

void checkMeanY(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames, double tolerance) {
    for(int index = 0; index < static_cast<int>(frames.size()); ++index) {
        CHECK(near(meanY(*frames[static_cast<std::size_t>(index)]), 16.0 + 7.0 * index, tolerance));
    }
}

void checkC3(const dai::ImgFrame& frame, int index) {
    checkTs(frame, 100 + index, 2000 + 100 * static_cast<std::int64_t>(index));
    const auto size = frame.transformation.getSize();
    CHECK(size.first == 1920);
    CHECK(size.second == 1080);
    const auto intrinsics = frame.transformation.getIntrinsicMatrix();
    CHECK(near(intrinsics[0][0], 960.0, 0.01));
    CHECK(near(intrinsics[1][1], 960.0, 0.01));
    CHECK(near(intrinsics[0][2], 960.0, 0.01));
    CHECK(near(intrinsics[1][2], 540.0, 0.01));
}

bool matrixIsIdentity(const std::array<std::array<float, 3>, 3>& matrix) {
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            const float expected = row == col ? 1.f : 0.f;
            if(std::fabs(matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] - expected) > 1e-3f) return false;
        }
    }
    return true;
}

bool bytesNear(const dai::ImgFrame& frame, int value, int tolerance) {
    const auto data = frame.getData();
    if(data.empty()) return false;
    for(auto byte : data) {
        if(std::abs(static_cast<int>(byte) - value) > tolerance) return false;
    }
    return true;
}

void checkUvNear(const dai::ImgFrame& frame, int value, int tolerance) {
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const int stride = std::max(static_cast<int>(frame.getStride()), width);
    const auto data = frame.getData();
    const int uv = frame.fb.p2Offset > 0 ? static_cast<int>(frame.fb.p2Offset) : stride * height;
    for(int row = 0; row < height / 2; ++row) {
        for(int col = 0; col < width; ++col) {
            const auto at = static_cast<std::size_t>(uv + row * stride + col);
            REQUIRE(at < data.size());
            CHECK(std::abs(static_cast<int>(data[at]) - value) <= tolerance);
        }
    }
}

std::filesystem::path customDir(HostReplay& fx, int width, int height, const std::vector<std::int64_t>& sequences, const std::vector<std::int64_t>& timestamps, FrameFill fill = {}, const std::vector<std::string>& names = {}) {
    fx.source = fx.dir.path / "src";
    makeDaiDir(fx.source, width, height, sequences, timestamps, fill, names);
    setSource(*fx.node, fx.source);
    setFast(*fx.node);
    return fx.source;
}

}  // namespace

TEST_CASE("A1 no source errors at start", "[replay][host]") {
    HostReplay fx;
    requireStartError(fx);
}

TEST_CASE("A2 missing source errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, fx.dir.path / "does_not_exist");
    requireStartError(fx);
}

TEST_CASE("A3 negative fps errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setFps(*fx.node, -1.f);
    requireStartError(fx, "fps");
}

TEST_CASE("A3b fps zero keeps recorded pacing", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_h264.mp4");
    const auto meta = metaFile(fx.dir.path, "meta30.mcap", 30, 100, 2000, 1920, 1080, 100);
    setSource(*fx.node, video);
    setMetadataFile(*fx.node, meta);
    setLoop(*fx.node, false);
    setStopPipelineAtEnd(*fx.node, false);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    const auto arrivals = takeTimed(fx.out, 30);
    std::vector<double> nominal(30);
    for(int index = 0; index < 30; ++index) nominal[static_cast<std::size_t>(index)] = 100.0 * index;
    checkArrivals(arrivals, nominal);
}

TEST_CASE("A4 nan fps errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setFps(*fx.node, std::numeric_limits<float>::quiet_NaN());
    requireStartError(fx, "fps");
}

TEST_CASE("A5 infinite fps errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setFps(*fx.node, std::numeric_limits<float>::infinity());
    requireStartError(fx, "fps");
}

TEST_CASE("A6 odd width errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setSize(*fx.node, 321, 240);
    requireStartError(fx, "size");
}

TEST_CASE("A7 odd height errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setSize(*fx.node, 320, 241);
    requireStartError(fx, "size");
}

TEST_CASE("A8 zero width errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setSize(*fx.node, 0, 240);
    requireStartError(fx);
}

TEST_CASE("A9 size 2x2 yields three frames", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setSize(*fx.node, 2, 2);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    for(int index = 0; index < 3; ++index) {
        CHECK(frames[static_cast<std::size_t>(index)]->getSequenceNum() == index);
        CHECK(frames[static_cast<std::size_t>(index)]->getWidth() == 2);
        CHECK(frames[static_cast<std::size_t>(index)]->getHeight() == 2);
        CHECK(frames[static_cast<std::size_t>(index)]->getType() == dai::ImgFrame::Type::NV12);
    }
}

TEST_CASE("A13 RAW16 output type errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setOutFrameType(*fx.node, dai::ImgFrame::Type::RAW16);
    requireStartError(fx);
}

TEST_CASE("A14 fluent setters give three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    fx.source = source;
    fx.node->setSource(source).setFps(1000).setLoop(false).setStopPipelineAtEnd(false);
    playDefault(fx);
}

TEST_CASE("A15 replay without setRunOnHost", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    playDefault(fx);
    fx.pipeline.stop();
}

TEST_CASE("A16 setRunOnHost true gives three frames", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setRunOnHost(*fx.node, true);
    playDefault(fx);
}

TEST_CASE("A17 setRunOnHost false errors at start", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setRunOnHost(*fx.node, false);
    requireStartError(fx);
}

TEST_CASE("A18 relative device source errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, "relative/dir", Location::DEVICE);
    requireStartError(fx);
}

TEST_CASE("A19 absolute device source errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, "/data/x", Location::DEVICE);
    requireStartError(fx);
}

TEST_CASE("A23 legacy setters replay the directory", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    fx.source = source;
    const auto meta = fx.dir.path / "x.mcap";
    writeMeta(meta, 3, 0, 0, 64, 48, 100);
    setReplayVideoFile(*fx.node, source);
    setReplayMetadataFile(*fx.node, meta);
    setFast(*fx.node);
    playDefault(fx);
    CHECK(fx.log.count(LineLevel::WARN, "x.mcap") == 1);
}

TEST_CASE("A24 legacy video setters match C3", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_h264.mp4");
    const auto meta = metaFile(fx.dir.path, "meta30.mcap", 30, 100, 2000, 1920, 1080, 100);
    setReplayVideoFile(*fx.node, video);
    setReplayMetadataFile(*fx.node, meta);
    setLoop(*fx.node, false);
    setStopPipelineAtEnd(*fx.node, false);
    setFps(*fx.node, 1000.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    for(int index = 0; index < 30; ++index) checkC3(*frames[static_cast<std::size_t>(index)], index);
}

TEST_CASE("A21 second start throws and the first run still yields frames", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    fx.pipeline.start();
    REQUIRE_THROWS(fx.pipeline.start());
    auto frames = takeFrames(fx.out, 3);
    checkDefault(frames, fx.source, 3);
}

TEST_CASE("A22 restart after stop reads the source again", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    playDefault(fx);
    fx.pipeline.stop();
    fx.pipeline.wait();
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    checkDefault(frames, fx.source, 3);
}

TEST_CASE("B1 directory frames are ImgFrame", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    fx.pipeline.start();
    for(int index = 0; index < 3; ++index) {
        std::shared_ptr<dai::ADatatype> message = record_test::getFor<dai::ImgFrame>(*fx.out, std::chrono::seconds(10));
        auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(message);
        REQUIRE(frame != nullptr);
        CHECK(frame->getSequenceNum() == index);
    }
}

TEST_CASE("B2 flat tar gives three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "flat.tar", source, TarKind::FLAT);
    setSource(*fx.node, archive);
    setFast(*fx.node);
    fx.source = source;
    playDefault(fx);
}

TEST_CASE("B3 gzip tar gives three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "flat.tar.gz", source, TarKind::GZ);
    setSource(*fx.node, archive);
    setFast(*fx.node);
    fx.source = source;
    playDefault(fx);
}

TEST_CASE("B4 tgz rename gives three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "flat.tar.gz", source, TarKind::GZ);
    const auto renamed = fx.dir.path / "flat.tgz";
    std::filesystem::rename(archive, renamed);
    setSource(*fx.node, renamed);
    setFast(*fx.node);
    fx.source = source;
    playDefault(fx);
}

TEST_CASE("B5 xz tar gives three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "flat.tar.xz", source, TarKind::XZ);
    setSource(*fx.node, archive);
    setFast(*fx.node);
    fx.source = source;
    playDefault(fx);
}

TEST_CASE("B6 nested tar gives three frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "cam";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "nested.tar", source, TarKind::NESTED);
    setSource(*fx.node, archive);
    setFast(*fx.node);
    fx.source = source;
    playDefault(fx);
}

TEST_CASE("B7 two directories in a tar error at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(1), rangeTs(1));
    const auto archive = makeTar(fx.dir.path / "two.tar", source, TarKind::TWO_DIRS);
    setSource(*fx.node, archive);
    requireStartError(fx);
}

TEST_CASE("B8 tar path escape errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(1), rangeTs(1));
    const auto archive = makeTar(fx.dir.path / "escape.tar", source, TarKind::ESCAPE);
    setSource(*fx.node, archive);
    requireStartError(fx, "escape.dai");
}

TEST_CASE("B9 absolute tar entry errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(1), rangeTs(1));
    const auto archive = makeTar(fx.dir.path / "abs.tar", source, TarKind::ABSOLUTE);
    setSource(*fx.node, archive);
    requireStartError(fx, "replay_abs_x.dai");
}

TEST_CASE("B10 tar symlink errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(1), rangeTs(1));
    const auto archive = makeTar(fx.dir.path / "link.tar", source, TarKind::SYMLINK);
    setSource(*fx.node, archive);
    requireStartError(fx, "cam_0.dai");
}

TEST_CASE("B11 empty tar errors at start", "[replay][host]") {
    HostReplay fx;
    const auto archive = makeTar(fx.dir.path / "empty.tar", fx.dir.path, TarKind::EMPTY);
    setSource(*fx.node, archive);
    requireStartError(fx);
}

TEST_CASE("B12 truncated tar errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(3), rangeTs(3));
    const auto archive = makeTar(fx.dir.path / "cut.tar.gz", source, TarKind::TRUNCATED);
    setSource(*fx.node, archive);
    requireStartError(fx);
}

TEST_CASE("B13 zip errors at start", "[replay][host]") {
    HostReplay fx;
    const auto archive = videoOf(fx, "cam.zip");
    setSource(*fx.node, archive);
    requireStartError(fx);
}

TEST_CASE("B14 empty directory errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "empty";
    std::filesystem::create_directories(source);
    setSource(*fx.node, source);
    requireStartError(fx);
}

TEST_CASE("B15 notes only errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "notes";
    std::filesystem::create_directories(source);
    std::ofstream(source / "notes.txt") << "notes";
    setSource(*fx.node, source);
    requireStartError(fx);
}

TEST_CASE("B16 vp9 errors at start", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_vp9.mp4");
    setSource(*fx.node, video);
    setFast(*fx.node);
    try {
        fx.pipeline.start();
        FAIL("expected an error at the start");
    } catch(const std::exception& error) {
        std::string text = error.what();
        for(char& ch : text) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        CHECK(text.find("vp9") != std::string::npos);
    }
}

TEST_CASE("B17 10-bit h265 errors at start", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_h265_10bit.mp4");
    setSource(*fx.node, video);
    setFast(*fx.node);
    requireStartErrorEither(fx, "10", "Main 10");
}

TEST_CASE("B18 upper-case dai names give two frames", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(2), rangeTs(2));
    std::filesystem::rename(source / "cam_0.dai", source / "CAM_0.DAI");
    std::filesystem::rename(source / "cam_1.dai", source / "CAM_1.DAI");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getSequenceNum() == 0);
    CHECK(frames[1]->getSequenceNum() == 1);
}

TEST_CASE("B19 ignored underscore file gives two frames", "[replay][host]") {
    HostReplay fx;
    const auto source = customDir(fx, 64, 48, rangeSeq(3), rangeTs(3));
    std::filesystem::rename(source / "cam_2.dai", source / "_2.dai");
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getSequenceNum() == 0);
    CHECK(frames[1]->getSequenceNum() == 1);
}

TEST_CASE("B20 stream cam_a gives two frames", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, rangeSeq(2), rangeTs(2), {}, {"cam_a_0.dai", "cam_a_1.dai"});
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getSequenceNum() == 0);
    CHECK(frames[1]->getSequenceNum() == 1);
}

TEST_CASE("B21 extra names are ignored", "[replay][host]") {
    HostReplay fx;
    const auto source = customDir(fx, 64, 48, rangeSeq(3), rangeTs(3));
    std::ofstream(source / "notes.txt") << "notes";
    std::ofstream(source / "cam.json") << "{}";
    std::ofstream(source / "._cam_9.dai", std::ios::binary) << "x";
    std::ofstream(source / ".cam_8.dai", std::ios::binary) << "y";
    std::filesystem::create_directories(source / "sub");
    std::filesystem::copy_file(source / "cam_0.dai", source / "sub" / "cam_9.dai");
    playDefault(fx);
}

TEST_CASE("B22 two streams error at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    makeDaiFrame(source / "dog_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    setSource(*fx.node, source);
    try {
        fx.pipeline.start();
        FAIL("expected an error at the start");
    } catch(const std::exception& error) {
        const std::string text = error.what();
        INFO(text);
        CHECK(text.find("cam") != std::string::npos);
        CHECK(text.find("dog") != std::string::npos);
    }
}

TEST_CASE("B23 source subdirectory selects that stream", "[replay][host]") {
    HostReplay fx;
    const auto root = fx.dir.path / "dir";
    makeDaiFrame(root / "cam" / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    const auto dog = makeDaiFrame(root / "dog" / "dog_0.dai", 64, 48, 7, 300, dai::ImgFrame::Type::NV12);
    setSource(*fx.node, root / "dog");
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 1);
    CHECK(frames[0]->getSequenceNum() == 7);
    CHECK(pixelsEqual(*frames[0], dog, 64, 48, dai::ImgFrame::Type::NV12));
}

TEST_CASE("B25 mixed dai and jpg errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    copyFile(videoOf(fx, "gray100.jpg"), source / "cam_1.jpg");
    setSource(*fx.node, source);
    requireStartError(fx);
}

TEST_CASE("B26 jpg and jpeg are one extension", "[replay][host]") {
    HostReplay fx;
    const auto jpg = videoOf(fx, "gray100.jpg");
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(jpg, source / "cam_0.jpg");
    copyFile(jpg, source / "cam_1.jpeg");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames.size() == 2);
}

TEST_CASE("B27 dai extension case is ignored", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, rangeSeq(2), rangeTs(2), {}, {"cam_0.DAI", "cam_1.dai"});
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getSequenceNum() == 0);
    CHECK(frames[1]->getSequenceNum() == 1);
}

TEST_CASE("B28 frames sort by sequence number", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {7, 10, 2}, {700, 1000, 200}, {}, {"cam_7.dai", "cam_010.dai", "cam_2.dai"});
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    checkTs(*frames[0], 2, 200);
    checkTs(*frames[1], 7, 700);
    checkTs(*frames[2], 10, 1000);
}

TEST_CASE("B29 sequence numbers above 32 bits keep order", "[replay][host]") {
    HostReplay fx;
    const std::int64_t first = 4294967296LL;
    const std::int64_t second = 4294967297LL;
    customDir(fx, 64, 48, {first, second}, {0, 100});
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getSequenceNum() == first);
    CHECK(frames[1]->getSequenceNum() == second);
}

TEST_CASE("B30 duplicate numeric names error at start", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {1, 2}, {100, 200}, {}, {"cam_1.dai", "cam_01.dai"});
    requireStartErrorEither(fx, "cam_01.dai", "cam_1.dai");
}

TEST_CASE("B34 imu dai errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiOther(source / "cam_0.dai", OtherKind::IMU);
    setSource(*fx.node, source);
    requireStartError(fx, "cam_0.dai");
}

TEST_CASE("B35 bitstream dai errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::BITSTREAM);
    setSource(*fx.node, source);
    requireStartError(fx, "cam_0.dai");
}

TEST_CASE("B36 h264 dai errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiOther(source / "cam_0.dai", OtherKind::H264);
    setSource(*fx.node, source);
    requireStartError(fx, "cam_0.dai");
}

TEST_CASE("B37 mjpeg dai frames decode", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiOther(source / "cam_0.dai", OtherKind::MJPEG, 5);
    makeDaiOther(source / "cam_1.dai", OtherKind::MJPEG, 6);
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(frames[1]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(frames[0]->getWidth() == 64);
    CHECK(frames[0]->getHeight() == 48);
    checkTs(*frames[0], 5, 500);
    checkTs(*frames[1], 6, 600);
    CHECK(near(meanY(*frames[0]), 98.0, 2.0));
    CHECK(near(meanY(*frames[1]), 98.0, 2.0));
}

TEST_CASE("B38 mixed sizes error at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    makeDaiFrame(source / "cam_1.dai", 32, 24, 1, 100, dai::ImgFrame::Type::NV12);
    setSource(*fx.node, source);
    setFast(*fx.node);
    requireStartError(fx, "cam_1.dai");
}

TEST_CASE("B39 setSize does not accept mixed dai sizes", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    makeDaiFrame(source / "cam_1.dai", 32, 24, 1, 100, dai::ImgFrame::Type::NV12);
    setSource(*fx.node, source);
    setSize(*fx.node, 32, 24);
    setFast(*fx.node);
    requireStartError(fx, "cam_1.dai");
}

TEST_CASE("B40 mixed types error at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::NV12);
    makeDaiFrame(source / "cam_1.dai", 64, 48, 1, 100, dai::ImgFrame::Type::GRAY8, FrameFill::plainByte(100));
    setSource(*fx.node, source);
    setFast(*fx.node);
    requireStartError(fx, "cam_1.dai");
}

TEST_CASE("B41 odd jpeg size errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "gray100_641.jpg"), source / "img_0.jpg");
    setSource(*fx.node, source);
    requireStartError(fx, "641");
}

TEST_CASE("B42 setSize resizes the odd jpeg", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "gray100_641.jpg"), source / "img_0.jpg");
    setSource(*fx.node, source);
    setSize(*fx.node, 640, 480);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 1);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(frames[0]->getWidth() == 640);
    CHECK(frames[0]->getHeight() == 480);
    CHECK(near(meanY(*frames[0]), 98.0, 2.0));
}

TEST_CASE("B43 jpeg source is yuv420p near 98", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "gray100.jpg"), source / "img_0.jpg");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 1);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(frames[0]->getWidth() == 64);
    CHECK(frames[0]->getHeight() == 48);
    CHECK(near(meanY(*frames[0]), 98.0, 2.0));
    checkUvNear(*frames[0], 128, 2);
}

TEST_CASE("B44 png source mean Y is near 100", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "gray100.png"), source / "img_0.png");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 1);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(near(meanY(*frames[0]), 100.0, 2.0));
}

TEST_CASE("B45 rgba png drops alpha", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "rgba.png"), source / "img_0.png");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 1);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(near(meanY(*frames[0]), 100.0, 2.0));
}

TEST_CASE("B46 16-bit png errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "png16.png"), source / "img_0.png");
    setSource(*fx.node, source);
    requireStartError(fx, "img_0.png");
}

TEST_CASE("B49 h264 mean Y follows 16 plus 7n", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    checkMeanY(frames, 2.0);
}

TEST_CASE("B50 ffv1 mean Y is exact", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_ffv1.avi"));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    checkMeanY(frames, 0.01);
}

TEST_CASE("B51 mjpeg mean Y follows 16 plus 7n", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_mjpeg.mp4"));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    checkMeanY(frames, 2.0);
}

TEST_CASE("B52 S1 order timestamps and finished", "[replay][host]") {
    HostReplay fx;
    const auto source = makeS1(fx.dir.path / "s1");
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 5);
    const std::int64_t sequences[] = {0, 1, 2, 5, 7};
    const std::int64_t times[] = {0, 100, 200, 500, 700};
    for(int index = 0; index < 5; ++index) {
        checkTs(*frames[static_cast<std::size_t>(index)], sequences[index], times[index]);
        const auto packed = packedOf(source / ("cam_" + std::to_string(sequences[index]) + ".dai"));
        CHECK(pixelsEqual(*frames[static_cast<std::size_t>(index)], packed, 64, 48, dai::ImgFrame::Type::NV12));
    }
    CHECK(fx.log.count(LineLevel::WARN, "cam.mcap") == 1);
    REQUIRE(fx.log.waitFor("Replay finished", std::chrono::seconds(10)));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    CHECK(fx.pipeline.isRunning());
    CHECK(fx.out->tryGet<dai::ImgFrame>() == nullptr);
}

TEST_CASE("C1 directory metadata warns and dai timestamps stay", "[replay][host]") {
    HostReplay fx;
    const auto source = customDir(fx, 64, 48, rangeSeq(3), rangeTs(3));
    writeMeta(source / "cam.mcap", 3, 0, 0, 64, 48, 100);
    playDefault(fx);
    CHECK(fx.log.count(LineLevel::WARN, "cam.mcap") == 1);
}

TEST_CASE("C2 other metadata file warns and dai timestamps stay", "[replay][host]") {
    HostReplay fx;
    const auto source = customDir(fx, 64, 48, rangeSeq(3), rangeTs(3));
    const auto meta = fx.dir.path / "other.mcap";
    writeMeta(meta, 3, 0, 0, 64, 48, 100);
    setMetadataFile(*fx.node, meta);
    playDefault(fx);
    CHECK(fx.log.count(LineLevel::WARN, "other.mcap") == 1);
}

TEST_CASE("C3 metadata overrides h264 timestamps", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "meta30.mcap", 30, 100, 2000, 1920, 1080, 100));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    for(int index = 0; index < 30; ++index) checkC3(*frames[static_cast<std::size_t>(index)], index);
}

TEST_CASE("C4 metadata one short errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "meta29.mcap", 29, 100, 2000, 1920, 1080, 100));
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C5 metadata one long errors at start", "[replay][host]") {
    HostReplay fx;
    const auto meta = fx.dir.path / "meta31.mcap";
    std::vector<record_test::McapEntry> entries;
    for(int index = 0; index < 30; ++index) entries.push_back({100 + index, 2000 + 100 * index, 1920, 1080});
    entries.push_back({130, 5000, 1920, 1080});
    record_test::makeMcap(meta, entries);
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, meta);
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C6 metadata size mismatch errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "meta720.mcap", 30, 100, 2000, 1280, 720, 100));
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C7 imu metadata errors at start", "[replay][host]") {
    HostReplay fx;
    const auto meta = fx.dir.path / "imu10.mcap";
    replay_test::makeImuMcap(meta, 10);
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, meta);
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C8 missing metadata errors at start", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, fx.dir.path / "no_such.mcap");
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C9 backwards timestamp errors at start", "[replay][host]") {
    HostReplay fx;
    const auto meta = fx.dir.path / "metaback.mcap";
    writeMeta(meta, 30, 100, 2000, 1920, 1080, 100, std::make_pair(10, 2500));
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, meta);
    setFast(*fx.node);
    requireStartError(fx);
}

TEST_CASE("C10 image metadata supplies sequence and time", "[replay][host]") {
    HostReplay fx;
    const auto jpg = videoOf(fx, "gray100.jpg");
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(jpg, source / "img_0.jpg");
    copyFile(jpg, source / "img_1.jpg");
    writeMeta(source / "img.mcap", 2, 40, 0, 64, 48, 100);
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    checkTs(*frames[0], 40, 0);
    checkTs(*frames[1], 41, 100);
}

TEST_CASE("C11 video fb fields", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "meta30.mcap", 30, 100, 2000, 1920, 1080, 100));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->fb.type == dai::ImgFrame::Type::YUV420p);
    CHECK(frame->fb.width == 1920);
    CHECK(frame->fb.height == 1080);
    CHECK(frame->fb.stride >= 1920);
}

TEST_CASE("C12 video without metadata uses defaults", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 30);
    REQUIRE(frames.size() == 30);
    std::int64_t previous = -1;
    for(int index = 0; index < 30; ++index) {
        const auto& frame = *frames[static_cast<std::size_t>(index)];
        CHECK(frame.getType() == dai::ImgFrame::Type::YUV420p);
        CHECK(frame.getSequenceNum() == index);
        const auto ts = timestampMs(frame.getTimestampDevice());
        CHECK(ts > previous);
        previous = ts;
        const auto size = frame.transformation.getSize();
        CHECK(size.first == 1920);
        CHECK(size.second == 1080);
        CHECK(matrixIsIdentity(frame.transformation.getTransformationMatrix()));
        CHECK(matrixIsIdentity(frame.transformation.getIntrinsicMatrix()));
        CHECK(frame.sourceFb.width == frame.fb.width);
        CHECK(frame.sourceFb.height == frame.fb.height);
        CHECK(frame.sourceFb.stride == frame.fb.stride);
        CHECK(frame.sourceFb.type == frame.fb.type);
        CHECK_FALSE(frame.getTimestampSystem().has_value());
    }
}

TEST_CASE("C13 loop continues sequence numbers", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setLoop(*fx.node, true);
    setStopPipelineAtEnd(*fx.node, false);
    setFps(*fx.node, 1000.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 60);
    for(int index = 30; index < 60; ++index) CHECK(frames[static_cast<std::size_t>(index)]->getSequenceNum() == index);
}

TEST_CASE("C14 first device timestamp is the steady epoch", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(timestampMs(frame->getTimestampDevice()) == 0);
}

TEST_CASE("C15 loop extrapolates recorded intervals", "[replay][host]") {
    // The listed tuples use the recorded mean interval. setFps(1000) would replace that interval, so fps stays unset.
    HostReplay fx;
    customDir(fx, 64, 48, {10, 11, 12}, {1000, 1100, 1300});
    setLoop(*fx.node, true);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 7);
    const std::pair<std::int64_t, std::int64_t> expected[] = {{10, 1000}, {11, 1100}, {12, 1300}, {13, 1450}, {14, 1550}, {15, 1750}, {16, 1900}};
    for(int index = 0; index < 7; ++index) checkTs(*frames[static_cast<std::size_t>(index)], expected[index].first, expected[index].second);
}

TEST_CASE("C16 set fps changes the loop boundary", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {10, 11, 12}, {1000, 1100, 1300});
    setLoop(*fx.node, true);
    setFps(*fx.node, 20.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 4);
    checkTs(*frames[3], 13, 1350);
}

TEST_CASE("C17 S1 loop pass one", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, makeS1(fx.dir.path / "s1"));
    setLoop(*fx.node, true);
    setStopPipelineAtEnd(*fx.node, false);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 10);
    const std::pair<std::int64_t, std::int64_t> pass1[] = {{8, 875}, {9, 975}, {10, 1075}, {13, 1375}, {15, 1575}};
    for(int index = 0; index < 5; ++index) checkTs(*frames[static_cast<std::size_t>(5 + index)], pass1[index].first, pass1[index].second);
}

TEST_CASE("C18 two-frame loop offset", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0, 1}, {0, 100});
    setLoop(*fx.node, true);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 7);
    for(int index = 0; index < 7; ++index) checkTs(*frames[static_cast<std::size_t>(index)], index, index * 100);
}

TEST_CASE("C19 decreasing sequence errors at start", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {5, 4}, {0, 100}, {}, {"cam_0.dai", "cam_1.dai"});
    requireStartError(fx);
}

TEST_CASE("C20 decreasing timestamp errors at start", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0, 1}, {500, 400});
    requireStartError(fx);
}

TEST_CASE("C21 equal timestamps are allowed", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0, 1}, {500, 500});
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames.size() == 2);
}

TEST_CASE("C22 one frame loop uses one thirtieth of a second", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0});
    setLoop(*fx.node, true);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[1]->getSequenceNum() == 1);
    CHECK(std::llabs(timestampMs(frames[1]->getTimestampDevice()) - 33) <= 1);
}

TEST_CASE("C23 resize updates the transformation", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 640, 480, rangeSeq(2), rangeTs(2));
    setSize(*fx.node, 320, 240);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    for(const auto& frame : frames) {
        CHECK(frame->getWidth() == 320);
        CHECK(frame->getHeight() == 240);
        const auto size = frame->transformation.getSize();
        CHECK(size.first == 320);
        CHECK(size.second == 240);
        const auto intrinsics = frame->transformation.getIntrinsicMatrix();
        CHECK(near(intrinsics[0][0], 160.0, 0.01));
        CHECK(near(intrinsics[1][1], 160.0, 0.01));
        CHECK(near(intrinsics[0][2], 160.0, 0.01));
        CHECK(near(intrinsics[1][2], 120.0, 0.01));
    }
}

TEST_CASE("D1 plain nv12 matches the file", "[replay][host]") {
    HostReplay fx;
    const auto fill = FrameFill::plainYuv(100, 128, 128);
    customDir(fx, 64, 48, {0}, {0}, fill);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::NV12);
    CHECK(pixelsEqual(*frame, packedOf(fx.source / "cam_0.dai"), 64, 48, dai::ImgFrame::Type::NV12));
}

TEST_CASE("D2 gray output is the Y plane", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::GRAY8);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::GRAY8);
    CHECK(bytesNear(*frame, 100, 0));
}

TEST_CASE("D3 rgb output is near 98", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::RGB888i);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::RGB888i);
    CHECK(bytesNear(*frame, 98, 2));
}

TEST_CASE("D4 bgr output is near 98", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::BGR888i);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::BGR888i);
    CHECK(bytesNear(*frame, 98, 2));
}

TEST_CASE("D5 yuv420p planes keep the plain values", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::YUV420p);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::YUV420p);
    CHECK(near(meanY(*frame), 100.0, 0.01));
    const auto data = frame->getData();
    const int width = 64;
    const int height = 48;
    const int ySize = width * height;
    for(int index = ySize; index < static_cast<int>(data.size()); ++index) CHECK(data[static_cast<std::size_t>(index)] == 128);
}

TEST_CASE("D6 nv12 output matches the file", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::NV12);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(pixelsEqual(*frame, packedOf(fx.source / "cam_0.dai"), 64, 48, dai::ImgFrame::Type::NV12));
}

TEST_CASE("D7 gray source to nv12 fills UV with 128", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::GRAY8, FrameFill::plainByte(77));
    makeDaiFrame(source / "cam_1.dai", 64, 48, 1, 100, dai::ImgFrame::Type::GRAY8, FrameFill::plainByte(77));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setOutFrameType(*fx.node, dai::ImgFrame::Type::NV12);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    for(const auto& frame : frames) {
        CHECK(frame->getType() == dai::ImgFrame::Type::NV12);
        CHECK(near(meanY(*frame), 77.0, 0.01));
        checkUvNear(*frame, 128, 0);
    }
}

TEST_CASE("D8 raw16 stays raw16", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    const auto first = makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    const auto second = makeDaiFrame(source / "cam_1.dai", 64, 48, 1, 100, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    setSource(*fx.node, source);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    CHECK(frames[0]->getType() == dai::ImgFrame::Type::RAW16);
    CHECK(frames[1]->getType() == dai::ImgFrame::Type::RAW16);
    CHECK(pixelsEqual(*frames[0], first, 64, 48, dai::ImgFrame::Type::RAW16));
    CHECK(pixelsEqual(*frames[1], second, 64, 48, dai::ImgFrame::Type::RAW16));
}

TEST_CASE("D9 raw16 to nv12 errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    makeDaiFrame(source / "cam_1.dai", 64, 48, 1, 100, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    setSource(*fx.node, source);
    setOutFrameType(*fx.node, dai::ImgFrame::Type::NV12);
    requireStartError(fx);
}

TEST_CASE("D10 raw16 setSize errors at start", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    makeDaiFrame(source / "cam_0.dai", 64, 48, 0, 0, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    makeDaiFrame(source / "cam_1.dai", 64, 48, 1, 100, dai::ImgFrame::Type::RAW16, FrameFill::plainByte(40));
    setSource(*fx.node, source);
    setSize(*fx.node, 32, 24);
    requireStartError(fx);
}

TEST_CASE("D11 jpeg to gray is near 98", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    std::filesystem::create_directories(source);
    copyFile(videoOf(fx, "gray100.jpg"), source / "img_0.jpg");
    setSource(*fx.node, source);
    setOutFrameType(*fx.node, dai::ImgFrame::Type::GRAY8);
    setFast(*fx.node);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::GRAY8);
    CHECK(bytesNear(*frame, 98, 2));
}

TEST_CASE("D12 two nodes convert independently", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setOutFrameType(*fx.node, dai::ImgFrame::Type::RGB888i);
    auto second = fx.pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*second);
    setSource(*second, source);
    setFast(*second);
    setOutFrameType(*second, dai::ImgFrame::Type::GRAY8);
    auto gray = second->out.createOutputQueue(16, true);
    fx.pipeline.start();
    auto frame = record_test::getFor<dai::ImgFrame>(*gray, std::chrono::seconds(10));
    REQUIRE(frame != nullptr);
    CHECK(frame->getType() == dai::ImgFrame::Type::GRAY8);
    CHECK(bytesNear(*frame, 100, 0));
}

TEST_CASE("D13 half size keeps plain yuv within 1", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 640, 480, rangeSeq(2), rangeTs(2), FrameFill::plainYuv(90, 100, 160));
    setSize(*fx.node, 320, 240);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    for(const auto& frame : frames) {
        CHECK(frame->getWidth() == 320);
        CHECK(frame->getHeight() == 240);
        CHECK(near(meanY(*frame), 90.0, 1.0));
    }
}

TEST_CASE("D14 same size does not resize", "[replay][host]") {
    HostReplay fx;
    const auto source = customDir(fx, 640, 480, rangeSeq(2), rangeTs(2));
    setSize(*fx.node, 640, 480);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    for(int index = 0; index < 2; ++index) {
        CHECK(pixelsEqual(*frames[static_cast<std::size_t>(index)], packedOf(source / ("cam_" + std::to_string(index) + ".dai")), 640, 480, dai::ImgFrame::Type::NV12));
    }
}

TEST_CASE("D15 width-only resize keeps Y", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 640, 480, rangeSeq(2), rangeTs(2), FrameFill::plainYuv(90, 100, 160));
    setSize(*fx.node, 320, 480);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 2);
    for(const auto& frame : frames) {
        CHECK(frame->getWidth() == 320);
        CHECK(frame->getHeight() == 480);
        CHECK(near(meanY(*frame), 90.0, 1.0));
    }
}

TEST_CASE("D16 fb stride covers the row", "[replay][host]") {
    HostReplay fx;
    const auto source = fx.frames(64, 48, 3);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    for(const auto& frame : frames) {
        CHECK(frame->fb.width == 64);
        CHECK(frame->fb.height == 48);
        CHECK(frame->fb.stride >= 64);
    }
    checkDefault(frames, source, 3);
}

TEST_CASE("D17 loop sequence has no gap", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 20);
    setLoop(*fx.node, true);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 40);
    for(int index = 0; index < 40; ++index) CHECK(frames[static_cast<std::size_t>(index)]->getSequenceNum() == index);
}

TEST_CASE("D18 predecode matches stream pixels", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_h264.mp4");
    setSource(*fx.node, video);
    setFast(*fx.node);
    fx.pipeline.start();
    auto streamed = takeFrames(fx.out, 30);
    fx.pipeline.stop();
    fx.pipeline.wait();
    HostReplay pre;
    setSource(*pre.node, video);
    setFast(*pre.node);
    setDecodeMode(*pre.node, DecodeMode::PRE_DECODE);
    pre.pipeline.start();
    auto decoded = takeFrames(pre.out, 30);
    REQUIRE(decoded.size() == streamed.size());
    for(std::size_t index = 0; index < decoded.size(); ++index) {
        CHECK(replay_test::pixelsWithin(*decoded[index], *streamed[index], 0));
    }
}

TEST_CASE("D19 predecode rgb matches D3", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {0}, {0}, FrameFill::plainYuv(100, 128, 128));
    setOutFrameType(*fx.node, dai::ImgFrame::Type::RGB888i);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    fx.pipeline.start();
    auto frame = takeFrames(fx.out, 1).front();
    CHECK(frame->getType() == dai::ImgFrame::Type::RGB888i);
    CHECK(bytesNear(*frame, 98, 2));
}

TEST_CASE("E1 ten frames at 10 fps", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 10);
    setFps(*fx.node, 10.f);
    fx.pipeline.start();
    std::vector<double> nominal(10);
    for(int index = 0; index < 10; ++index) nominal[static_cast<std::size_t>(index)] = 100.0 * index;
    checkArrivals(takeTimed(fx.out, 10), nominal);
}

TEST_CASE("E2 queue of one catches up after a hold", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 10);
    setFps(*fx.node, 10.f);
    fx.queueOf(1, true);
    fx.pipeline.start();
    std::vector<std::chrono::steady_clock::time_point> arrivals;
    std::vector<std::int64_t> sequences;
    for(int index = 0; index < 10; ++index) {
        if(index == 4) std::this_thread::sleep_for(std::chrono::milliseconds(350));
        auto frame = record_test::getFor<dai::ImgFrame>(*fx.out, std::chrono::seconds(10));
        REQUIRE(frame != nullptr);
        arrivals.push_back(std::chrono::steady_clock::now());
        sequences.push_back(frame->getSequenceNum());
    }
    for(int index = 0; index < 10; ++index) CHECK(sequences[static_cast<std::size_t>(index)] == index);
    const auto gap = [](const std::chrono::steady_clock::time_point& left, const std::chrono::steady_clock::time_point& right) {
        return std::chrono::duration<double, std::milli>(right - left).count();
    };
    CHECK(gap(arrivals[4], arrivals[5]) < 50.0);
    CHECK(gap(arrivals[5], arrivals[6]) < 50.0);
    CHECK(gap(arrivals[7], arrivals[8]) >= 90.0);
    CHECK(gap(arrivals[8], arrivals[9]) >= 90.0);
}

TEST_CASE("E3 recorded gaps without fps", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, rangeSeq(4), {0, 100, 300, 300});
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    const auto arrivals = takeTimed(fx.out, 4);
    std::vector<double> nominal{0, 100, 300, 333};
    checkArrivals(arrivals, nominal);
}

TEST_CASE("E4 loop boundary pacing", "[replay][host]") {
    HostReplay fx;
    customDir(fx, 64, 48, {10, 11, 12}, {1000, 1100, 1300});
    setLoop(*fx.node, true);
    setFps(*fx.node, 0.f);
    fx.pipeline.start();
    checkArrivals(takeTimed(fx.out, 6), {0, 100, 300, 450, 550, 750});
}

TEST_CASE("E5 container duration without metadata", "[replay][host]") {
    std::vector<double> spans;
    std::vector<double> arrivals;
    for(int repeat = 0; repeat < 3; ++repeat) {
        HostReplay fx;
        setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
        setLoop(*fx.node, false);
        setStopPipelineAtEnd(*fx.node, false);
        fx.pipeline.start();
        const auto start = std::chrono::steady_clock::now();
        auto frames = takeFrames(fx.out, 30);
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        spans.push_back(static_cast<double>(timestampMs(frames.back()->getTimestampDevice()) - timestampMs(frames.front()->getTimestampDevice())));
        arrivals.push_back(elapsed);
        fx.pipeline.stop();
    }
    std::sort(spans.begin(), spans.end());
    std::sort(arrivals.begin(), arrivals.end());
    CHECK(std::fabs(spans[1] - 966.7) <= 966.7 * 0.10);
    CHECK(std::fabs(arrivals[1] - 966.7) <= 966.7 * 0.25);
}

TEST_CASE("E6 fast fps does not drop for a slow consumer", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 20);
    setFps(*fx.node, 1000.f);
    fx.queueOf(1, true);
    fx.pipeline.start();
    const auto start = std::chrono::steady_clock::now();
    for(int index = 0; index < 20; ++index) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto frame = record_test::getFor<dai::ImgFrame>(*fx.out, std::chrono::seconds(10));
        REQUIRE(frame != nullptr);
        CHECK(frame->getSequenceNum() == index);
        if(index == 19) {
            const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            CHECK(std::fabs(elapsed - 950.0) <= 950.0 * 0.20);
        }
    }
}

TEST_CASE("E7 metadata at max rate finishes inside one second", "[replay][host]") {
    HostReplay fx;
    const auto video = videoOf(fx, "plain_h264_small.mp4");
    setSource(*fx.node, video);
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "small.mcap", 20, 0, 0, 64, 48, 100));
    setFps(*fx.node, 1000.f);
    setLoop(*fx.node, false);
    setStopPipelineAtEnd(*fx.node, false);
    fx.pipeline.start();
    const auto start = std::chrono::steady_clock::now();
    auto first = record_test::getFor<dai::ImgFrame>(*fx.out, std::chrono::seconds(10));
    REQUIRE(first != nullptr);
    for(int index = 1; index < 20; ++index) {
        REQUIRE(record_test::getFor<dai::ImgFrame>(*fx.out, std::chrono::seconds(10)) != nullptr);
    }
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    CHECK(elapsed < 1.0);
}

TEST_CASE("E8 set fps beats metadata intervals", "[replay][host]") {
    HostReplay fx;
    setSource(*fx.node, videoOf(fx, "plain_h264.mp4"));
    setMetadataFile(*fx.node, metaFile(fx.dir.path, "meta30.mcap", 30, 100, 2000, 1920, 1080, 100));
    setFps(*fx.node, 20.f);
    setLoop(*fx.node, false);
    setStopPipelineAtEnd(*fx.node, false);
    fx.pipeline.start();
    const auto arrivals = takeTimed(fx.out, 30);
    std::vector<double> gaps;
    for(std::size_t index = 1; index < arrivals.size(); ++index) {
        gaps.push_back(std::chrono::duration<double, std::milli>(arrivals[index] - arrivals[index - 1]).count());
        CHECK(gaps.back() + 0.01 >= 40.0);
    }
    const double mean = std::accumulate(gaps.begin(), gaps.end(), 0.0) / static_cast<double>(gaps.size());
    CHECK(std::fabs(mean - 50.0) <= 50.0 * 0.15);
}

TEST_CASE("E9 loop with stop at end still runs and warns nothing", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setLoop(*fx.node, true);
    setStopPipelineAtEnd(*fx.node, true);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 9);
    CHECK(frames.size() == 9);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    CHECK(fx.pipeline.isRunning());
    CHECK(fx.log.count(LineLevel::WARN) == 0);
}

TEST_CASE("E10 finished leaves the pipeline running", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    fx.pipeline.start();
    CHECK(takeFrames(fx.out, 3).size() == 3);
    REQUIRE(fx.log.waitFor("Replay finished", std::chrono::seconds(10)));
    CHECK(fx.log.count(LineLevel::INFO, "Replay finished") >= 1);
    CHECK(fx.pipeline.isRunning());
    CHECK(fx.out->tryGet<dai::ImgFrame>() == nullptr);
}

TEST_CASE("E11 stop at end stops the pipeline", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setStopPipelineAtEnd(*fx.node, true);
    fx.pipeline.start();
    CHECK(takeFrames(fx.out, 3).size() == 3);
    CHECK(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, std::chrono::seconds(10)));
    fx.pipeline.wait();
}

TEST_CASE("E12 the first finished node stops the pipeline", "[replay][host]") {
    HostReplay fx;
    fx.frames(64, 48, 3);
    setStopPipelineAtEnd(*fx.node, true);
    auto slow = fx.pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*slow);
    const auto source = fx.dir.path / "slow";
    makeDaiDir(source, 64, 48, rangeSeq(30), rangeTs(30));
    setSource(*slow, source);
    setLoop(*slow, false);
    setFps(*slow, 10.f);
    setStopPipelineAtEnd(*slow, true);
    slow->out.createOutputQueue(4, false);
    fx.pipeline.start();
    auto frames = takeFrames(fx.out, 3);
    REQUIRE(frames.size() == 3);
    const auto marked = std::chrono::steady_clock::now();
    CHECK(record_test::waitUntil([&] { return !fx.pipeline.isRunning(); }, std::chrono::milliseconds(1500)));
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - marked).count();
    CHECK(elapsed <= 1.5);
}

TEST_CASE("G1 persistent predecode keeps one decoded entry", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(100), rangeTs(100));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    fx.pipeline.start();
    REQUIRE(fx.log.waitFor("100/100", std::chrono::seconds(60)));
    CHECK(fx.log.count(LineLevel::INFO, "Replay decode") == 10);
    fx.pipeline.stop();
    const auto info = getHostCacheInfo();
    REQUIRE(info.entries.size() == 1);
    CHECK(info.entries[0].kind == CacheKind::DECODED);
    CHECK(info.entries[0].sourceHash == hashDir(source));
}

TEST_CASE("G2 second run reuses the decoded entry", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(100), rangeTs(100));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    fx.pipeline.start();
    REQUIRE(fx.log.waitFor("100/100", std::chrono::seconds(60)));
    fx.pipeline.stop();
    fx.pipeline.wait();
    const int before = fx.log.countSubstring("Replay decode");
    fx.pipeline.start();
    REQUIRE(fx.log.waitFor("Replay finished", std::chrono::seconds(30)));
    fx.pipeline.stop();
    CHECK(fx.log.countSubstring("Replay decode") == before);
    CHECK(getHostCacheInfo().entries.size() == 1);
}

TEST_CASE("G3 cache limit names the needed size", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    setHostCacheLimit(100000);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 640, 480, rangeSeq(10), rangeTs(10));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    requireStartError(fx, "4608000");
}

TEST_CASE("G4 exact cache limit starts", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    setHostCacheLimit(4608000);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 640, 480, rangeSeq(10), rangeTs(10));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    fx.pipeline.start();
    CHECK(fx.pipeline.isRunning());
    fx.pipeline.stop();
}

TEST_CASE("G5 limit one byte short errors at start", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    setHostCacheLimit(4607999);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 640, 480, rangeSeq(10), rangeTs(10));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    requireStartError(fx);
}

TEST_CASE("G6 clear host cache removes the entry", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(100), rangeTs(100));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    setPersistentUpload(*fx.node, true);
    fx.pipeline.start();
    REQUIRE(fx.log.waitFor("100/100", std::chrono::seconds(60)));
    fx.pipeline.stop();
    clearHostCache();
    CHECK(getHostCacheInfo().entries.empty());
}

TEST_CASE("G7 default upload drops the entry at stop", "[replay][host]") {
    HostReplay fx;
    HostCacheGuard guard(&fx.pipeline);
    const auto source = fx.dir.path / "src";
    makeDaiDir(source, 64, 48, rangeSeq(100), rangeTs(100));
    setSource(*fx.node, source);
    setFast(*fx.node);
    setDecodeMode(*fx.node, DecodeMode::PRE_DECODE);
    fx.pipeline.start();
    REQUIRE(takeFrames(fx.out, 1).size() == 1);
    const auto during = getHostCacheInfo();
    REQUIRE(during.entries.size() == 1);
    CHECK(during.entries[0].kind == CacheKind::DECODED);
    fx.pipeline.stop();
    CHECK(getHostCacheInfo().entries.empty());
}
