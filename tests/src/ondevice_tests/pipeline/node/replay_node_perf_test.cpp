extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

#include <thread>

#include "../../../helpers/replay_test_support.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"

using record_test::deleteRecording;
using record_test::timestampMs;
using replay_test::DecodeMode;
using replay_test::DeviceReplay;
using replay_test::Location;
using replay_test::makeDaiDir;
using replay_test::makeDeviceSource;
using replay_test::rangeSeq;
using replay_test::rangeTs;
using replay_test::replayDataAvailable;
using replay_test::replayDataRoot;
using replay_test::setDecodeMode;
using replay_test::setFps;
using replay_test::setLoop;
using replay_test::setSource;
using replay_test::setStopPipelineAtEnd;

namespace {

std::filesystem::path perfFile(const char* name) {
    if(!replayDataAvailable()) SKIP("needs the generated data");
    const auto path = replayDataRoot() / name;
    if(!std::filesystem::is_regular_file(path)) SKIP("needs the generated data");
    return path;
}

std::vector<std::shared_ptr<dai::ImgFrame>> readFrames(dai::MessageQueue& queue, int count) {
    std::vector<std::shared_ptr<dai::ImgFrame>> frames;
    frames.reserve(static_cast<std::size_t>(count));
    for(int index = 0; index < count; ++index) {
        auto frame = record_test::getFor<dai::ImgFrame>(queue, std::chrono::seconds(30));
        if(frame == nullptr) break;
        frames.push_back(frame);
    }
    return frames;
}

void checkSpan(const std::vector<std::shared_ptr<dai::ImgFrame>>& frames) {
    REQUIRE(frames.size() == 600);
    const double span = static_cast<double>(timestampMs(frames.back()->getTimestampDevice()) - timestampMs(frames.front()->getTimestampDevice())) / 1000.0;
    CHECK(std::fabs(span - 19.97) <= 19.97 * 0.02);
}

}  // namespace

TEST_CASE("VD1 predecode loop stays at 60 fps", "[replay][perf]") {
    DeviceReplay fx;
    const auto host = fx.dir.path / "src";
    makeDaiDir(host, 4000, 3000, rangeSeq(100), rangeTs(100));
    const auto devicePath = makeDeviceSource(fx.device, host);
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, devicePath, Location::DEVICE);
    setDecodeMode(*replay, DecodeMode::PRE_DECODE);
    setFps(*replay, 1000.f);
    setLoop(*replay, true);
    setStopPipelineAtEnd(*replay, false);
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setOutputSize(1920, 1080, dai::ImageManipConfig::ResizeMode::STRETCH);
    replay->out.link(manip->inputImage);
    auto bench = pipeline.create<dai::node::BenchmarkIn>();
    bench->sendReportEveryNMessages(600);
    manip->out.link(bench->input);
    auto reportQueue = bench->report.createOutputQueue(4, false);
    auto sys = pipeline.create<dai::node::SystemLogger>();
    sys->setRate(1.f);
    auto sysQueue = sys->out.createOutputQueue(16, false);
    pipeline.start();
    std::vector<float> samples;
    std::shared_ptr<dai::BenchmarkReport> report;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
    while(std::chrono::steady_clock::now() < deadline && report == nullptr) {
        if(auto message = sysQueue->tryGet()) {
            if(auto cpu = std::dynamic_pointer_cast<dai::SystemInformationRVC4>(message))
                samples.push_back(cpu->cpuAvgUsage.average);
            else if(auto cpu = std::dynamic_pointer_cast<dai::SystemInformation>(message))
                samples.push_back(cpu->leonCssCpuUsage.average);
        }
        report = reportQueue->tryGet<dai::BenchmarkReport>();
        if(report == nullptr) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    REQUIRE(report != nullptr);
    CHECK(report->fps >= 60.f);
    double mean = 0.0;
    for(float sample : samples) mean += sample;
    if(!samples.empty()) mean /= static_cast<double>(samples.size());
    std::cout << "VD1 benchmark fps " << report->fps << " cpu mean " << mean << " samples " << samples.size() << std::endl;
    pipeline.stop();
    deleteRecording(*fx.device, std::filesystem::path(devicePath).parent_path().filename().string());
}

TEST_CASE("VD2 4k h264 device time has no late frame", "[replay][perf]") {
    const auto source = perfFile("perf_4k_h264.mp4");
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setDecodeMode(*replay, DecodeMode::STREAM);
    setLoop(*replay, false);
    setStopPipelineAtEnd(*replay, false);
    auto bench = pipeline.create<dai::node::BenchmarkIn>();
    bench->sendReportEveryNMessages(600);
    replay->out.link(bench->input);
    auto out = replay->out.createOutputQueue(4, false);
    pipeline.start();
    const auto frames = readFrames(*out, 600);
    pipeline.stop();
    REQUIRE(frames.size() == 600);
    double sum = 0.0;
    double worst = 0.0;
    for(int index = 1; index < 600; ++index) {
        const double gap = static_cast<double>(timestampMs(frames[static_cast<std::size_t>(index)]->getTimestampDevice())
                                               - timestampMs(frames[static_cast<std::size_t>(index - 1)]->getTimestampDevice()));
        sum += gap;
        if(gap > worst) worst = gap;
    }
    const double mean = sum / 599.0;
    std::cout << "VD2 mean tsDevice gap ms " << mean << " worst " << worst << std::endl;
    CHECK(std::fabs(mean - 33.3) <= 33.3 * 0.02);
    CHECK(worst <= 38.3);
}

TEST_CASE("VD3 ffv1 span is 19.97 seconds", "[replay][perf]") {
    const auto source = perfFile("perf_1080_ffv1.avi");
    DeviceReplay fx;
    auto& pipeline = fx.begin();
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    record_test::watchNode(*replay);
    setSource(*replay, source);
    setDecodeMode(*replay, DecodeMode::STREAM);
    setLoop(*replay, false);
    setStopPipelineAtEnd(*replay, false);
    auto out = replay->out.createOutputQueue(4, false);
    pipeline.start();
    const auto frames = readFrames(*out, 600);
    pipeline.stop();
    const double span =
        frames.size() < 2 ? 0.0
                          : static_cast<double>(timestampMs(frames.back()->getTimestampDevice()) - timestampMs(frames.front()->getTimestampDevice())) / 1000.0;
    std::cout << "VD3 tsDevice span s " << span << std::endl;
    checkSpan(frames);
}

TEST_CASE("VD4 three cameras span 19.97 seconds", "[replay][perf]") {
    const auto sourceA = perfFile("perf_1080_h264_a.mp4");
    const auto sourceB = perfFile("perf_1080_h264_b.mp4");
    const auto sourceC = perfFile("perf_1080_h264_c.mp4");
    DeviceReplay fx;
    if(fx.device->getConnectedCameras().size() < 3) SKIP("fewer than three cameras");
    auto& pipeline = fx.begin();
    const std::filesystem::path sources[] = {sourceA, sourceB, sourceC};
    const dai::CameraBoardSocket sockets[] = {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C};
    std::shared_ptr<dai::MessageQueue> queues[3];
    for(int index = 0; index < 3; ++index) {
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        record_test::watchNode(*replay);
        setSource(*replay, sources[index]);
        setLoop(*replay, false);
        setStopPipelineAtEnd(*replay, false);
        auto camera = replay_test::buildCamera(pipeline, sockets[index], *replay);
        auto* output = camera->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
        queues[index] = output->createOutputQueue(4, false);
    }
    pipeline.start();
    std::vector<std::shared_ptr<dai::ImgFrame>> frames[3];
    std::thread readers[3];
    for(int index = 0; index < 3; ++index) {
        readers[index] = std::thread([&frames, &queues, index] { frames[index] = readFrames(*queues[index], 600); });
    }
    for(auto& reader : readers) reader.join();
    pipeline.stop();
    for(int index = 0; index < 3; ++index) {
        const double span =
            frames[index].size() < 2
                ? 0.0
                : static_cast<double>(timestampMs(frames[index].back()->getTimestampDevice()) - timestampMs(frames[index].front()->getTimestampDevice()))
                      / 1000.0;
        std::cout << "VD4 camera " << index << " tsDevice span s " << span << std::endl;
        checkSpan(frames[index]);
    }
}
