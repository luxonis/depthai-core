#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/RecordControl.hpp"
#include "depthai/pipeline/datatype/RecordStatus.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

#include <opencv2/videoio.hpp>

namespace {

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        path = std::filesystem::temp_directory_path() / ("dai-stop-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
};

std::shared_ptr<dai::ImgFrame> makeFrame(std::int64_t sequence) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(16);
    frame->setHeight(16);
    frame->setType(dai::ImgFrame::Type::NV12);
    frame->setData(std::vector<std::uint8_t>(16 * 16 * 3 / 2, static_cast<std::uint8_t>(sequence + 1)));
    frame->setSequenceNum(sequence);
    frame->setTimestamp(std::chrono::steady_clock::now());
    frame->setTimestampDevice(frame->getTimestamp());
    return frame;
}

bool waitForFile(const std::filesystem::path& path) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while(std::chrono::steady_clock::now() < deadline) {
        std::error_code error;
        if(std::filesystem::is_regular_file(path, error)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

std::filesystem::path framePath(const std::filesystem::path& dir, std::int64_t sequence) {
    std::ostringstream name;
    name << "cam_" << std::setw(8) << std::setfill('0') << sequence << ".dai";
    return dir / "rec" / "cam" / name.str();
}

// pipeline.stop() on the calling thread. Exit the process if it is still blocked after 10s
// so a wedged record or replay thread cannot hang the runner.
void stopWithin(dai::Pipeline& pipeline) {
    std::atomic<bool> done{false};
    std::exception_ptr error;
    std::thread worker([&] {
        try {
            pipeline.stop();
        } catch(...) {
            error = std::current_exception();
        }
        done = true;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while(!done.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!done.load()) std::_Exit(1);
    worker.join();
    if(error) std::rethrow_exception(error);
}

bool writeFrames(cv::VideoWriter& writer) {
    if(!writer.isOpened()) return false;
    cv::Mat frame(48, 64, CV_8UC3, cv::Scalar(32, 64, 96));
    for(int index = 0; index < 12; ++index) {
        frame.setTo(cv::Scalar(index * 10, 64, 96));
        writer.write(frame);
    }
    writer.release();
    return true;
}

std::filesystem::path writeTinyVideo(const std::filesystem::path& dir) {
    const auto mp4 = dir / "clip.mp4";
    cv::VideoWriter writer;
    writer.open(mp4.string(), cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30.0, cv::Size(64, 48));
    if(!writer.isOpened()) writer.open(mp4.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(64, 48));
    if(writeFrames(writer)) {
        std::error_code error;
        if(std::filesystem::is_regular_file(mp4, error) && std::filesystem::file_size(mp4, error) > 100) return mp4;
    }
    const auto avi = dir / "clip.avi";
    writer.open(avi.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(64, 48));
    if(writeFrames(writer)) {
        std::error_code error;
        if(std::filesystem::is_regular_file(avi, error) && std::filesystem::file_size(avi, error) > 100) return avi;
    }
    const std::string command =
        "ffmpeg -y -loglevel error -f lavfi -i testsrc=size=64x48:rate=30 -frames:v 12 -c:v libx264 -pix_fmt yuv420p \"" + mp4.string() + "\"";
    if(std::system(command.c_str()) == 0) {
        std::error_code error;
        if(std::filesystem::is_regular_file(mp4, error) && std::filesystem::file_size(mp4, error) > 100) return mp4;
    }
    return {};
}

class StatusSink : public dai::node::CustomThreadedNode<StatusSink> {
   public:
    constexpr static const char* NAME = "StatusSink";
    // Stays open across stop() so a blocked status.send is released by overflow, not by close().
    Input input{*this, {"input", "", true, 4, {{{dai::DatatypeEnum::RecordStatus, false}}}, false}};
    std::atomic<bool> keepOpen{true};

    void stop() override {
        keepOpen = false;
    }

    void run() override {
        while(keepOpen.load() && mainLoop()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};

}  // namespace

TEST_CASE("LockingQueue one-shot overflow leaves max size unchanged") {
    dai::LockingQueue<int> queue(2, true);
    REQUIRE(queue.push(1));
    REQUIRE(queue.push(2));
    REQUIRE_FALSE(queue.tryWaitAndPush(3, std::chrono::milliseconds(30)));
    REQUIRE(queue.getSize() == 2u);
    REQUIRE(queue.getMaxSize() == 2u);

    std::atomic<bool> pushed{false};
    std::thread blocked([&] {
        if(queue.push(3)) pushed = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    REQUIRE_FALSE(pushed.load());
    queue.allowOneOverflow();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(!pushed.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if(!pushed.load()) std::_Exit(1);
    blocked.join();
    REQUIRE(queue.getSize() == 3u);
    REQUIRE(queue.getMaxSize() == 2u);
    REQUIRE_FALSE(queue.tryWaitAndPush(4, std::chrono::milliseconds(30)));
    queue.allowOneOverflow();
    REQUIRE(queue.tryWaitAndPush(4, std::chrono::milliseconds(200)));
    REQUIRE(queue.getSize() == 4u);
    REQUIRE(queue.getMaxSize() == 2u);
    queue.allowOneOverflow();
    queue.clearOverflow();
    REQUIRE_FALSE(queue.tryWaitAndPush(5, std::chrono::milliseconds(30)));
    REQUIRE(queue.getSize() == 4u);

    dai::LockingQueue<int> dropping(1, false);
    dropping.allowOneOverflow();
    REQUIRE(dropping.push(1));
    REQUIRE(dropping.push(2));
    REQUIRE(dropping.getSize() == 1u);
    dropping.setBlocking(true);
    REQUIRE(dropping.tryWaitAndPush(3, std::chrono::milliseconds(200)));
    REQUIRE(dropping.getSize() == 2u);
    REQUIRE(dropping.getMaxSize() == 1u);
    REQUIRE_FALSE(dropping.tryWaitAndPush(4, std::chrono::milliseconds(30)));
}

TEST_CASE("pipeline stop returns when a blocking RecordVideo status queue is full") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setRunOnHost(true);
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("rec");
    record->setStreamName("cam");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(0);
    auto input = record->input.createInputQueue(64, true);
    auto control = record->inputControl.createInputQueue(64, true);
    auto status = record->status.createOutputQueue(16, true);
    pipeline.start();
    REQUIRE(status->getMaxSize() == 16u);

    for(int segment = 0; segment < 8; ++segment) {
        control->send(dai::RecordControl::start(1, -1));
        input->send(makeFrame(segment));
        REQUIRE(waitForFile(framePath(dir.path, segment)));
    }
    const auto fullDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while(std::chrono::steady_clock::now() < fullDeadline && status->getSize() < 16u) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(status->getSize() == 16u);
    REQUIRE(status->getMaxSize() == 16u);

    control->send(dai::RecordControl::start(1, -1));
    input->send(makeFrame(8));
    const auto blockDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(std::chrono::steady_clock::now() < blockDeadline) {
        const auto blocked = record->getStatus();
        if(blocked && blocked->state == dai::RecordStatus::State::STARTED && blocked->segment == 8 && status->getSize() == 16u) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto blocked = record->getStatus();
    REQUIRE(blocked.has_value());
    REQUIRE(blocked->state == dai::RecordStatus::State::STARTED);
    REQUIRE(blocked->segment == 8);
    REQUIRE(status->getSize() == 16u);

    stopWithin(pipeline);
    REQUIRE(status->getMaxSize() == 16u);

    int messages = 0;
    bool sawStopped = false;
    bool threw = false;
    std::string error;
    try {
        while(true) {
            auto message = status->tryGet<dai::RecordStatus>();
            if(!message) break;
            ++messages;
            if(message->state == dai::RecordStatus::State::STOPPED || message->state == dai::RecordStatus::State::ERROR) sawStopped = true;
        }
    } catch(const dai::MessageQueue::QueueException& ex) {
        threw = true;
        error = ex.what();
    }
    REQUIRE(threw);
    REQUIRE(error == "MessageQueue was closed");
    REQUIRE(sawStopped);
    REQUIRE(messages > 16);
}

TEST_CASE("pipeline stop returns when a blocking ReplayVideo output queue is full") {
    TempDir dir;
    const auto video = writeTinyVideo(dir.path);
    REQUIRE_FALSE(video.empty());

    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setRunOnHost(true);
    replay->setReplayVideoFile(video);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    replay->setLoop(true);
    replay->setFps(200.f);
    auto queue = replay->out.createOutputQueue(4, true);
    pipeline.start();
    REQUIRE(queue->getMaxSize() == 4u);
    const auto fillDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while(std::chrono::steady_clock::now() < fillDeadline && queue->getSize() < 4u) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto filled = queue->getSize();
    REQUIRE(filled == 4u);

    stopWithin(pipeline);
    REQUIRE(queue->getMaxSize() == 4u);

    unsigned got = 0;
    bool threw = false;
    std::string error;
    try {
        while(true) {
            auto frame = queue->tryGet<dai::ImgFrame>();
            if(!frame) break;
            ++got;
        }
    } catch(const dai::MessageQueue::QueueException& ex) {
        threw = true;
        error = ex.what();
    }
    REQUIRE(got == filled);
    REQUIRE(threw);
    REQUIRE(error == "MessageQueue was closed");
}

TEST_CASE("pipeline stop returns when a Node::Input linked to RecordVideo status is full") {
    TempDir dir;
    dai::Pipeline pipeline(false);
    auto record = pipeline.create<dai::node::RecordVideo>();
    auto sink = pipeline.create<StatusSink>();
    record->setRunOnHost(true);
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(dir.path);
    record->setRecordingName("rec");
    record->setStreamName("cam");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(0);
    record->status.link(sink->input);
    auto input = record->input.createInputQueue(64, true);
    auto control = record->inputControl.createInputQueue(64, true);
    pipeline.start();
    REQUIRE(sink->input.getMaxSize() == 4u);

    for(int segment = 0; segment < 2; ++segment) {
        control->send(dai::RecordControl::start(1, -1));
        input->send(makeFrame(segment));
        REQUIRE(waitForFile(framePath(dir.path, segment)));
    }
    const auto fullDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while(std::chrono::steady_clock::now() < fullDeadline && sink->input.getSize() < 4u) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(sink->input.getSize() == 4u);

    control->send(dai::RecordControl::start(1, -1));
    input->send(makeFrame(2));
    const auto blockDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(std::chrono::steady_clock::now() < blockDeadline) {
        const auto blocked = record->getStatus();
        if(blocked && blocked->state == dai::RecordStatus::State::STARTED && blocked->segment == 2 && sink->input.getSize() == 4u) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto blocked = record->getStatus();
    REQUIRE(blocked.has_value());
    REQUIRE(blocked->state == dai::RecordStatus::State::STARTED);
    REQUIRE(blocked->segment == 2);
    REQUIRE(sink->input.getSize() == 4u);
    REQUIRE(sink->input.getMaxSize() == 4u);

    stopWithin(pipeline);
    REQUIRE(sink->input.getMaxSize() == 4u);
    REQUIRE(sink->input.getSize() > 4u);
}
