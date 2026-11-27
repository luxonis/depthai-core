#pragma once

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <unistd.h>
#include <utility>
#include <vector>

#include "utility/Platform.hpp"

#include <opencv2/videoio.hpp>
#include <spdlog/sinks/base_sink.h>

#include "depthai/depthai.hpp"
#include "mcap/reader.hpp"
#include "record_test_api.hpp"
#include "record_test_data.hpp"
#include "utility/Logging.hpp"

namespace record_test {

template <typename T>
inline std::shared_ptr<T> getFor(dai::MessageQueue& queue, std::chrono::milliseconds timeout) {
    bool timedOut = false;
    auto message = queue.get<T>(timeout, timedOut);
    if(timedOut || !message) return nullptr;
    return message;
}

inline bool waitUntil(const std::function<bool()>& pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        if(pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

class TempDir {
   public:
    std::filesystem::path path;
    TempDir() {
        path = dai::platform::getTempPath() / ("dai-record-cat-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
};

inline std::string uniqueName(const std::string& caseId) {
    return "test_" + caseId + "_" + std::to_string(static_cast<long long>(::getpid()));
}

inline bool endsWith(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline std::int64_t timestampMs(const std::chrono::steady_clock::time_point& when) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(when.time_since_epoch()).count();
}

inline std::string frameFileName(const std::string& stream, std::int64_t sequence) {
    std::ostringstream out;
    out << stream << '_' << std::setw(8) << std::setfill('0') << sequence << ".dai";
    return out.str();
}

inline bool isFrameFile(const std::string& name, const std::string& stream) {
    const std::string prefix = stream + "_";
    const std::string suffix = ".dai";
    if(name.size() < prefix.size() + 8 + suffix.size()) return false;
    if(name.compare(0, prefix.size(), prefix) != 0) return false;
    if(!endsWith(name, suffix)) return false;
    const auto digits = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
    if(digits.size() < 8) return false;
    for(char ch : digits) {
        if(ch < '0' || ch > '9') return false;
    }
    return true;
}

inline std::vector<std::string> allFiles(const std::filesystem::path& dir) {
    std::vector<std::string> names;
    if(!std::filesystem::exists(dir)) return names;
    for(const auto& entry : std::filesystem::directory_iterator(dir)) {
        if(entry.is_regular_file()) names.push_back(entry.path().filename().string());
    }
    std::sort(names.begin(), names.end());
    return names;
}

inline int fileCount(const std::filesystem::path& dir, const std::string& stream = "cam") {
    int count = 0;
    for(const auto& name : allFiles(dir)) {
        if(isFrameFile(name, stream)) ++count;
    }
    return count;
}

inline std::uint64_t fileSizeSum(const std::filesystem::path& dir) {
    std::uint64_t sum = 0;
    if(!std::filesystem::exists(dir)) return 0;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
        if(entry.is_regular_file()) sum += static_cast<std::uint64_t>(entry.file_size());
    }
    return sum;
}

enum class LineLevel { TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OTHER };

inline LineLevel fromSpd(spdlog::level::level_enum level) {
    switch(level) {
        case spdlog::level::trace:
            return LineLevel::TRACE;
        case spdlog::level::debug:
            return LineLevel::DEBUG;
        case spdlog::level::info:
            return LineLevel::INFO;
        case spdlog::level::warn:
            return LineLevel::WARN;
        case spdlog::level::err:
            return LineLevel::ERROR;
        case spdlog::level::critical:
            return LineLevel::CRITICAL;
        default:
            return LineLevel::OTHER;
    }
}

inline LineLevel fromDai(dai::LogLevel level) {
    switch(level) {
        case dai::LogLevel::TRACE:
            return LineLevel::TRACE;
        case dai::LogLevel::DEBUG:
            return LineLevel::DEBUG;
        case dai::LogLevel::INFO:
            return LineLevel::INFO;
        case dai::LogLevel::WARN:
            return LineLevel::WARN;
        case dai::LogLevel::ERR:
            return LineLevel::ERROR;
        case dai::LogLevel::CRITICAL:
            return LineLevel::CRITICAL;
        default:
            return LineLevel::OTHER;
    }
}

struct LogLine {
    LineLevel level;
    std::string text;
};

struct LineSink {
    mutable std::mutex linesMutex;
    std::vector<LogLine> lines;
};

// Node loggers are private stdout loggers (ThreadedNode), and a few lines use
// spdlog::warn directly. Both show up on stdout once the node level is debug.
// dai::Logging also writes to stdout, so one stdout tee sees every host line
// once. A sink on dai::Logging alone misses the node lines.
class StdoutTap {
   public:
    static StdoutTap& instance() {
        static StdoutTap tap;
        return tap;
    }

    void attach(LineSink* sink) {
        std::lock_guard<std::mutex> lock(mutex);
        const bool first = sinks.empty();
        sinks.push_back(sink);
        if(first) startUnlocked();
    }

    void detach(LineSink* sink) {
        std::thread toJoin;
        int saved = -1;
        int readEnd = -1;
        {
            std::lock_guard<std::mutex> lock(mutex);
            sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
            if(sinks.empty() && reader.joinable()) {
                std::fflush(stdout);
                if(savedStdout >= 0) {
                    ::dup2(savedStdout, STDOUT_FILENO);
                    saved = savedStdout;
                    savedStdout = -1;
                }
                readEnd = readFd;
                readFd = -1;
                toJoin = std::move(reader);
            }
        }
        if(toJoin.joinable()) toJoin.join();
        if(saved >= 0) ::close(saved);
        if(readEnd >= 0) ::close(readEnd);
    }

   private:
    StdoutTap() = default;
    ~StdoutTap() = default;
    StdoutTap(const StdoutTap&) = delete;
    StdoutTap& operator=(const StdoutTap&) = delete;

    static std::string stripAnsi(const std::string& in) {
        std::string out;
        out.reserve(in.size());
        for(std::size_t i = 0; i < in.size(); ++i) {
            if(in[i] == '\033' && i + 1 < in.size() && in[i + 1] == '[') {
                i += 2;
                while(i < in.size()) {
                    const unsigned char ch = static_cast<unsigned char>(in[i]);
                    if(ch >= 0x40 && ch <= 0x7e) break;
                    ++i;
                }
                continue;
            }
            if(in[i] != '\r') out.push_back(in[i]);
        }
        return out;
    }

    static LineLevel levelOf(const std::string& line) {
        if(line.find("[trace]") != std::string::npos) return LineLevel::TRACE;
        if(line.find("[debug]") != std::string::npos) return LineLevel::DEBUG;
        if(line.find("[info]") != std::string::npos) return LineLevel::INFO;
        if(line.find("[warning]") != std::string::npos) return LineLevel::WARN;
        if(line.find("[error]") != std::string::npos) return LineLevel::ERROR;
        if(line.find("[critical]") != std::string::npos) return LineLevel::CRITICAL;
        return LineLevel::OTHER;
    }

    void publish(const std::string& raw) {
        const std::string text = stripAnsi(raw);
        if(text.empty()) return;
        const LogLine line{levelOf(text), text};
        std::vector<LineSink*> copy;
        {
            std::lock_guard<std::mutex> lock(mutex);
            copy = sinks;
        }
        for(LineSink* sink : copy) {
            std::lock_guard<std::mutex> lock(sink->linesMutex);
            sink->lines.push_back(line);
        }
    }

    void loop(int fd, int echoFd) {
        std::string pending;
        char buffer[4096];
        while(true) {
            const ssize_t got = ::read(fd, buffer, sizeof(buffer));
            if(got < 0) {
                if(errno == EINTR) continue;
                break;
            }
            if(got == 0) break;
            if(echoFd >= 0) {
                std::size_t left = static_cast<std::size_t>(got);
                const char* data = buffer;
                while(left > 0) {
                    const ssize_t wrote = ::write(echoFd, data, left);
                    if(wrote < 0) {
                        if(errno == EINTR) continue;
                        break;
                    }
                    data += wrote;
                    left -= static_cast<std::size_t>(wrote);
                }
            }
            pending.append(buffer, static_cast<std::size_t>(got));
            std::size_t start = 0;
            while(true) {
                const auto nl = pending.find('\n', start);
                if(nl == std::string::npos) break;
                publish(pending.substr(start, nl - start));
                start = nl + 1;
            }
            pending.erase(0, start);
        }
        if(!pending.empty()) publish(pending);
    }

    void startUnlocked() {
        int fds[2] = {-1, -1};
        if(::pipe(fds) != 0) throw std::runtime_error("record test log pipe failed");
        savedStdout = ::dup(STDOUT_FILENO);
        if(savedStdout < 0) {
            ::close(fds[0]);
            ::close(fds[1]);
            throw std::runtime_error("record test log dup failed");
        }
        if(::dup2(fds[1], STDOUT_FILENO) < 0) {
            ::close(fds[0]);
            ::close(fds[1]);
            ::close(savedStdout);
            savedStdout = -1;
            throw std::runtime_error("record test log redirect failed");
        }
        ::close(fds[1]);
        readFd = fds[0];
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        const int fd = readFd;
        const int echoFd = savedStdout;
        reader = std::thread([this, fd, echoFd] { loop(fd, echoFd); });
    }

    std::mutex mutex;
    std::vector<LineSink*> sinks;
    std::thread reader;
    int savedStdout = -1;
    int readFd = -1;
};

class LogCapture {
   public:
    LogCapture() {
        auto& logger = dai::Logging::getInstance().logger;
        savedLevel = logger.level();
        logger.set_level(spdlog::level::debug);
        sink = std::make_shared<LineSink>();
        StdoutTap::instance().attach(sink.get());
    }
    ~LogCapture() {
        StdoutTap::instance().detach(sink.get());
        dai::Logging::getInstance().logger.set_level(savedLevel);
    }
    LogCapture(const LogCapture&) = delete;
    LogCapture& operator=(const LogCapture&) = delete;

    int count(LineLevel level, const std::string& substring = "") const {
        std::lock_guard<std::mutex> lock(sink->linesMutex);
        int found = 0;
        for(const auto& line : sink->lines) {
            if(line.level != level) continue;
            if(!substring.empty() && line.text.find(substring) == std::string::npos) continue;
            ++found;
        }
        return found;
    }
    int countSubstring(const std::string& substring) const {
        std::lock_guard<std::mutex> lock(sink->linesMutex);
        int found = 0;
        for(const auto& line : sink->lines) {
            if(line.text.find(substring) != std::string::npos) ++found;
        }
        return found;
    }
    bool waitFor(const std::string& substring, std::chrono::milliseconds timeout) const {
        return waitUntil([&] { return countSubstring(substring) > 0; }, timeout);
    }
    std::vector<std::string> texts() const {
        std::lock_guard<std::mutex> lock(sink->linesMutex);
        std::vector<std::string> out;
        out.reserve(sink->lines.size());
        for(const auto& line : sink->lines) out.push_back(line.text);
        return out;
    }
    std::vector<std::string> textsAt(LineLevel level) const {
        std::lock_guard<std::mutex> lock(sink->linesMutex);
        std::vector<std::string> out;
        for(const auto& line : sink->lines) {
            if(line.level == level) out.push_back(line.text);
        }
        return out;
    }

   private:
    std::shared_ptr<LineSink> sink;
    spdlog::level::level_enum savedLevel = spdlog::level::info;
};

inline void watchNode(dai::ThreadedNode& node) {
    node.setLogLevel(dai::LogLevel::DEBUG);
}

class DeviceLogCapture {
   public:
    DeviceLogCapture(dai::Device& device, dai::LogLevel level) : device(&device), saved(device.getLogLevel()) {
        device.setLogLevel(level);
        id = device.addLogCallback([this](dai::LogMessage message) {
            std::lock_guard<std::mutex> lock(mutex);
            lines.push_back(LogLine{fromDai(message.level), message.payload});
        });
    }
    ~DeviceLogCapture() {
        if(device != nullptr && id >= 0) device->removeLogCallback(id);
        if(device != nullptr) device->setLogLevel(saved);
    }
    DeviceLogCapture(const DeviceLogCapture&) = delete;
    DeviceLogCapture& operator=(const DeviceLogCapture&) = delete;

    int count(LineLevel level, const std::string& substring = "") const {
        std::lock_guard<std::mutex> lock(mutex);
        int found = 0;
        for(const auto& line : lines) {
            if(line.level != level) continue;
            if(!substring.empty() && line.text.find(substring) == std::string::npos) continue;
            ++found;
        }
        return found;
    }
    int countSubstring(const std::string& substring) const {
        std::lock_guard<std::mutex> lock(mutex);
        int found = 0;
        for(const auto& line : lines) {
            if(line.text.find(substring) != std::string::npos) ++found;
        }
        return found;
    }
    bool waitFor(const std::string& substring, std::chrono::milliseconds timeout) const {
        return waitUntil([&] { return countSubstring(substring) > 0; }, timeout);
    }
    std::vector<std::string> texts() const {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::string> out;
        out.reserve(lines.size());
        for(const auto& line : lines) out.push_back(line.text);
        return out;
    }
    std::vector<std::string> textsAt(LineLevel level) const {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::string> out;
        for(const auto& line : lines) {
            if(line.level == level) out.push_back(line.text);
        }
        return out;
    }

   private:
    dai::Device* device = nullptr;
    dai::LogLevel saved = dai::LogLevel::WARN;
    int id = -1;
    mutable std::mutex mutex;
    std::vector<LogLine> lines;
};

class EnvGuard {
   public:
    explicit EnvGuard(std::string variable) : name(std::move(variable)), unset(true) {
        capture();
        ::unsetenv(name.c_str());
    }
    EnvGuard(std::string variable, std::string value) : name(std::move(variable)), value(std::move(value)) {
        capture();
        ::setenv(name.c_str(), this->value.c_str(), 1);
    }
    ~EnvGuard() {
        if(had) {
            ::setenv(name.c_str(), old.c_str(), 1);
        } else {
            ::unsetenv(name.c_str());
        }
    }
    EnvGuard(const EnvGuard&) = delete;
    EnvGuard& operator=(const EnvGuard&) = delete;

   private:
    void capture() {
        const char* current = ::getenv(name.c_str());
        had = current != nullptr;
        if(had) old = current;
    }
    std::string name;
    std::string value;
    std::string old;
    bool unset = false;
    bool had = false;
};

class RecordingGuard {
   public:
    RecordingGuard(dai::Device& device, dai::Pipeline& pipeline) : device(&device), pipeline(&pipeline) {
        for(const auto& info : listRecordings(device)) before.insert(info.name);
    }
    ~RecordingGuard() {
        try {
            if(pipeline != nullptr && pipeline->isRunning()) pipeline->stop();
            waitUntil(
                [&] {
                    bool pending = false;
                    for(const auto& info : listRecordings(*device)) {
                        if(before.count(info.name) == 0 && !info.complete) pending = true;
                    }
                    return !pending;
                },
                std::chrono::seconds(10));
            for(const auto& info : listRecordings(*device)) {
                if(before.count(info.name) != 0) continue;
                try {
                    deleteRecording(*device, info.name);
                } catch(const std::exception&) {
                }
            }
        } catch(const std::exception&) {
        }
    }
    RecordingGuard(const RecordingGuard&) = delete;
    RecordingGuard& operator=(const RecordingGuard&) = delete;
    // A second pipeline on the same device must not destroy this guard: that would delete the recording under test.
    void retarget(dai::Pipeline& next) {
        pipeline = &next;
    }
    void release() {
        pipeline = nullptr;
    }

   private:
    dai::Device* device = nullptr;
    dai::Pipeline* pipeline = nullptr;
    std::unordered_set<std::string> before;
};

struct SendRecord {
    std::vector<std::chrono::steady_clock::time_point> times;
    int sent = 0;
    std::string error;
};

inline SendRecord sendPaced(const std::shared_ptr<dai::InputQueue>& queue,
                            const std::vector<std::shared_ptr<dai::ADatatype>>& frames,
                            int periodMs,
                            dai::Pipeline* pipeline) {
    SendRecord record;
    std::thread worker([&] {
        try {
            for(const auto& frame : frames) {
                if(pipeline != nullptr && !pipeline->isRunning()) break;
                queue->send(frame);
                record.times.push_back(std::chrono::steady_clock::now());
                ++record.sent;
                if(periodMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
            }
        } catch(const std::exception& ex) {
            record.error = ex.what();
        }
    });
    worker.join();
    return record;
}

inline SendRecord sendPacedUntilStop(const std::shared_ptr<dai::InputQueue>& queue,
                                     const std::function<std::shared_ptr<dai::ADatatype>(int)>& makeFrame,
                                     int periodMs,
                                     dai::Pipeline* pipeline) {
    SendRecord record;
    std::thread worker([&] {
        try {
            for(int i = 0; pipeline == nullptr || pipeline->isRunning(); ++i) {
                queue->send(makeFrame(i));
                record.times.push_back(std::chrono::steady_clock::now());
                ++record.sent;
                if(periodMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
            }
        } catch(const std::exception& ex) {
            record.error = ex.what();
        }
    });
    worker.join();
    return record;
}

inline void sendFrame(const std::shared_ptr<dai::InputQueue>& queue, const std::shared_ptr<dai::ADatatype>& frame) {
    queue->send(frame);
}

inline std::shared_ptr<dai::RecordStatus> expectStatus(const std::shared_ptr<dai::MessageQueue>& queue,
                                                       dai::RecordStatus::State state,
                                                       std::int32_t segment,
                                                       std::int32_t frameCount,
                                                       std::int64_t first,
                                                       std::int64_t last,
                                                       std::int32_t skipped) {
    auto status = getFor<dai::RecordStatus>(*queue, std::chrono::seconds(10));
    REQUIRE(status != nullptr);
    CHECK(status->state == state);
    CHECK(status->segment == segment);
    CHECK(status->frameCount == frameCount);
    CHECK(status->firstSequenceNum == first);
    CHECK(status->lastSequenceNum == last);
    CHECK(status->skippedFrames == skipped);
    return status;
}

// pipeline.stop() closes the status queue. Read a queued STOPPED if it is still there, otherwise getStatus() within 1s.
inline std::shared_ptr<dai::RecordStatus> stopAndStatus(dai::Pipeline& pipeline, const std::shared_ptr<dai::MessageQueue>& queue, const dai::node::RecordVideo& rec) {
    pipeline.stop();
    std::shared_ptr<dai::RecordStatus> status;
    try {
        status = queue->tryGet<dai::RecordStatus>();
    } catch(const std::exception&) {
        status = nullptr;
    }
    if(status && status->state == dai::RecordStatus::State::STOPPED) return status;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while(std::chrono::steady_clock::now() < deadline) {
        status = getStatus(rec);
        if(status && status->state == dai::RecordStatus::State::STOPPED) return status;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return getStatus(rec);
}

inline std::shared_ptr<dai::ImgFrame> loadImg(const std::filesystem::path& path) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->load(path);
    return frame;
}

inline std::shared_ptr<dai::EncodedFrame> loadEnc(const std::filesystem::path& path) {
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->load(path);
    return frame;
}

struct McapMessage {
    std::string schemaName;
    std::int64_t sequenceNum = 0;
    std::int64_t tsMs = 0;
};

inline std::vector<McapMessage> readMcap(const std::filesystem::path& path) {
    dai::utility::BytePlayer player;
    const auto schema = player.init(path.string());
    std::vector<McapMessage> messages;
    while(auto proto = player.next<dai::proto::img_frame::ImgFrame>()) {
        McapMessage message;
        message.schemaName = schema;
        message.sequenceNum = proto->sequencenum();
        message.tsMs = proto->tsdevice().sec() * 1000 + proto->tsdevice().nsec() / 1000000;
        messages.push_back(message);
    }
    player.close();
    return messages;
}

inline std::string mcapCompression(const std::filesystem::path& path) {
    mcap::McapReader reader;
    const auto status = reader.open(path.string());
    if(!status.ok()) throw std::runtime_error(status.message);
    const auto summary = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if(!summary.ok()) throw std::runtime_error(summary.message);
    const auto& chunks = reader.chunkIndexes();
    if(chunks.empty()) return "";
    return chunks.front().compression;
}

inline int videoCaptureCount(const std::filesystem::path& video) {
    cv::VideoCapture capture(video.string());
    if(!capture.isOpened()) throw std::runtime_error("cannot open " + video.string());
    const int reported = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT));
    if(reported > 0) return reported;
    int count = 0;
    cv::Mat frame;
    while(capture.read(frame)) ++count;
    return count;
}

inline double videoDurationS(const std::filesystem::path& video) {
    cv::VideoCapture capture(video.string());
    if(!capture.isOpened()) throw std::runtime_error("cannot open " + video.string());
    const double frames = capture.get(cv::CAP_PROP_FRAME_COUNT);
    const double fps = capture.get(cv::CAP_PROP_FPS);
    if(fps <= 0.0) throw std::runtime_error("video fps is 0");
    return frames / fps;
}

inline int countVideoFrames(const std::filesystem::path& video, const std::filesystem::path& mcap) {
    const int decoded = videoCaptureCount(video);
    const int metadata = static_cast<int>(readMcap(mcap).size());
    REQUIRE(decoded == metadata);
    return decoded;
}

inline void requireRecordData() {
    if(!recordDataAvailable()) SKIP("needs the generated data");
}

struct HostRecord {
    TempDir dir;
    LogCapture log;
    dai::Pipeline pipeline{false};
    std::shared_ptr<dai::node::RecordVideo> node;
    std::shared_ptr<dai::InputQueue> frames;
    std::shared_ptr<dai::MessageQueue> status;
    std::shared_ptr<dai::InputQueue> controlQueue;

    explicit HostRecord(bool defaults = true) {
        node = pipeline.create<dai::node::RecordVideo>();
        watchNode(*node);
        if(defaults) {
            setFormat(*node, Format::RAW_FRAMES);
            setRecordingName(*node, "test");
            setOutputDirectory(*node, dir.path);
            setStreamName(*node, "cam");
        }
        frames = node->input.createInputQueue();
        status = node->status.createOutputQueue(16, true);
    }
    std::shared_ptr<dai::InputQueue> control() {
        if(!controlQueue) controlQueue = node->inputControl.createInputQueue();
        return controlQueue;
    }
    std::filesystem::path recDir() const {
        return dir.path / "test";
    }
    std::filesystem::path streamDir(const std::string& stream = "cam") const {
        return recDir() / stream;
    }
};

inline std::shared_ptr<dai::Device> openRvc4OrSkip() {
    auto device = openFirst(dai::Platform::RVC4);
    if(!device) SKIP("needs an RVC4");
    return device;
}

inline std::shared_ptr<dai::Device> openRvc2OrSkip() {
    auto device = openFirst(dai::Platform::RVC2);
    if(!device) SKIP("needs an RVC2");
    return device;
}

struct DeviceRecord {
    std::shared_ptr<dai::Device> device;
    EnvGuard unsetReplay;
    EnvGuard unsetRecord;
    TempDir dir;
    DeviceLogCapture devLog;
    LogCapture hostLog;
    dai::Pipeline pipeline;
    RecordingGuard guard;

    DeviceRecord()
        : device(openRvc4OrSkip()),
          unsetReplay("DEPTHAI_REPLAY"),
          unsetRecord("DEPTHAI_RECORD"),
          devLog(*device, dai::LogLevel::DEBUG),
          pipeline(device),
          guard(*device, pipeline) {}
};

inline void prepareRaw(dai::node::RecordVideo& rec, const std::filesystem::path& output, const std::string& name, bool transfer) {
    setFormat(rec, Format::RAW_FRAMES);
    setRecordingName(rec, name);
    setOutputDirectory(rec, output);
    setStreamName(rec, "cam");
    setTransferToHost(rec, transfer);
}

}  // namespace record_test
