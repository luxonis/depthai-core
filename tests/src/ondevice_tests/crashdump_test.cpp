#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

namespace {

using namespace std::chrono_literals;
namespace fs = std::filesystem;

constexpr auto NO_CALLBACK_TIMEOUT = 5s;
constexpr auto CRASH_DUMP_CALLBACK_TIMEOUT = 30s;
constexpr auto CRASH_DUMP_FILE_TIMEOUT = 15s;
constexpr auto PIPELINE_RUNNING_TIMEOUT = 15s;
constexpr auto RECONNECT_EVENT_TIMEOUT = 45s;
constexpr auto CRASH_DETECTION_TIMEOUT = 20s;

class ScopedEnvVar {
   public:
    ScopedEnvVar(std::string key, std::optional<std::string> value) : key(std::move(key)) {
        if(const char* existing = std::getenv(this->key.c_str())) {
            previous = std::string(existing);
        }

        if(value.has_value()) {
            dai::utility::setEnv(this->key, *value);
        } else {
            dai::utility::unsetEnv(this->key);
        }
    }

    ~ScopedEnvVar() {
        if(previous.has_value()) {
            dai::utility::setEnv(key, *previous);
        } else {
            dai::utility::unsetEnv(key);
        }
    }

   private:
    std::string key;
    std::optional<std::string> previous;
};

struct CrashObserver {
    std::mutex mtx;
    std::condition_variable cv;
    std::vector<std::shared_ptr<dai::CrashDump>> dumps;

    void callback(const std::shared_ptr<dai::CrashDump>& dump) {
        std::lock_guard<std::mutex> lock(mtx);
        dumps.push_back(dump);
        cv.notify_all();
    }

    bool waitForCount(size_t expectedCount, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mtx);
        return cv.wait_for(lock, timeout, [&] { return dumps.size() >= expectedCount; });
    }

    size_t count() {
        std::lock_guard<std::mutex> lock(mtx);
        return dumps.size();
    }

    std::shared_ptr<dai::CrashDump> get(size_t index) {
        std::lock_guard<std::mutex> lock(mtx);
        return dumps.at(index);
    }
};

bool waitUntil(const std::function<bool()>& predicate, std::chrono::milliseconds timeout, std::chrono::milliseconds poll = 200ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        if(predicate()) return true;
        std::this_thread::sleep_for(poll);
    }
    return predicate();
}

bool filenameMatchesCrashDumpPattern(const std::string& filename) {
    static const std::string prefix = "crash_dump_";
    static const std::string suffix = ".tar.gz";
    return filename.size() >= prefix.size() + suffix.size() && filename.compare(0, prefix.size(), prefix) == 0
           && filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isUsbProtocol(XLinkProtocol_t protocol) {
    return protocol == X_LINK_USB_VSC || protocol == X_LINK_USB_CDC || protocol == X_LINK_USB_EP;
}

// RVC4 crashdump coverage over USB is not working yet. Skip these cases for now
// and leave a note so the gap is removed once the USB path is fixed in the future.
void skipIfRvc4OverUsb(const dai::DeviceBase& device) {
    const auto deviceInfo = device.getDeviceInfo();
    if(device.getPlatform() == dai::Platform::RVC4 && isUsbProtocol(deviceInfo.protocol)) {
        SKIP("Skipping crashdump test on RVC4 over USB for now; this path is not working yet and should be fixed in the future.");
    }
}

void requireCrashDumpPayload(const std::shared_ptr<dai::CrashDump>& dump) {
    REQUIRE(dump != nullptr);
    REQUIRE_FALSE(dump->deviceId.empty());
    REQUIRE_FALSE(dump->crashdumpTimestamp.empty());
    REQUIRE_FALSE(dump->depthaiCommitHash.empty());
    if(auto rvc2Dump = std::dynamic_pointer_cast<dai::CrashDumpRVC2>(dump)) {
        REQUIRE_FALSE(rvc2Dump->crashReports.crashReports.empty());
    } else if(auto rvc4Dump = std::dynamic_pointer_cast<dai::CrashDumpRVC4>(dump)) {
        REQUIRE_FALSE(rvc4Dump->data.empty());
        REQUIRE_FALSE(rvc4Dump->filename.empty());
    } else {
        FAIL("Unknown crash dump type returned by callback");
    }
}

void waitForCrashDetection(dai::DeviceBase& device, std::chrono::milliseconds timeout = CRASH_DETECTION_TIMEOUT) {
    bool crashDetected = waitUntil([&] { return device.hasCrashed(); }, timeout);
    REQUIRE(crashDetected);
}

fs::path makeCrashDumpDir() {
    auto base = fs::temp_directory_path() / fs::path("depthai_crashdump_test_" + std::to_string(std::rand()));
    fs::create_directories(base);
    return base;
}

std::optional<std::string> reconnectTimeoutValue(bool disableReconnect) {
    if(disableReconnect) return "0";
    return std::nullopt;
}

void waitForNoCallback(CrashObserver& observer, std::chrono::milliseconds timeout = NO_CALLBACK_TIMEOUT) {
    REQUIRE_FALSE(observer.waitForCount(1, timeout));
}

void waitForPipelineRunning(dai::DeviceBase& device, std::chrono::milliseconds timeout = PIPELINE_RUNNING_TIMEOUT) {
    const bool pipelineRunning = waitUntil(
        [&] {
            try {
                return device.isPipelineRunning();
            } catch(const std::exception&) {
                return false;
            }
        },
        timeout);
    REQUIRE(pipelineRunning);
}

}  // namespace

// Verifies the registration contract: if automatic crashdump collection is disabled
// up front, the callback API should reject registration instead of failing later.
TEST_CASE("Crashdump callback rejects disabled automatic collection") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");
    ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", std::nullopt);

    SECTION("DEPTHAI_CRASHDUMP=0") {
        ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", "0");
        ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", std::nullopt);

        dai::Device device;
        skipIfRvc4OverUsb(device);
        REQUIRE_THROWS(device.registerCrashdumpCallback([](std::shared_ptr<dai::CrashDump>) {}));
    }

    SECTION("DEPTHAI_CRASHDUMP_TIMEOUT=0") {
        ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
        ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", "0");

        dai::Device device;
        skipIfRvc4OverUsb(device);
        REQUIRE_THROWS(device.registerCrashdumpCallback([](std::shared_ptr<dai::CrashDump>) {}));
    }
}

// Verifies that automatic collection is controlled dynamically by the env vars:
// a callback registered while enabled must stop firing once collection is disabled.
TEST_CASE("Crashdump callback is not invoked after automatic collection is disabled") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");

    SECTION("DEPTHAI_RECONNECT_TIMEOUT=0") {
        ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", reconnectTimeoutValue(true));
        ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
        ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", std::nullopt);

        CrashObserver observer;
        std::optional<ScopedEnvVar> disableCrashdump;
        {
            dai::Device device;
            skipIfRvc4OverUsb(device);
            device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

            disableCrashdump.emplace("DEPTHAI_CRASHDUMP", "0");
            device.crashDevice();
            waitForNoCallback(observer);
        }
    }

    SECTION("DEPTHAI_RECONNECT_TIMEOUT unset") {
        ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", reconnectTimeoutValue(false));
        ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
        ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", std::nullopt);

        CrashObserver observer;
        std::optional<ScopedEnvVar> disableCrashdumpTimeout;
        {
            dai::Device device;
            skipIfRvc4OverUsb(device);
            device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

            disableCrashdumpTimeout.emplace("DEPTHAI_CRASHDUMP_TIMEOUT", "0");
            device.crashDevice();
            waitForNoCallback(observer);
        }
    }
}

// Verifies the basic happy path for automatic collection: a device crash produces
// a callback with a real payload, both with reconnect disabled and with default reconnect behavior.
TEST_CASE("Crashdump callback is invoked on device crash regardless of reconnect timeout setting") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");
    ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
    ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", std::nullopt);

    SECTION("DEPTHAI_RECONNECT_TIMEOUT=0") {
        ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", reconnectTimeoutValue(true));

        CrashObserver observer;
        dai::Device device;
        skipIfRvc4OverUsb(device);
        device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

        device.crashDevice();
        REQUIRE(observer.waitForCount(1, CRASH_DUMP_CALLBACK_TIMEOUT));
        requireCrashDumpPayload(observer.get(0));
    }

    SECTION("DEPTHAI_RECONNECT_TIMEOUT unset") {
        ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", reconnectTimeoutValue(false));

        CrashObserver observer;
        dai::Device device;
        skipIfRvc4OverUsb(device);
        device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

        device.crashDevice();
        REQUIRE(observer.waitForCount(1, CRASH_DUMP_CALLBACK_TIMEOUT));
        requireCrashDumpPayload(observer.get(0));
    }
}

// Verifies the file-output path of automatic collection: when DEPTHAI_CRASHDUMP points
// to a directory, the collected archive is saved there and can be loaded back successfully.
TEST_CASE("Crashdump is written to the configured path") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");
    ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", "0");
    ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", std::nullopt);

    auto crashDumpDir = makeCrashDumpDir();
    ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", crashDumpDir.string());

    CrashObserver observer;
    {
        dai::Device device;
        skipIfRvc4OverUsb(device);
        device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

        device.crashDevice();
        REQUIRE(observer.waitForCount(1, CRASH_DUMP_CALLBACK_TIMEOUT));
        requireCrashDumpPayload(observer.get(0));

        fs::path expectedPath;
        REQUIRE(waitUntil(
            [&] {
                for(const auto& entry : fs::directory_iterator(crashDumpDir)) {
                    if(!entry.is_regular_file()) continue;
                    const auto filename = entry.path().filename().string();
                    if(filenameMatchesCrashDumpPattern(filename)) {
                        expectedPath = entry.path();
                        return true;
                    }
                }
                return false;
            },
            CRASH_DUMP_FILE_TIMEOUT));

        auto loadedDump = dai::CrashDump::load(expectedPath);
        requireCrashDumpPayload(std::shared_ptr<dai::CrashDump>(std::move(loadedDump)));
    }

    fs::remove_all(crashDumpDir);
}

// Verifies that crashdump collection is not a one-shot state: after a crash, reconnect,
// and pipeline recovery, a later crash should still produce another crashdump callback.
TEST_CASE("Crashdump can be collected more than once when reconnection is allowed") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");
    ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", "45000");
    ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
    ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", "30000");

    CrashObserver observer;
    std::mutex reconnectMtx;
    std::condition_variable reconnectCv;
    int reconnectCount = 0;

    auto device = std::make_shared<dai::Device>();
    skipIfRvc4OverUsb(*device);
    device->setMaxReconnectionAttempts(5, [&](dai::DeviceBase::ReconnectionStatus status) {
        if(status == dai::DeviceBase::ReconnectionStatus::RECONNECTED) {
            std::lock_guard<std::mutex> lock(reconnectMtx);
            reconnectCount++;
            reconnectCv.notify_all();
        }
    });
    device->registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) { observer.callback(dump); });

    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto* cameraOutput = camera->requestOutput(std::make_pair(640, 400));
    REQUIRE(cameraOutput != nullptr);
    auto outputQueue = cameraOutput->createOutputQueue();

    pipeline.start();

    bool hasTimedOut = false;
    auto img = outputQueue->get<dai::ImgFrame>(std::chrono::duration<double>(15), hasTimedOut);
    REQUIRE(!hasTimedOut);
    REQUIRE(img != nullptr);

    device->crashDevice();
    REQUIRE(observer.waitForCount(1, CRASH_DUMP_CALLBACK_TIMEOUT));
    requireCrashDumpPayload(observer.get(0));

    {
        std::unique_lock<std::mutex> lock(reconnectMtx);
        REQUIRE(reconnectCv.wait_for(lock, RECONNECT_EVENT_TIMEOUT, [&] { return reconnectCount >= 1; }));
    }

    waitForPipelineRunning(*device, PIPELINE_RUNNING_TIMEOUT);
    std::optional<ScopedEnvVar> disableReconnectAfterSecondProof;
    if(device->getPlatform() == dai::Platform::RVC4) {
        disableReconnectAfterSecondProof.emplace("DEPTHAI_RECONNECT_TIMEOUT", "0");
    }
    device->crashDevice();
    REQUIRE(observer.waitForCount(2, CRASH_DUMP_CALLBACK_TIMEOUT));
    requireCrashDumpPayload(observer.get(1));

    if(device->getPlatform() != dai::Platform::RVC4) {
        std::unique_lock<std::mutex> lock(reconnectMtx);
        REQUIRE(reconnectCv.wait_for(lock, RECONNECT_EVENT_TIMEOUT, [&] { return reconnectCount >= 2; }));
        lock.unlock();
        waitForPipelineRunning(*device, PIPELINE_RUNNING_TIMEOUT);
    }

    outputQueue->close();
    pipeline.stop();
    device->close();
}

// Verifies the lightweight crash-state surface separately from callback delivery:
// after a crash, the host-side hasCrashed() flag should eventually become true.
TEST_CASE("hasCrashed returns true after device crash") {
    ScopedEnvVar disableUpload("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    ScopedEnvVar crashDevice("DEPTHAI_CRASH_DEVICE", "1");
    ScopedEnvVar reconnectTimeout("DEPTHAI_RECONNECT_TIMEOUT", "0");
    ScopedEnvVar crashdumpPath("DEPTHAI_CRASHDUMP", std::nullopt);
    ScopedEnvVar crashdumpTimeout("DEPTHAI_CRASHDUMP_TIMEOUT", "30000");

    dai::Device device;
    skipIfRvc4OverUsb(device);
    REQUIRE_FALSE(device.hasCrashed());

    device.crashDevice();
    waitForCrashDetection(device, CRASH_DETECTION_TIMEOUT);
    REQUIRE(device.hasCrashed());
    device.close();
}
