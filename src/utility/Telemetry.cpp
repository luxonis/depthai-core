#include "utility/Telemetry.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "build/version.hpp"
#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "utility/Logging.hpp"
#include "utility/Platform.hpp"
#include "utility/Uuid.hpp"

#ifdef DEPTHAI_ENABLE_CURL
    #include <cpr/cpr.h>
    #include <curl/curl.h>
#endif

namespace dai {
namespace utility {

namespace {

using Clock = std::chrono::system_clock;

constexpr std::size_t DEFAULT_FLUSH_AT = 20;
constexpr std::size_t DEFAULT_MAX_QUEUE_SIZE = 1000;
constexpr std::size_t DEFAULT_MAX_BATCH_SIZE = 50;
constexpr std::chrono::seconds DEFAULT_FLUSH_INTERVAL{30};
constexpr std::chrono::seconds RETRY_DELAY{5};
constexpr std::chrono::seconds MAX_RETRY_DELAY{30};
constexpr char DEFAULT_POSTHOG_HOST[] = "https://b.luxonis.com";
constexpr char DEFAULT_POSTHOG_API_KEY[] = "phc_ojEByaCiZZ5eigzaM43PaEVbfLfFDF5NgkXEMPabrT9a";
constexpr char DEFAULT_TELEMETRY_ROOT_DIR[] = "telemetry";
constexpr char TMP_IDS_FILENAME[] = "tmpIds.json";
constexpr auto TMP_ID_TTL = std::chrono::hours(24);

std::atomic_bool telemetryUsesPython{false};

std::string readEnv(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string{};
}

std::string trim(std::string value) {
    auto isSpace = [](unsigned char ch) { return std::isspace(ch) != 0; };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [&](unsigned char ch) { return !isSpace(ch); }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [&](unsigned char ch) { return !isSpace(ch); }).base(), value.end());
    return value;
}

std::string defaultCaptureUrl() {
    return std::string(DEFAULT_POSTHOG_HOST) + "/capture/";
}

std::string normalizeCaptureUrl(std::string url) {
    url = trim(std::move(url));
    if(url.empty()) {
        return defaultCaptureUrl();
    }

    const auto schemePos = url.find("://");
    const auto pathPos = schemePos == std::string::npos ? url.find('/') : url.find('/', schemePos + 3);
    if(pathPos == std::string::npos) {
        return url + "/capture/";
    }

    const auto path = url.substr(pathPos);
    if(path.empty() || path == "/") {
        if(url.back() == '/') {
            return url + "capture/";
        }
        return url + "/capture/";
    }

    return url;
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::filesystem::path defaultTelemetryBaseDir() {
    return std::filesystem::current_path() / ".cache" / "depthai" / DEFAULT_TELEMETRY_ROOT_DIR;
}

std::string generateUuidV4() {
    static std::mt19937_64 generator(std::random_device{}());
    static std::uniform_int_distribution<int> distribution(0, 255);

    std::array<std::uint8_t, 16> bytes{};
    for(auto& byte : bytes) {
        byte = static_cast<std::uint8_t>(distribution(generator));
    }

    bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0F) | 0x40);
    bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3F) | 0x80);

    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for(std::size_t index = 0; index < bytes.size(); ++index) {
        stream << std::setw(2) << static_cast<int>(bytes[index]);
        if(index == 3 || index == 5 || index == 7 || index == 9) {
            stream << '-';
        }
    }
    return stream.str();
}

std::string toIso8601(const Clock::time_point& timePoint) {
    const auto time = Clock::to_time_t(timePoint);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()) % std::chrono::seconds(1);

    std::tm utcTime{};
#ifdef _WIN32
    gmtime_s(&utcTime, &time);
#else
    gmtime_r(&time, &utcTime);
#endif

    std::ostringstream stream;
    stream << std::put_time(&utcTime, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << milliseconds.count() << 'Z';
    return stream.str();
}

std::string makeQueueFilename() {
    const auto timestampMs = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
    std::ostringstream stream;
    stream << std::setw(20) << std::setfill('0') << timestampMs << '_' << generateUuidV4() << ".json";
    return stream.str();
}

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}

struct TemporaryIdEntry {
    std::string id;
    int64_t expiresAtMs = 0;
};

struct TemporaryDeviceIdEntry {
    std::string mxid;
    std::string tmpDeviceId;
    int64_t expiresAtMs = 0;
};

struct TemporaryIdsDocument {
    TemporaryIdEntry host;
    std::vector<TemporaryDeviceIdEntry> devices;
};

bool isExpired(const TemporaryIdEntry& entry, int64_t currentMs) {
    return entry.id.empty() || entry.expiresAtMs <= currentMs;
}

bool isExpired(const TemporaryDeviceIdEntry& entry, int64_t currentMs) {
    return entry.tmpDeviceId.empty() || entry.expiresAtMs <= currentMs;
}

std::string getTelemetryHostOSImpl() {
    if(!readEnv("OAKAGENT_APP_ID").empty() || !readEnv("OAKAGENT_APP_IDENTIFIER").empty()) {
        return "oakapp";
    }

    const auto platformName = dai::platform::getOSPlatform();
    if(platformName == "Windows") return "windows";
    if(platformName == "Linux") return "linux";
    if(platformName == "MacOS") return "mac";
    return lowercase(platformName);
}

std::string getTelemetryHostOS() {
    return getTelemetryHostOSImpl();
}

std::string getTelemetryHostOSVersion() {
    return dai::platform::getOSVersion();
}

std::filesystem::path tmpIdsPath() {
    return defaultTelemetryBaseDir() / TMP_IDS_FILENAME;
}

TemporaryIdsDocument readTemporaryIdsDocument(const std::filesystem::path& path) {
    TemporaryIdsDocument document;

    std::ifstream input(path);
    if(!input.good()) {
        return document;
    }

    nlohmann::json json;
    try {
        input >> json;
    } catch(const std::exception&) {
        return document;
    }

    if(json.contains("tmp_host_id") && json["tmp_host_id"].is_object()) {
        const auto& host = json["tmp_host_id"];
        document.host.id = host.value("value", std::string{});
        document.host.expiresAtMs = host.value("expires_at_ms", int64_t{0});
    }

    if(json.contains("tmp_device_ids") && json["tmp_device_ids"].is_array()) {
        for(const auto& item : json["tmp_device_ids"]) {
            if(!item.is_object()) continue;
            TemporaryDeviceIdEntry entry;
            entry.mxid = item.value("mxid", std::string{});
            entry.tmpDeviceId = item.value("tmp_device_id", std::string{});
            entry.expiresAtMs = item.value("expires_at_ms", int64_t{0});
            if(!entry.mxid.empty()) {
                document.devices.push_back(std::move(entry));
            }
        }
    }

    return document;
}

nlohmann::json writeTemporaryIdsDocument(const TemporaryIdsDocument& document) {
    nlohmann::json json = nlohmann::json::object();
    json["tmp_host_id"] = {
        {"value", document.host.id},
        {"expires_at_ms", document.host.expiresAtMs},
    };

    json["tmp_device_ids"] = nlohmann::json::array();
    for(const auto& entry : document.devices) {
        json["tmp_device_ids"].push_back({
            {"mxid", entry.mxid},
            {"tmp_device_id", entry.tmpDeviceId},
            {"expires_at_ms", entry.expiresAtMs},
        });
    }

    return json;
}

void persistTemporaryIdsDocument(const std::filesystem::path& path, const TemporaryIdsDocument& document) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if(!output.good()) {
        throw std::runtime_error("Failed to open tmpIds file for writing");
    }

    output << writeTemporaryIdsDocument(document).dump(2);
    if(!output.good()) {
        throw std::runtime_error("Failed to write tmpIds file");
    }
}

class TemporaryIdsManager {
   public:
    std::string getHostId() {
        std::lock_guard<std::mutex> guard(mutex);
        const auto currentMs = nowMs();
        auto fileLock = acquireLock();
        (void)fileLock;
        bool changed = false;
        auto document = loadLocked(currentMs, changed);
        if(isExpired(document.host, currentMs)) {
            document.host.id = utility::generateUuidV7();
            document.host.expiresAtMs = currentMs + std::chrono::duration_cast<std::chrono::milliseconds>(TMP_ID_TTL).count();
            changed = true;
        }
        if(changed) saveLocked(document);
        return document.host.id;
    }

    std::string getDeviceId(const std::string& mxid) {
        std::lock_guard<std::mutex> guard(mutex);
        const auto currentMs = nowMs();
        auto fileLock = acquireLock();
        (void)fileLock;
        bool changed = false;
        auto document = loadLocked(currentMs, changed);

        auto it = std::find_if(document.devices.begin(), document.devices.end(), [&](const TemporaryDeviceIdEntry& entry) { return entry.mxid == mxid; });
        if(it == document.devices.end()) {
            TemporaryDeviceIdEntry entry;
            entry.mxid = mxid;
            entry.tmpDeviceId = utility::generateUuidV7();
            entry.expiresAtMs = currentMs + std::chrono::duration_cast<std::chrono::milliseconds>(TMP_ID_TTL).count();
            const auto tmpDeviceId = entry.tmpDeviceId;
            document.devices.push_back(entry);
            changed = true;
            if(changed) saveLocked(document);
            return tmpDeviceId;
        }

        if(isExpired(*it, currentMs)) {
            it->tmpDeviceId = utility::generateUuidV7();
            it->expiresAtMs = currentMs + std::chrono::duration_cast<std::chrono::milliseconds>(TMP_ID_TTL).count();
            changed = true;
        }

        if(changed) saveLocked(document);
        return it->tmpDeviceId;
    }

   private:
    std::unique_ptr<dai::platform::FileLock> acquireLock() {
        std::filesystem::create_directories(defaultTelemetryBaseDir());
        return dai::platform::FileLock::lock(tmpIdsPath(), true);
    }

    TemporaryIdsDocument loadLocked(int64_t currentMs, bool& changed) {
        auto document = readTemporaryIdsDocument(tmpIdsPath());
        const auto originalSize = document.devices.size();
        document.devices.erase(std::remove_if(document.devices.begin(),
                                              document.devices.end(),
                                              [&](const TemporaryDeviceIdEntry& entry) { return entry.mxid.empty() || isExpired(entry, currentMs); }),
                               document.devices.end());
        changed = changed || document.devices.size() != originalSize;
        return document;
    }

    void saveLocked(const TemporaryIdsDocument& document) {
        persistTemporaryIdsDocument(tmpIdsPath(), document);
    }

    std::mutex mutex;
};

TemporaryIdsManager& temporaryIdsManager() {
    static TemporaryIdsManager manager;
    return manager;
}

std::string safeTemporaryTelemetryHostId() {
    try {
        return temporaryIdsManager().getHostId();
    } catch(const std::exception& ex) {
        logger::debug("Failed to resolve temporary telemetry host id: {}", ex.what());
        return "unknown";
    }
}

}  // namespace

struct TelemetrySharedState {
    bool enabled{false};
    std::string apiKey;
    std::string captureUrl;
    std::string sessionKey{utility::generateUuidV7()};

    std::filesystem::path queueDir;

    std::size_t flushAt{DEFAULT_FLUSH_AT};
    std::size_t maxQueueSize{DEFAULT_MAX_QUEUE_SIZE};
    std::size_t maxBatchSize{DEFAULT_MAX_BATCH_SIZE};
    std::chrono::seconds flushInterval{DEFAULT_FLUSH_INTERVAL};

    std::deque<std::string> queuedFiles;
    std::vector<std::shared_ptr<Telemetry::AggregateMetricsCallback>> aggregateMetricsCallbacks;

    std::mutex mutex;
    std::mutex aggregateMetricsMtx;
    std::condition_variable condition;
    std::thread worker;
    bool stopRequested{false};
    bool flushRequested{false};
    std::size_t retryCount{0};
    std::optional<Clock::time_point> pausedUntil;

    TelemetrySharedState();
    ~TelemetrySharedState();

    void event(std::string eventName, nlohmann::json properties);
    void addAggregateMetrics(Telemetry::AggregateMetricsCallback functionLikeCb);
    void loadQueueFromDisk();
    void workerLoop();
    void flushOneBatch();
    void cleanupRunDirectory();
    void removeFileFromQueueLocked(const std::string& filename);
    void deleteFilesLocked(const std::vector<std::string>& filenames);
    void deleteFileOnDisk(const std::string& filename);
    void applyAggregateMetrics(nlohmann::json& properties);
};

TelemetrySharedState& telemetrySharedState() {
    static TelemetrySharedState state;
    return state;
}

class Telemetry::Impl {
   public:
    void event(std::string eventName, nlohmann::json properties) {
        telemetrySharedState().event(std::move(eventName), std::move(properties));
    }

    void addAggregateMetrics(Telemetry::AggregateMetricsCallback functionLikeCb) {
        telemetrySharedState().addAggregateMetrics(std::move(functionLikeCb));
    }
};

TelemetrySharedState::TelemetrySharedState() {
    captureUrl = normalizeCaptureUrl(readEnv("DEPTHAI_TELEMETRY_URL"));
    apiKey = trim(readEnv("DEPTHAI_TELEMETRY_API_KEY"));
    if(apiKey.empty()) {
        apiKey = DEFAULT_POSTHOG_API_KEY;
    }
    queueDir = defaultTelemetryBaseDir() / sessionKey;

    try {
        std::filesystem::create_directories(queueDir);
        loadQueueFromDisk();
    } catch(const std::exception& ex) {
        logger::warn("Failed to initialize telemetry storage at '{}': {}", queueDir.string(), ex.what());
        return;
    }

#ifdef DEPTHAI_ENABLE_CURL
    enabled = Telemetry::isTelemetryEnabled();
    if(!enabled) {
        logger::debug("Telemetry disabled via DEPTHAI_TELEMETRY");
        return;
    }

    const auto curlInitResult = curl_global_init(CURL_GLOBAL_DEFAULT);
    if(curlInitResult != CURLE_OK) {
        logger::debug("Telemetry curl global init failed: {}", curl_easy_strerror(curlInitResult));
    }

    worker = std::thread([this]() { workerLoop(); });
#else
    logger::debug("Telemetry disabled: depthai-core was built without CURL support");
#endif
}

TelemetrySharedState::~TelemetrySharedState() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopRequested = true;
        flushRequested = true;
    }
    condition.notify_all();

    if(worker.joinable()) {
        worker.join();
    }

    cleanupRunDirectory();
}

void TelemetrySharedState::event(std::string eventName, nlohmann::json properties) {
    if(!enabled) {
        return;
    }

    eventName = trim(std::move(eventName));
    if(eventName.empty()) {
        logger::debug("Skipping telemetry event with an empty name");
        return;
    }

    if(!properties.is_object()) {
        logger::warn("Telemetry properties for '{}' must be a JSON object. Dropping invalid properties.", eventName);
        properties = nlohmann::json::object();
    }
    if(eventName == "depthai_ping") {
        applyAggregateMetrics(properties);
        if(!properties.is_object()) {
            logger::warn("Telemetry aggregate metrics for '{}' produced invalid properties. Dropping invalid properties.", eventName);
            properties = nlohmann::json::object();
        }
    }

    if(!properties.contains("$lib")) {
        properties["$lib"] = "depthai-core";
    }
    if(!properties.contains("$lib_version")) {
        properties["$lib_version"] = build::VERSION;
    }
    if(!properties.contains("$session_id")) {
        properties["$session_id"] = sessionKey;
    }
    if(!properties.contains("$process_person_profile")) {
        properties["$process_person_profile"] = false;
    }
    properties["source_product"] = "depthai";
    properties["source_component"] = "depthai-core";

    const auto hostId = safeTemporaryTelemetryHostId();

    const auto payload = nlohmann::json{
        {"event", eventName},
        {"distinct_id", hostId},
        {"properties", std::move(properties)},
        {"timestamp", toIso8601(Clock::now())},
        {"uuid", generateUuidV4()},
    };

    std::string filename;
    {
        std::lock_guard<std::mutex> lock(mutex);

        if(queuedFiles.size() >= maxQueueSize && !queuedFiles.empty()) {
            const auto oldest = queuedFiles.front();
            queuedFiles.pop_front();
            deleteFileOnDisk(oldest);
            logger::debug("Telemetry queue is full, dropping oldest event '{}'", oldest);
        }

        filename = makeQueueFilename();
        const auto eventPath = queueDir / filename;
        std::ofstream output(eventPath, std::ios::binary | std::ios::trunc);
        if(!output) {
            logger::warn("Failed to open telemetry queue file '{}'", eventPath.string());
            return;
        }

        output << payload.dump();
        if(!output.good()) {
            output.close();
            std::error_code ec;
            std::filesystem::remove(eventPath, ec);
            logger::warn("Failed to write telemetry queue file '{}'", eventPath.string());
            return;
        }

        queuedFiles.push_back(filename);
        if(queuedFiles.size() >= flushAt) {
            flushRequested = true;
        }
    }

    condition.notify_one();
}

void TelemetrySharedState::addAggregateMetrics(Telemetry::AggregateMetricsCallback functionLikeCb) {
    if(!functionLikeCb) {
        return;
    }

    std::lock_guard<std::mutex> lock(aggregateMetricsMtx);
    aggregateMetricsCallbacks.push_back(std::make_shared<Telemetry::AggregateMetricsCallback>(std::move(functionLikeCb)));
}

void TelemetrySharedState::applyAggregateMetrics(nlohmann::json& properties) {
    std::vector<std::shared_ptr<Telemetry::AggregateMetricsCallback>> callbacks;
    {
        std::lock_guard<std::mutex> lock(aggregateMetricsMtx);
        callbacks = aggregateMetricsCallbacks;
    }

    for(const auto& callback : callbacks) {
        try {
            if(callback && *callback) {
                (*callback)(properties);
            }
        } catch(const std::exception& ex) {
            logger::debug("Telemetry aggregate metrics callback failed: {}", ex.what());
        } catch(...) {
            logger::debug("Telemetry aggregate metrics callback failed with an unknown exception");
        }
    }
}

void TelemetrySharedState::loadQueueFromDisk() {
    std::vector<std::string> filenames;
    for(const auto& entry : std::filesystem::directory_iterator(queueDir)) {
        if(entry.is_regular_file()) {
            filenames.push_back(entry.path().filename().string());
        }
    }
    std::sort(filenames.begin(), filenames.end());

    if(filenames.size() > maxQueueSize) {
        const auto toDrop = filenames.size() - maxQueueSize;
        for(std::size_t index = 0; index < toDrop; ++index) {
            deleteFileOnDisk(filenames[index]);
        }
        filenames.erase(filenames.begin(), filenames.begin() + static_cast<std::ptrdiff_t>(toDrop));
    }

    queuedFiles.assign(filenames.begin(), filenames.end());
}

void TelemetrySharedState::workerLoop() {
    auto nextFlushAt = Clock::now() + flushInterval;

    std::unique_lock<std::mutex> lock(mutex);
    while(!stopRequested) {
        auto wakeAt = nextFlushAt;

        condition.wait_until(lock, wakeAt, [this]() { return stopRequested || flushRequested; });
        if(stopRequested) {
            break;
        }

        const auto now = Clock::now();
        if(pausedUntil && now < *pausedUntil) {
            flushRequested = false;
            if(now >= nextFlushAt) {
                nextFlushAt = now + flushInterval;
            }
            continue;
        }

        if(!flushRequested && now < nextFlushAt) {
            continue;
        }

        flushRequested = false;
        nextFlushAt = now + flushInterval;

        lock.unlock();
        flushOneBatch();
        lock.lock();
    }
    lock.unlock();

    // Best effort flush on shutdown. This intentionally mirrors the one-shot
    // behavior of the iOS queue rather than draining the entire backlog.
    flushOneBatch();
}

void TelemetrySharedState::flushOneBatch() {
    std::vector<std::string> batchFiles;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if(queuedFiles.empty()) {
            return;
        }

        const auto batchSize = std::min<std::size_t>(maxBatchSize, queuedFiles.size());
        batchFiles.assign(queuedFiles.begin(), queuedFiles.begin() + static_cast<std::ptrdiff_t>(batchSize));
    }

    std::vector<std::string> invalidFiles;
    nlohmann::json events = nlohmann::json::array();

    for(const auto& filename : batchFiles) {
        const auto filePath = queueDir / filename;

        std::ifstream input(filePath, std::ios::binary);
        if(!input) {
            logger::warn("Telemetry queue file '{}' is missing", filePath.string());
            invalidFiles.push_back(filename);
            continue;
        }

        std::stringstream buffer;
        buffer << input.rdbuf();

        try {
            const auto eventPayload = nlohmann::json::parse(buffer.str());
            if(!eventPayload.is_object()) {
                throw std::runtime_error("payload is not a JSON object");
            }
            events.push_back(eventPayload);
        } catch(const std::exception& ex) {
            logger::warn("Failed to parse telemetry queue file '{}': {}", filePath.string(), ex.what());
            invalidFiles.push_back(filename);
        }
    }

    if(!invalidFiles.empty()) {
        std::lock_guard<std::mutex> lock(mutex);
        deleteFilesLocked(invalidFiles);
        batchFiles.erase(
            std::remove_if(batchFiles.begin(),
                           batchFiles.end(),
                           [&](const std::string& filename) { return std::find(invalidFiles.begin(), invalidFiles.end(), filename) != invalidFiles.end(); }),
            batchFiles.end());
    }

    if(events.empty()) {
        return;
    }

    std::vector<std::string> deletableFiles;
    bool shouldRetry = false;

#ifdef DEPTHAI_ENABLE_CURL
    try {
        for(std::size_t index = 0; index < events.size(); ++index) {
            const auto& queuedEvent = events[index];
            auto properties = queuedEvent.value("properties", nlohmann::json::object());
            if(!properties.is_object()) {
                properties = nlohmann::json::object();
            }
            std::string distinctId = "unknown";
            if(auto it = queuedEvent.find("distinct_id"); it != queuedEvent.end() && it->is_string()) {
                distinctId = it->get<std::string>();
            } else {
                distinctId = safeTemporaryTelemetryHostId();
            }

            nlohmann::json requestBody = {
                {"api_key", apiKey},
                {"event", queuedEvent.value("event", "")},
                {"distinct_id", std::move(distinctId)},
                {"properties", std::move(properties)},
            };

            if(queuedEvent.contains("timestamp")) {
                requestBody["timestamp"] = queuedEvent["timestamp"];
            }

            auto response = cpr::Post(cpr::Url{captureUrl},
                                      cpr::Body{requestBody.dump()},
                                      cpr::Header{{"Content-Type", "application/json"}, {"User-Agent", std::string("depthai-core/") + build::VERSION}},
                                      cpr::Timeout{10000},
                                      cpr::VerifySsl(true));

            const auto statusCode = response.status_code;
            const bool retryable = static_cast<bool>(response.error) || statusCode <= 0 || (statusCode >= 300 && statusCode <= 399) || statusCode == 429
                                   || (statusCode >= 500 && statusCode <= 599);

            if(response.error) {
                logger::debug("Telemetry capture upload error {}: {}", static_cast<int>(response.error.code), response.error.message);
            } else if(statusCode < 200 || statusCode > 299) {
                logger::debug("Telemetry capture upload failed with status {}: {}", statusCode, response.text);
            }

            if(retryable) {
                shouldRetry = true;
                break;
            }

            deletableFiles.push_back(batchFiles[index]);
        }
    } catch(const std::exception& ex) {
        logger::debug("Telemetry capture upload threw an exception: {}", ex.what());
        shouldRetry = true;
    }
#endif

    std::lock_guard<std::mutex> lock(mutex);
    deleteFilesLocked(deletableFiles);

    if(shouldRetry) {
        retryCount += 1;
        const auto delaySeconds =
            std::min<std::size_t>(retryCount * static_cast<std::size_t>(RETRY_DELAY.count()), static_cast<std::size_t>(MAX_RETRY_DELAY.count()));
        pausedUntil = Clock::now() + std::chrono::seconds(delaySeconds);
        logger::debug("Pausing telemetry uploads for {} seconds after retryable failure", delaySeconds);
        return;
    }

    retryCount = 0;
    pausedUntil.reset();
    deleteFilesLocked(batchFiles);
}

void TelemetrySharedState::removeFileFromQueueLocked(const std::string& filename) {
    const auto iterator = std::find(queuedFiles.begin(), queuedFiles.end(), filename);
    if(iterator != queuedFiles.end()) {
        queuedFiles.erase(iterator);
    }
}

void TelemetrySharedState::deleteFilesLocked(const std::vector<std::string>& filenames) {
    for(const auto& filename : filenames) {
        removeFileFromQueueLocked(filename);
        deleteFileOnDisk(filename);
    }
}

void TelemetrySharedState::deleteFileOnDisk(const std::string& filename) {
    std::error_code ec;
    std::filesystem::remove(queueDir / filename, ec);
    if(ec) {
        logger::debug("Failed to delete telemetry queue file '{}': {}", filename, ec.message());
    }
}

void TelemetrySharedState::cleanupRunDirectory() {
    std::error_code ec;
    std::filesystem::remove_all(queueDir, ec);
    if(ec) {
        logger::debug("Failed to delete telemetry run directory '{}': {}", queueDir.string(), ec.message());
    }
}

Telemetry& Telemetry::getInstance() {
    static Telemetry telemetry;
    return telemetry;
}

Telemetry::Telemetry() : impl(std::make_unique<Impl>()) {}

Telemetry::~Telemetry() = default;

namespace {

void normalizeTelemetryProperties(nlohmann::json& properties) {
    if(!properties.is_object()) {
        logger::warn("Telemetry properties must be a JSON object. Dropping invalid properties.");
        properties = nlohmann::json::object();
    }
    if(!properties.contains("$process_person_profile")) {
        properties["$process_person_profile"] = false;
    }
}

void addDeviceTelemetryProperties(const DeviceBase& device, nlohmann::json& properties) {
    normalizeTelemetryProperties(properties);
    if(!properties.contains("device_id")) {
        properties["device_id"] = device.getTemporaryTelemetryDeviceId();
    }
    if(!properties.contains("pipeline_id")) {
        if(auto pipelineId = device.getActiveTelemetryPipelineId()) {
            properties["pipeline_id"] = *pipelineId;
        }
    }
}

void addPipelineTelemetryProperties(const Pipeline& pipeline, nlohmann::json& properties) {
    normalizeTelemetryProperties(properties);
    if(!properties.contains("pipeline_id")) {
        properties["pipeline_id"] = pipeline.getTelemetryPipelineId();
    }
    if(auto device = pipeline.getDefaultDevice()) {
        if(!properties.contains("device_id")) {
            properties["device_id"] = device->getTemporaryTelemetryDeviceId();
        }
    }
}

}  // namespace

std::string Telemetry::getTemporaryTelemetryDeviceId(const std::string& mxid) {
    return temporaryIdsManager().getDeviceId(mxid);
}

bool Telemetry::isTelemetryEnabled() {
    auto value = trim(readEnv("DEPTHAI_TELEMETRY"));
    if(value.empty()) {
        return true;
    }

    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value != "0" && value != "false" && value != "off" && value != "no";
}

void Telemetry::setTelemetryUsesPython(bool value) {
    telemetryUsesPython.store(value);
}

void Telemetry::emitDepthaiTelemetryLoadEvent() {
    Telemetry::getInstance().event("depthai_load",
                                   nlohmann::json{
                                       {"host_os", getTelemetryHostOS()},
                                       {"host_os_version", getTelemetryHostOSVersion()},
                                       {"is_oak_app", !readEnv("OAKAGENT_PRIVATE_HTTP_PWD").empty()},
                                       {"uses_python", telemetryUsesPython.load()},
                                   });
}

void Telemetry::event(std::string eventName, nlohmann::json properties) {
    normalizeTelemetryProperties(properties);
    if(impl) {
        impl->event(std::move(eventName), std::move(properties));
    }
}

void Telemetry::event(const DeviceBase& device, std::string eventName, nlohmann::json properties) {
    addDeviceTelemetryProperties(device, properties);
    if(impl) {
        impl->event(std::move(eventName), std::move(properties));
    }
}

void Telemetry::event(const Pipeline& pipeline, std::string eventName, nlohmann::json properties) {
    addPipelineTelemetryProperties(pipeline, properties);
    if(impl) {
        impl->event(std::move(eventName), std::move(properties));
    }
}

void Telemetry::addAggregateMetrics(AggregateMetricsCallback functionLikeCb) {
    if(impl) {
        impl->addAggregateMetrics(std::move(functionLikeCb));
    }
}

}  // namespace utility
}  // namespace dai
