#include <httplib.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <filesystem>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "subprocess.hpp"
#include "utility/Platform.hpp"

namespace {

constexpr int kPort = 8955;
constexpr char kTelemetryUrl[] = "http://localhost:8955";
constexpr char kTelemetryApiKey[] = "phc_depthai_telemetry_test";
constexpr auto kRequestTimeout = std::chrono::seconds(20);

using Json = nlohmann::json;

subprocess::env_string_t makeEnvString(const std::string& value) {
#ifdef _MSC_VER
    return std::wstring(value.begin(), value.end());
#else
    return value;
#endif
}

class ScopedTempDir {
   public:
    explicit ScopedTempDir(const std::string&) : path(dai::platform::getTempPath()) {}

    ~ScopedTempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }

    const std::filesystem::path& get() const {
        return path;
    }

   private:
    std::filesystem::path path;
};

struct ChildResult {
    int status = -1;
    std::string stdoutText;
    std::string stderrText;
    bool wasKilled = false;
};

struct ReceivedRequest {
    std::string path;
    std::string userAgent;
    std::string rawBody;
    Json body;
    std::string parseError;
};

class LocalTelemetryServer {
   public:
    void start() {
        server.Post(R"(/capture/?)", [this](const httplib::Request& req, httplib::Response& res) {
            ReceivedRequest captured;
            captured.path = req.path;
            captured.userAgent = req.get_header_value("User-Agent");
            captured.rawBody = req.body;

            try {
                captured.body = Json::parse(req.body);
            } catch(const std::exception& ex) {
                captured.parseError = ex.what();
            }

            {
                std::lock_guard<std::mutex> lock(mutex);
                requests.push_back(std::move(captured));
            }
            condition.notify_all();

            res.status = 200;
            res.set_content("ok", "text/plain");
        });

        const auto boundPort = server.bind_to_port("0.0.0.0", kPort);
        INFO("Failed to bind telemetry server to port " << kPort);
        REQUIRE(boundPort >= 0);

        thread = std::thread([this]() {
            if(!server.listen_after_bind()) {
                std::lock_guard<std::mutex> lock(mutex);
                listenFailed = true;
                condition.notify_all();
            }
        });
    }

    void stop() {
        server.stop();
        if(thread.joinable()) {
            thread.join();
        }
    }

    bool waitForRequiredEvents(const std::set<std::string>& requiredEvents, std::chrono::steady_clock::duration timeout) {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, timeout, [&] {
            std::set<std::string> seen;
            for(const auto& request : requests) {
                if(request.body.is_object()) {
                    seen.insert(request.body.value("event", std::string{}));
                }
            }
            return std::includes(seen.begin(), seen.end(), requiredEvents.begin(), requiredEvents.end()) || listenFailed;
        });
    }

    std::vector<ReceivedRequest> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        return requests;
    }

    bool hadListenFailure() const {
        std::lock_guard<std::mutex> lock(mutex);
        return listenFailed;
    }

   private:
    httplib::Server server;
    std::thread thread;
    mutable std::mutex mutex;
    std::condition_variable condition;
    std::vector<ReceivedRequest> requests;
    bool listenFailed{false};
};

subprocess::env_map_t makeChildEnv(const std::filesystem::path& tempHome) {
    return {
        {makeEnvString("DEPTHAI_TELEMETRY_URL"), makeEnvString(kTelemetryUrl)},
        {makeEnvString("DEPTHAI_TELEMETRY_API_KEY"), makeEnvString(kTelemetryApiKey)},
        {makeEnvString("OAKAGENT_PRIVATE_HTTP_PWD"), makeEnvString("telemetry-test")},
        {makeEnvString("HOME"), makeEnvString(tempHome.string())},
    };
}

ChildResult collectChildResult(subprocess::Popen& proc, bool wasKilled) {
    auto childOutput = proc.communicate();
    return {
        proc.retcode(),
        std::string(childOutput.first.buf.begin(), childOutput.first.buf.end()),
        std::string(childOutput.second.buf.begin(), childOutput.second.buf.end()),
        wasKilled,
    };
}

const ReceivedRequest& getSingleEventRequest(const std::vector<ReceivedRequest>& requests, const std::string& eventName) {
    const ReceivedRequest* match = nullptr;
    std::size_t matchCount = 0;
    for(const auto& request : requests) {
        if(request.body.is_object() && request.body.value("event", std::string{}) == eventName) {
            match = &request;
            ++matchCount;
        }
    }

    INFO("Expected exactly one telemetry event named '" << eventName << "'");
    CAPTURE(matchCount);
    REQUIRE(matchCount == 1);

    return *match;
}

std::vector<ReceivedRequest> getEventRequests(const std::vector<ReceivedRequest>& requests, const std::string& eventName) {
    std::vector<ReceivedRequest> matches;
    for(const auto& request : requests) {
        if(request.body.is_object() && request.body.value("event", std::string{}) == eventName) {
            matches.push_back(request);
        }
    }
    return matches;
}

void expectCommonEventShape(const ReceivedRequest& request) {
    INFO("Telemetry request raw body: " << request.rawBody);
    INFO("Telemetry request path: " << request.path);
    INFO("Telemetry request User-Agent: " << request.userAgent);
    REQUIRE(request.parseError.empty());
    REQUIRE((request.path == "/capture/" || request.path == "/capture"));
    REQUIRE(request.userAgent.rfind("depthai-core/", 0) == 0);
    REQUIRE(request.body.is_object());
    REQUIRE(request.body.value("api_key", std::string{}) == kTelemetryApiKey);
    REQUIRE_FALSE(request.body.value("timestamp", std::string{}).empty());
    REQUIRE_FALSE(request.body.value("distinct_id", std::string{}).empty());

    const auto properties = request.body.value("properties", Json::object());
    INFO("Telemetry properties: " << properties.dump());
    REQUIRE(properties.is_object());
    REQUIRE(properties.value("$lib", std::string{}) == "depthai-core");
    REQUIRE_FALSE(properties.value("$lib_version", std::string{}).empty());
    REQUIRE_FALSE(properties.value("$session_id", std::string{}).empty());
    REQUIRE(properties.value("source_product", std::string{}) == "depthai");
    REQUIRE(properties.value("source_component", std::string{}) == "depthai-core");
    REQUIRE(properties.contains("$process_person_profile"));
    REQUIRE(properties["$process_person_profile"].is_boolean());
    REQUIRE_FALSE(properties["$process_person_profile"].get<bool>());
}

void expectIntegerProperty(const Json& value, const std::string& key) {
    INFO("Checking telemetry integer property '" << key << "' in " << value.dump());
    REQUIRE(value.contains(key));
    REQUIRE(value.at(key).is_number_integer());
}

void expectPipelineSchemaProperty(const Json& properties) {
    REQUIRE(properties.contains("pipeline_schema"));
    const auto& pipelineSchema = properties["pipeline_schema"];
    INFO("Pipeline schema: " << pipelineSchema.dump());
    REQUIRE(pipelineSchema.is_object());
    REQUIRE(pipelineSchema.contains("nodes"));
    REQUIRE(pipelineSchema["nodes"].is_array());
    REQUIRE(pipelineSchema.contains("connections"));
    REQUIRE(pipelineSchema["connections"].is_array());
    REQUIRE(pipelineSchema.contains("bridges"));
    REQUIRE(pipelineSchema["bridges"].is_array());
}

std::vector<ReceivedRequest> runTelemetryScenario() {
#ifndef DEPTHAI_TELEMETRY_TEST_CHILD_PATH
    FAIL("DEPTHAI_TELEMETRY_TEST_CHILD_PATH was not defined");
#endif

    LocalTelemetryServer server;
    server.start();

    const ScopedTempDir tempHome("depthai-telemetry-test");
    std::filesystem::create_directories(tempHome.get());
    const auto executableString = std::filesystem::path(DEPTHAI_TELEMETRY_TEST_CHILD_PATH).string();
    auto childEnv = makeChildEnv(tempHome.get());
    subprocess::Popen proc({executableString},
                           subprocess::output{subprocess::PIPE},
                           subprocess::error{subprocess::PIPE},
                           subprocess::environment{std::move(childEnv)},
                           subprocess::session_leader{true});

    const std::set<std::string> requiredEvents = {
        "depthai_load", "depthai_device_constructor", "depthai_camera_sensor_mode_started", "depthai_pipeline_start", "depthai_pipeline_stop"};
    const bool waitedForEvents = server.waitForRequiredEvents(requiredEvents, kRequestTimeout);
    const auto requests = server.snapshot();
    const bool listenFailed = server.hadListenFailure();
    int status = proc.poll();
    bool childKilled = false;
    if(status == -1) {
        proc.kill();
        childKilled = true;
        const auto killDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while((status = proc.poll()) == -1 && std::chrono::steady_clock::now() < killDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    const auto childResult = collectChildResult(proc, childKilled);

    CAPTURE(childResult.status);
    CAPTURE(childResult.wasKilled);
    INFO("Child stdout:\n" << childResult.stdoutText);
    INFO("Child stderr:\n" << childResult.stderrText);
    if(!waitedForEvents) {
        std::set<std::string> seenEvents;
        for(const auto& request : requests) {
            if(request.body.is_object()) {
                seenEvents.insert(request.body.value("event", std::string{}));
            }
        }

        std::string message = "Timed out waiting for required telemetry events. Seen:";
        for(const auto& event : seenEvents) {
            message += " " + event;
        }
        if(listenFailed) {
            message += " (server listener failed)";
        }
        INFO(message);
    }
    server.stop();

    REQUIRE(waitedForEvents);
    REQUIRE_FALSE(listenFailed);
    if(!childResult.wasKilled) {
        REQUIRE(childResult.status == 0);
    }
    return requests;
}

std::vector<ReceivedRequest> runTelemetryDisabledScenario() {
#ifndef DEPTHAI_TELEMETRY_TEST_CHILD_PATH
    FAIL("DEPTHAI_TELEMETRY_TEST_CHILD_PATH was not defined");
#endif

    LocalTelemetryServer server;
    server.start();

    const ScopedTempDir tempHome("depthai-telemetry-disabled-test");
    std::filesystem::create_directories(tempHome.get());
    const auto executableString = std::filesystem::path(DEPTHAI_TELEMETRY_TEST_CHILD_PATH).string();
    auto childEnv = makeChildEnv(tempHome.get());
    childEnv[makeEnvString("DEPTHAI_TELEMETRY")] = makeEnvString("0");
    subprocess::Popen proc({executableString},
                           subprocess::output{subprocess::PIPE},
                           subprocess::error{subprocess::PIPE},
                           subprocess::environment{std::move(childEnv)},
                           subprocess::session_leader{true});

    const auto deadline = std::chrono::steady_clock::now() + kRequestTimeout;
    int status = proc.poll();
    bool childKilled = false;
    while(status == -1 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        status = proc.poll();
    }
    if(status == -1) {
        proc.kill();
        childKilled = true;
        const auto killDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while((status = proc.poll()) == -1 && std::chrono::steady_clock::now() < killDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const auto requests = server.snapshot();
    const bool listenFailed = server.hadListenFailure();
    const auto childResult = collectChildResult(proc, childKilled);
    server.stop();

    CAPTURE(childResult.status);
    CAPTURE(childResult.wasKilled);
    INFO("Child stdout:\n" << childResult.stdoutText);
    INFO("Child stderr:\n" << childResult.stderrText);
    REQUIRE_FALSE(listenFailed);
    REQUIRE_FALSE(childResult.wasKilled);
    REQUIRE(childResult.status == 0);

    return requests;
}

void validateRequests(const std::vector<ReceivedRequest>& requests) {
    CAPTURE(requests.size());
    REQUIRE(requests.size() >= 5);

    std::map<std::string, int> counts;
    std::set<std::string> sessionIds;

    for(const auto& request : requests) {
        expectCommonEventShape(request);

        const auto eventName = request.body.value("event", std::string{});
        INFO("Telemetry event: " << eventName);

        const auto properties = request.body["properties"];
        sessionIds.insert(properties.value("$session_id", std::string{}));
        counts[eventName] += 1;
    }

    CAPTURE(sessionIds.size());
    REQUIRE(sessionIds.size() == 1);

    const auto& depthaiLoad = getSingleEventRequest(requests, "depthai_load");
    const auto& deviceConstructor = getSingleEventRequest(requests, "depthai_device_constructor");
    const auto& cameraSensorModeStarted = getSingleEventRequest(requests, "depthai_camera_sensor_mode_started");
    const auto& pipelineStart = getSingleEventRequest(requests, "depthai_pipeline_start");
    const auto& pipelineStop = getSingleEventRequest(requests, "depthai_pipeline_stop");
    const auto depthaiNodeCreatedRequests = getEventRequests(requests, "depthai_node_created");
    REQUIRE_FALSE(depthaiNodeCreatedRequests.empty());
    const auto depthaiLoadProperties = depthaiLoad.body["properties"];
    REQUIRE_FALSE(depthaiLoadProperties.value("$session_id", std::string{}).empty());
    REQUIRE(depthaiLoadProperties.find("session_id") == depthaiLoadProperties.end());
    REQUIRE((depthaiLoadProperties.value("host_os", std::string{}) == "windows" || depthaiLoadProperties.value("host_os", std::string{}) == "linux"
             || depthaiLoadProperties.value("host_os", std::string{}) == "mac" || depthaiLoadProperties.value("host_os", std::string{}) == "oakapp"));
    REQUIRE_FALSE(depthaiLoadProperties.value("host_os_version", std::string{}).empty());
    REQUIRE(depthaiLoadProperties.value("is_oak_app", false));
    REQUIRE_FALSE(depthaiLoadProperties.value("uses_python", true));
    const auto deviceConstructorProperties = deviceConstructor.body["properties"];
    REQUIRE_FALSE(deviceConstructorProperties.value("$session_id", std::string{}).empty());
    REQUIRE(deviceConstructorProperties.find("session_id") == deviceConstructorProperties.end());
    REQUIRE_FALSE(deviceConstructorProperties.value("device_id", std::string{}).empty());
    REQUIRE_FALSE(deviceConstructorProperties.value("device_model", std::string{}).empty());
    REQUIRE_FALSE(deviceConstructorProperties.value("platform", std::string{}).empty());
    REQUIRE(
        (deviceConstructorProperties.value("protocol", std::string{}) == "usb" || deviceConstructorProperties.value("protocol", std::string{}) == "ethernet"));
    REQUIRE_FALSE(deviceConstructorProperties.value("protocol_speed", std::string{}).empty());
    REQUIRE(deviceConstructorProperties.contains("standalone"));
    REQUIRE(deviceConstructorProperties["standalone"].is_boolean());
    const auto cameraSensorModeStartedProperties = cameraSensorModeStarted.body["properties"];
    REQUIRE_FALSE(cameraSensorModeStartedProperties.value("socket", std::string{}).empty());
    expectIntegerProperty(cameraSensorModeStartedProperties, "width");
    expectIntegerProperty(cameraSensorModeStartedProperties, "height");
    REQUIRE(cameraSensorModeStartedProperties.contains("fps"));
    REQUIRE(cameraSensorModeStartedProperties["fps"].is_number());
    REQUIRE((cameraSensorModeStartedProperties.value("fsync_mode", std::string{}) == "none"
             || cameraSensorModeStartedProperties.value("fsync_mode", std::string{}) == "input"
             || cameraSensorModeStartedProperties.value("fsync_mode", std::string{}) == "output"
             || cameraSensorModeStartedProperties.value("fsync_mode", std::string{}) == "ptp"));
    REQUIRE(cameraSensorModeStartedProperties.contains("hdr_enabled"));
    REQUIRE(cameraSensorModeStartedProperties["hdr_enabled"].is_boolean());
    REQUIRE_FALSE(cameraSensorModeStartedProperties.value("pipeline_id", std::string{}).empty());

    for(const auto& request : depthaiNodeCreatedRequests) {
        const auto properties = request.body["properties"];
        REQUIRE_FALSE(properties.value("name", std::string{}).empty());
        REQUIRE(properties.contains("properties"));
        REQUIRE(properties["properties"].is_object());
        REQUIRE_FALSE(properties.value("device_id", std::string{}).empty());
        REQUIRE_FALSE(properties.value("pipeline_id", std::string{}).empty());
    }

    const auto pipelineStartProperties = pipelineStart.body["properties"];
    REQUIRE(pipelineStartProperties.contains("host_only"));
    REQUIRE(pipelineStartProperties["host_only"].is_boolean());
    REQUIRE_FALSE(pipelineStartProperties["host_only"].get<bool>());
    expectPipelineSchemaProperty(pipelineStartProperties);
    REQUIRE_FALSE(pipelineStartProperties.value("device_id", std::string{}).empty());
    REQUIRE_FALSE(pipelineStartProperties.value("pipeline_id", std::string{}).empty());

    const auto pipelineStopProperties = pipelineStop.body["properties"];
    REQUIRE(pipelineStopProperties.contains("host_only"));
    REQUIRE(pipelineStopProperties["host_only"].is_boolean());
    REQUIRE_FALSE(pipelineStopProperties["host_only"].get<bool>());
    expectIntegerProperty(pipelineStopProperties, "duration_ms");
    REQUIRE_FALSE(pipelineStopProperties.value("pipeline_id", std::string{}).empty());

    CAPTURE(counts["depthai_load"]);
    CAPTURE(counts["depthai_device_constructor"]);
    CAPTURE(counts["depthai_camera_sensor_mode_started"]);
    CAPTURE(counts["depthai_pipeline_start"]);
    CAPTURE(counts["depthai_pipeline_stop"]);
    REQUIRE(counts["depthai_load"] == 1);
    REQUIRE(counts["depthai_device_constructor"] == 1);
    REQUIRE(counts["depthai_camera_sensor_mode_started"] == 1);
    REQUIRE(counts["depthai_pipeline_start"] == 1);
    REQUIRE(counts["depthai_pipeline_stop"] == 1);
}

}  // namespace

TEST_CASE("Telemetry can be redirected with DEPTHAI_TELEMETRY_URL", "[ondevice][telemetry]") {
    const auto requests = runTelemetryScenario();
    validateRequests(requests);
}

TEST_CASE("Telemetry can be disabled with DEPTHAI_TELEMETRY", "[ondevice][telemetry]") {
    const auto requests = runTelemetryDisabledScenario();
    CAPTURE(requests.size());
    REQUIRE(requests.empty());
}
