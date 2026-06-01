#include <httplib.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <exception>
#include <filesystem>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "subprocess.hpp"
#include "utility/Platform.hpp"

namespace {

constexpr int kPort = 8955;
constexpr char kTelemetryUrl[] = "http://localhost:8955";
constexpr char kTelemetryApiKey[] = "phc_depthai_telemetry_multi_device_test";
constexpr auto kChildTimeout = std::chrono::seconds(120);
constexpr auto kPostExitDrainTimeout = std::chrono::seconds(5);
constexpr auto kPostExitQuietPeriod = std::chrono::milliseconds(500);

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

std::vector<ReceivedRequest> getEventRequests(const std::vector<ReceivedRequest>& requests, const std::string& eventName) {
    std::vector<ReceivedRequest> matches;
    for(const auto& request : requests) {
        if(request.body.is_object() && request.body.value("event", std::string{}) == eventName) {
            matches.push_back(request);
        }
    }
    return matches;
}

void waitForRequestDrain(LocalTelemetryServer& server) {
    auto lastCount = server.snapshot().size();
    auto stableSince = std::chrono::steady_clock::now();
    const auto deadline = std::chrono::steady_clock::now() + kPostExitDrainTimeout;

    while(std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto currentCount = server.snapshot().size();
        if(currentCount != lastCount) {
            lastCount = currentCount;
            stableSince = std::chrono::steady_clock::now();
            continue;
        }
        if(std::chrono::steady_clock::now() - stableSince >= kPostExitQuietPeriod) {
            break;
        }
    }
}

std::vector<ReceivedRequest> runTelemetryScenario() {
#ifndef DEPTHAI_TELEMETRY_MULTI_DEVICE_TEST_CHILD_PATH
    FAIL("DEPTHAI_TELEMETRY_MULTI_DEVICE_TEST_CHILD_PATH was not defined");
#endif

    LocalTelemetryServer server;
    server.start();

    const ScopedTempDir tempHome("depthai-telemetry-multi-device-test");
    std::filesystem::create_directories(tempHome.get());
    const auto executableString = std::filesystem::path(DEPTHAI_TELEMETRY_MULTI_DEVICE_TEST_CHILD_PATH).string();
    auto childEnv = makeChildEnv(tempHome.get());
    subprocess::Popen proc({executableString},
                           subprocess::output{subprocess::PIPE},
                           subprocess::error{subprocess::PIPE},
                           subprocess::environment{std::move(childEnv)},
                           subprocess::session_leader{true});

    const auto deadline = std::chrono::steady_clock::now() + kChildTimeout;
    int status = proc.poll();
    bool childKilled = false;
    while(status == -1 && !server.hadListenFailure() && std::chrono::steady_clock::now() < deadline) {
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

    const auto childResult = collectChildResult(proc, childKilled);
    CAPTURE(childResult.status);
    CAPTURE(childResult.wasKilled);
    INFO("Child stdout:\n" << childResult.stdoutText);
    INFO("Child stderr:\n" << childResult.stderrText);

    waitForRequestDrain(server);

    const auto requests = server.snapshot();
    const bool listenFailed = server.hadListenFailure();
    server.stop();

    REQUIRE_FALSE(listenFailed);
    if(!childResult.wasKilled) {
        REQUIRE(childResult.status == 0);
    } else {
        CAPTURE(childResult.status);
        REQUIRE(childResult.status == 0);
    }
    return requests;
}

void validateRequests(const std::vector<ReceivedRequest>& requests) {
    CAPTURE(requests.size());
    REQUIRE_FALSE(requests.empty());

    std::set<std::string> sessionIds;

    for(const auto& request : requests) {
        expectCommonEventShape(request);

        const auto eventName = request.body.value("event", std::string{});
        INFO("Telemetry event: " << eventName);
        sessionIds.insert(request.body["properties"].value("$session_id", std::string{}));
    }

    CAPTURE(sessionIds.size());
    REQUIRE(sessionIds.size() == 1);

    const auto depthaiLoadRequests = getEventRequests(requests, "depthai_load");
    CAPTURE(depthaiLoadRequests.size());
    REQUIRE(depthaiLoadRequests.size() == 1);
    const auto depthaiLoadProperties = depthaiLoadRequests.front().body["properties"];
    REQUIRE_FALSE(depthaiLoadProperties.value("$session_id", std::string{}).empty());
    REQUIRE(depthaiLoadProperties.find("session_id") == depthaiLoadProperties.end());
    REQUIRE((depthaiLoadProperties.value("host_os", std::string{}) == "windows" || depthaiLoadProperties.value("host_os", std::string{}) == "linux"
             || depthaiLoadProperties.value("host_os", std::string{}) == "mac" || depthaiLoadProperties.value("host_os", std::string{}) == "oakapp"));
    REQUIRE_FALSE(depthaiLoadProperties.value("host_os_version", std::string{}).empty());
    REQUIRE(depthaiLoadProperties.value("is_oak_app", false));
    REQUIRE_FALSE(depthaiLoadProperties.value("uses_python", true));

    const auto deviceConstructorRequests = getEventRequests(requests, "depthai_device_constructor");
    CAPTURE(deviceConstructorRequests.size());
    REQUIRE_FALSE(deviceConstructorRequests.empty());
    const auto deviceCount = deviceConstructorRequests.size();

    std::set<std::string> telemetryDeviceIds;

    for(const auto& request : deviceConstructorRequests) {
        const auto properties = request.body["properties"];
        REQUIRE_FALSE(properties.value("$session_id", std::string{}).empty());
        REQUIRE(properties.find("session_id") == properties.end());
        const auto deviceId = properties.value("device_id", std::string{});
        REQUIRE_FALSE(deviceId.empty());
        REQUIRE_FALSE(properties.value("device_model", std::string{}).empty());
        REQUIRE_FALSE(properties.value("platform", std::string{}).empty());
        REQUIRE((properties.value("protocol", std::string{}) == "usb" || properties.value("protocol", std::string{}) == "ethernet"));
        REQUIRE_FALSE(properties.value("protocol_speed", std::string{}).empty());
        REQUIRE(properties.contains("standalone"));
        REQUIRE(properties["standalone"].is_boolean());
        telemetryDeviceIds.insert(deviceId);
    }

    CAPTURE(telemetryDeviceIds.size());
    REQUIRE(telemetryDeviceIds.size() == deviceCount);

    const auto validateTelemetryDeviceRequests = [&](const std::string& eventName, auto&& validator) {
        const auto eventRequests = getEventRequests(requests, eventName);
        CAPTURE(eventName);
        CAPTURE(eventRequests.size());
        REQUIRE(eventRequests.size() == deviceCount);

        std::map<std::string, int> perEventDeviceCounts;
        for(const auto& request : eventRequests) {
            const auto properties = request.body["properties"];
            const auto deviceId = properties.value("device_id", std::string{});
            REQUIRE(telemetryDeviceIds.count(deviceId) == 1);
            perEventDeviceCounts[deviceId] += 1;
            validator(properties);
        }

        for(const auto& deviceId : telemetryDeviceIds) {
            CAPTURE(eventName);
            CAPTURE(deviceId);
            REQUIRE(perEventDeviceCounts[deviceId] == 1);
        }
    };

    validateTelemetryDeviceRequests("depthai_camera_sensor_mode_started", [&](const Json& properties) {
        REQUIRE_FALSE(properties.value("socket", std::string{}).empty());
        expectIntegerProperty(properties, "width");
        expectIntegerProperty(properties, "height");
        REQUIRE(properties.contains("fps"));
        REQUIRE(properties["fps"].is_number());
        REQUIRE((properties.value("fsync_mode", std::string{}) == "none" || properties.value("fsync_mode", std::string{}) == "input"
                 || properties.value("fsync_mode", std::string{}) == "output" || properties.value("fsync_mode", std::string{}) == "ptp"));
        REQUIRE(properties.contains("hdr_enabled"));
        REQUIRE(properties["hdr_enabled"].is_boolean());
        REQUIRE_FALSE(properties.value("pipeline_id", std::string{}).empty());
    });

    const auto depthaiNodeCreatedRequests = getEventRequests(requests, "depthai_node_created");
    CAPTURE(depthaiNodeCreatedRequests.size());
    REQUIRE_FALSE(depthaiNodeCreatedRequests.empty());
    for(const auto& request : depthaiNodeCreatedRequests) {
        const auto properties = request.body["properties"];
        REQUIRE_FALSE(properties.value("name", std::string{}).empty());
        REQUIRE(properties.contains("properties"));
        REQUIRE(properties["properties"].is_object());
        REQUIRE(telemetryDeviceIds.count(properties.value("device_id", std::string{})) == 1);
        REQUIRE_FALSE(properties.value("pipeline_id", std::string{}).empty());
    }

    const auto validatePipelineRequests = [&](const std::string& eventName, auto&& validator) {
        const auto eventRequests = getEventRequests(requests, eventName);
        CAPTURE(eventName);
        CAPTURE(eventRequests.size());
        REQUIRE(eventRequests.size() == deviceCount);

        std::set<std::string> rawDeviceIds;
        for(const auto& request : eventRequests) {
            const auto properties = request.body["properties"];
            const auto deviceId = properties.value("device_id", std::string{});
            REQUIRE_FALSE(deviceId.empty());
            rawDeviceIds.insert(deviceId);
            validator(properties);
        }
        CAPTURE(eventName);
        CAPTURE(rawDeviceIds.size());
        REQUIRE(rawDeviceIds.size() == deviceCount);
        return rawDeviceIds;
    };

    const auto pipelineStartDeviceIds = validatePipelineRequests("depthai_pipeline_start", [&](const Json& properties) {
        REQUIRE(properties.contains("host_only"));
        REQUIRE(properties["host_only"].is_boolean());
        REQUIRE_FALSE(properties["host_only"].get<bool>());
        expectPipelineSchemaProperty(properties);
        REQUIRE_FALSE(properties.value("pipeline_id", std::string{}).empty());
    });

    const auto pipelineStopDeviceIds = validatePipelineRequests("depthai_pipeline_stop", [&](const Json& properties) {
        REQUIRE(properties.contains("host_only"));
        REQUIRE(properties["host_only"].is_boolean());
        REQUIRE_FALSE(properties["host_only"].get<bool>());
        expectIntegerProperty(properties, "duration_ms");
        REQUIRE_FALSE(properties.value("pipeline_id", std::string{}).empty());
    });

    REQUIRE(pipelineStartDeviceIds == pipelineStopDeviceIds);

    validateTelemetryDeviceRequests("depthai_device_destructor", [&](const Json& properties) {
        REQUIRE_FALSE(properties.value("$session_id", std::string{}).empty());
        REQUIRE(properties.find("session_id") == properties.end());
        expectIntegerProperty(properties, "duration_ms");
    });
}

}  // namespace

TEST_CASE("Telemetry can be redirected with DEPTHAI_TELEMETRY_URL across multiple devices", "[ondevice][telemetry][multidevice]") {
    const auto requests = runTelemetryScenario();
    validateRequests(requests);
}
