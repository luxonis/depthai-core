#include <arpa/inet.h>
#include <httplib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#include "depthai/device/DeviceGate.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {

class DiscoveryReply {
    int socket = -1;
    std::thread worker;

   public:
    DiscoveryReply() {
        socket = ::socket(AF_INET, SOCK_DGRAM, 0);
        if(socket < 0) throw std::runtime_error("Cannot open discovery fixture socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(11491);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if(bind(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            close(socket);
            throw std::runtime_error("Loopback discovery test port is in use");
        }
        timeval timeout{2, 0};
        setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        worker = std::thread([this] {
            uint32_t command = 0;
            sockaddr_in sender{};
            socklen_t senderSize = sizeof(sender);
            if(recvfrom(socket, &command, sizeof(command), 0, reinterpret_cast<sockaddr*>(&sender), &senderSize) != sizeof(command) || command != 1) {
                return;
            }
            struct Reply {
                uint32_t command = 4;
                std::array<char, 32> id{};
                uint32_t state = 5;
                uint32_t protocol = 4;
                uint32_t platform = 5;
                uint32_t status = 0;
                uint16_t httpPort = 11492;
                uint16_t httpsPort = 0;
            } reply;
            static_assert(sizeof(Reply) == 56);
            std::strcpy(reply.id.data(), "RVC2NG-GATE-TEST");
            sendto(socket, &reply, sizeof(reply), 0, reinterpret_cast<const sockaddr*>(&sender), senderSize);
        });
    }

    ~DiscoveryReply() {
        if(worker.joinable()) worker.join();
        close(socket);
    }
};

class FirmwareOverride {
    std::optional<std::string> previousEnvironment;
    std::filesystem::path directory;

   public:
    std::filesystem::path firmware;

    FirmwareOverride() {
        if(const auto* value = std::getenv("DEPTHAI_DEVICE_RVC2NG_FWP")) previousEnvironment = value;
        auto pattern = (std::filesystem::temp_directory_path() / "depthai-rvc2ng-gate-XXXXXX").string();
        if(!mkdtemp(pattern.data())) throw std::runtime_error("Cannot create Gate firmware fixture directory");
        directory = pattern;
        firmware = directory / "firmware.tar.xz";
        std::ofstream(firmware, std::ios::binary) << "rvc2ng-test-package";
        setenv("DEPTHAI_DEVICE_RVC2NG_FWP", firmware.c_str(), 1);
    }

    ~FirmwareOverride() {
        if(previousEnvironment) {
            setenv("DEPTHAI_DEVICE_RVC2NG_FWP", previousEnvironment->c_str(), 1);
        } else {
            unsetenv("DEPTHAI_DEVICE_RVC2NG_FWP");
        }
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
};

class GateFixture {
   public:
    FirmwareOverride firmwareOverride;
    httplib::Server server;
    std::thread thread;
    std::vector<std::string> requests;
    std::string upload;
    nlohmann::json creation;
    std::string state = "CREATED";

    GateFixture() {
        server.new_task_queue = [] { return new httplib::ThreadPool(1); };
        server.set_socket_options([](auto socket) {
            const int reuseAddress = 1;
            setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuseAddress, sizeof(reuseAddress));
        });
        server.Get("/api/v1/status", [](const auto&, auto& response) { response.set_content(R"({"status":true})", "application/json"); });
        server.Post("/api/v1/sessions", [this](const auto& request, auto& response) {
            creation = nlohmann::json::parse(request.body);
            requests.push_back("create");
            response.set_content(R"({"id":"test-session","fwp_exists":false})", "application/json");
        });
        server.Post("/api/v1/sessions/test-session/fwp", [this](const auto& request, auto& response) {
            requests.push_back("upload");
            upload = request.get_file_value("file").content;
            response.status = 200;
        });
        server.Post("/api/v1/sessions/test-session/start", [this](const auto&, auto& response) {
            requests.push_back("start");
            state = "RUNNING";
            response.status = 200;
        });
        server.Get("/api/v1/sessions/test-session",
                   [this](const auto&, auto& response) { response.set_content(nlohmann::json{{"state", state}}.dump(), "application/json"); });
        server.Post("/api/v1/sessions/test-session/destroy", [this](const auto&, auto& response) {
            requests.push_back("destroy");
            state = "DESTROYED";
            response.status = 200;
        });
        if(!server.bind_to_port("127.0.0.1", 11492)) {
            throw std::runtime_error("Loopback Gate test port is in use");
        }
        thread = std::thread([this] { server.listen_after_bind(); });
    }

    ~GateFixture() {
        server.stop();
        if(thread.joinable()) thread.join();
    }
};

}

TEST_CASE("RVC2NG uses the Gate firmware and session lifecycle", "[platform][gate]") {
    GateFixture fixture;
    dai::DeviceInfo requirements("127.0.0.1");
    requirements.platform = X_LINK_RVC2NG;
    requirements.state = X_LINK_GATE;
    deviceDesc_t found{};
    {
        DiscoveryReply discovery;
        REQUIRE(dai::XLinkConnection::findFirstSuitableDevice(requirements, found) == X_LINK_SUCCESS);
    }
    dai::DeviceInfo info(found);
    REQUIRE(info.platform == X_LINK_RVC2NG);
    REQUIRE(info.getDeviceId() == "RVC2NG-GATE-TEST");
    REQUIRE(info.toString().find("X_LINK_RVC2NG") != std::string::npos);
    dai::DeviceGate gate(info);
    REQUIRE(gate.isOkay());
    std::filesystem::remove(fixture.firmwareOverride.firmware);
    REQUIRE_THROWS_WITH(gate.createSession(), "File at path " + fixture.firmwareOverride.firmware.string() + " doesn't exist.");
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
    std::ofstream(fixture.firmwareOverride.firmware, std::ios::binary) << "rvc2ng-test-package";
    REQUIRE(gate.createSession());
    REQUIRE(gate.startSession());
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::RUNNING);
    REQUIRE(gate.destroySession());
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::DESTROYED);
    fixture.server.stop();
    fixture.thread.join();
    REQUIRE(fixture.upload == "rvc2ng-test-package");
    REQUIRE(fixture.requests == std::vector<std::string>{"create", "upload", "start", "destroy"});
    REQUIRE_FALSE(fixture.creation.at("fwp_version").get<std::string>().empty());
    REQUIRE_FALSE(fixture.creation.contains("fwp_checksum"));
}

TEST_CASE("RVC2NG rejects an unset firmware package before creating a session", "[.][platform][gate][unset-firmware]") {
    GateFixture fixture;
    unsetenv("DEPTHAI_DEVICE_RVC2NG_FWP");
    dai::DeviceInfo info("127.0.0.1");
    info.platform = X_LINK_RVC2NG;
    info.state = X_LINK_GATE;
    dai::DeviceGate gate(info);
    REQUIRE(gate.isOkay());
    REQUIRE_THROWS_WITH(gate.createSession(), "No embedded device FWP is configured. Provide firmware with DEPTHAI_DEVICE_RVC2NG_FWP.");
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
    fixture.server.stop();
    fixture.thread.join();
    REQUIRE(fixture.requests.empty());
    REQUIRE(fixture.upload.empty());
}
