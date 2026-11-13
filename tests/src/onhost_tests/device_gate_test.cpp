#include <httplib.h>

#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <regex>
#include <string>
#include <thread>

#include "depthai/device/DeviceGate.hpp"
#include "device/GateWire.hpp"

#if defined(__unix__) || defined(__APPLE__)

namespace {

// DeviceGate connects to this port of the device address
constexpr int gatePort = 11492;

// Answers the create and upload calls of a gate over HTTP and remembers what the client sent
class FakeGate {
   public:
    explicit FakeGate(bool fwpExists) : fwpExists(fwpExists) {
        server.Post("/api/v1/sessions", [this](const httplib::Request& req, httplib::Response& res) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                createBody = nlohmann::json::parse(req.body);
            }
            res.set_content(nlohmann::json{{"id", "1"}, {"fwp_exists", this->fwpExists}}.dump(), "application/json");
        });
        server.Post("/api/v1/sessions/1/fwp", [this](const httplib::Request& req, httplib::Response& res) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                uploaded = req.get_file_value("file").content;
                uploads++;
            }
            res.set_content(R"({"result": true})", "application/json");
        });
    }

    ~FakeGate() {
        server.stop();
        if(thread.joinable()) thread.join();
    }

    bool start() {
        if(!server.bind_to_port("127.0.0.1", gatePort)) return false;
        thread = std::thread([this] { server.listen_after_bind(); });
        server.wait_until_ready();
        return true;
    }

    nlohmann::json getCreateBody() {
        std::lock_guard<std::mutex> lock(mtx);
        return createBody;
    }

    int getUploads() {
        std::lock_guard<std::mutex> lock(mtx);
        return uploads;
    }

    std::string getUploaded() {
        std::lock_guard<std::mutex> lock(mtx);
        return uploaded;
    }

   private:
    const bool fwpExists;
    httplib::Server server;
    std::thread thread;
    std::mutex mtx;
    nlohmann::json createBody;
    std::string uploaded;
    int uploads = 0;
};

// The firmware package is read once per process, so every test case uses the same file
const std::string& firmwarePackage() {
    static const std::string package = [] {
        const std::string content = "bytes of a firmware package, not an archive";
        const auto path = std::filesystem::temp_directory_path() / "device_gate_test_fwp.tar.xz";
        std::ofstream(path, std::ios::binary) << content;
        REQUIRE(::setenv("DEPTHAI_DEVICE_RVC4_FWP", path.string().c_str(), 1) == 0);
        return content;
    }();
    return package;
}

dai::DeviceInfo gateAddress() {
    return dai::DeviceInfo("127.0.0.1", "", X_LINK_GATE, X_LINK_TCP_IP, X_LINK_RVC4, X_LINK_SUCCESS);
}

}  // namespace

TEST_CASE("DeviceGate create sends the firmware package checksum and uploads a package the gate lacks") {
    const auto& package = firmwarePackage();
    const auto expectedChecksum = dai::gatewire::sha256Hex({package.begin(), package.end()});
    if(expectedChecksum.empty()) SKIP("built without a sha256 implementation");

    FakeGate gate(false);
    if(!gate.start()) SKIP("port 11492 is in use");

    dai::DeviceGate deviceGate(gateAddress());
    REQUIRE(deviceGate.createSession());

    const auto body = gate.getCreateBody();
    REQUIRE(body.at("fwp_checksum") == expectedChecksum);
    REQUIRE(std::regex_match(expectedChecksum, std::regex("[0-9a-f]{64}")));
    REQUIRE(gate.getUploads() == 1);
    REQUIRE(gate.getUploaded() == package);
}

TEST_CASE("DeviceGate create skips the upload when the gate has the package") {
    const auto& package = firmwarePackage();
    const auto expectedChecksum = dai::gatewire::sha256Hex({package.begin(), package.end()});
    if(expectedChecksum.empty()) SKIP("built without a sha256 implementation");

    FakeGate gate(true);
    if(!gate.start()) SKIP("port 11492 is in use");

    dai::DeviceGate deviceGate(gateAddress());
    REQUIRE(deviceGate.createSession());

    REQUIRE(gate.getCreateBody().at("fwp_checksum") == expectedChecksum);
    REQUIRE(gate.getUploads() == 0);
}

#else

TEST_CASE("DeviceGate create against a fake gate") {
    SKIP("POSIX only");
}

#endif
