#include <XLink/XLink.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstring>
#include <deque>
#include <nlohmann/json.hpp>
#include <vector>

#include "depthai/build/version.hpp"
#include "depthai/device/DeviceGate.hpp"
#include "device_gate_test_fixtures.hpp"

namespace {

std::vector<uint8_t> bytes(const std::string& text) {
    return {text.begin(), text.end()};
}

std::vector<uint8_t> header(uint32_t command, uint32_t size) {
    const std::array<uint32_t, 2> words{command, size};
    std::vector<uint8_t> result(sizeof(words));
    std::memcpy(result.data(), words.data(), result.size());
    return result;
}

struct UsbFixture;
UsbFixture* activeUsb = nullptr;

struct UsbFixture {
    std::deque<std::vector<uint8_t>> reads;
    std::vector<std::vector<uint8_t>> writes;
    bool failWrite = false;

    UsbFixture() {
        REQUIRE(activeUsb == nullptr);
        activeUsb = this;
    }

    ~UsbFixture() {
        activeUsb = nullptr;
    }

    void respond(const std::string& body = "") {
        reads.push_back(header(0, body.size()));
        if(!body.empty()) reads.push_back(bytes(body));
    }
};

std::string createBody(const std::string& version) {
    return nlohmann::json{{"name", "depthai_session"}, {"fwp_version", version}, {"library_version", dai::build::VERSION}, {"protected", true}}.dump();
}

dai::DeviceInfo usbDevice(XLinkPlatform_t platform) {
    dai::DeviceInfo info;
    info.name = "gate-test";
    info.protocol = X_LINK_USB_EP;
    info.platform = platform;
    info.state = X_LINK_GATE;
    return info;
}

}  // namespace

extern "C" XLinkError_t __wrap_XLinkGateWrite(const char* name, void* data, int size, int) {
    CHECK(std::string(name) == "gate-test");
    if(!activeUsb || size < 0) return X_LINK_ERROR;
    const auto* begin = static_cast<const uint8_t*>(data);
    activeUsb->writes.emplace_back(begin, begin + size);
    return activeUsb->failWrite ? X_LINK_ERROR : X_LINK_SUCCESS;
}

extern "C" XLinkError_t __wrap_XLinkGateRead(const char* name, void* data, int size, int) {
    CHECK(std::string(name) == "gate-test");
    if(!activeUsb || activeUsb->reads.empty() || size < 0) return X_LINK_ERROR;
    auto response = std::move(activeUsb->reads.front());
    activeUsb->reads.pop_front();
    CHECK(response.size() == static_cast<size_t>(size));
    if(response.size() != static_cast<size_t>(size)) return X_LINK_ERROR;
    std::memcpy(data, response.data(), response.size());
    return X_LINK_SUCCESS;
}

TEST_CASE("RVC2NG USB Gate sends firmware with the existing session framing", "[gate][usb]") {
    gate_test::FirmwareOverride firmware;
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC2NG));
    std::filesystem::remove(firmware.firmware);
    REQUIRE_THROWS_WITH(gate.createSession(), "File at path " + firmware.firmware.string() + " doesn't exist.");
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
    REQUIRE(fixture.writes.empty());
    std::ofstream(firmware.firmware, std::ios::binary) << "rvc2ng-test-package";
    fixture.respond(R"({"id":"test-session","fwp_exists":false})");
    fixture.respond();
    fixture.respond(R"({"state":"CREATED"})");
    fixture.respond();
    fixture.respond(R"({"state":"RUNNING"})");
    REQUIRE(gate.createSession());
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::CREATED);
    REQUIRE(gate.startSession());
    REQUIRE(gate.getState() == dai::DeviceGate::SessionState::RUNNING);
    const auto body = createBody(dai::build::DEVICE_RVC2NG_VERSION);
    const uint32_t sessionSize = 12;
    std::vector<uint8_t> sessionSizeBytes(sizeof(sessionSize));
    std::memcpy(sessionSizeBytes.data(), &sessionSize, sizeof(sessionSize));
    REQUIRE(fixture.writes
            == std::vector<std::vector<uint8_t>>{header(2, body.size()),
                                                 bytes(body),
                                                 header(4, 35),
                                                 sessionSizeBytes,
                                                 bytes("test-session"),
                                                 bytes("rvc2ng-test-package"),
                                                 header(10, 12),
                                                 bytes("test-session"),
                                                 header(3, 12),
                                                 bytes("test-session"),
                                                 header(10, 12),
                                                 bytes("test-session")});
    REQUIRE(fixture.reads.empty());
    fixture.failWrite = true;
    dai::DeviceGate disconnected(usbDevice(X_LINK_RVC2NG));
    REQUIRE_FALSE(disconnected.createSession());
    REQUIRE(disconnected.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
}

TEST_CASE("Legacy USB Gate sessions can start with cached firmware", "[gate][usb][legacy-cache]") {
    for(const auto platform : {X_LINK_RVC3, X_LINK_RVC4}) {
        CAPTURE(platform);
        gate_test::FirmwareOverride firmware(platform == X_LINK_RVC3 ? "DEPTHAI_DEVICE_KB_FWP" : "DEPTHAI_DEVICE_RVC4_FWP");
        std::filesystem::remove(firmware.firmware);
        const auto body = createBody(platform == X_LINK_RVC3 ? dai::build::DEVICE_RVC3_VERSION : dai::build::DEVICE_RVC4_VERSION);
        for(const auto cached : {true, false}) {
            CAPTURE(cached);
            UsbFixture fixture;
            fixture.respond(nlohmann::json{{"id", "test-session"}, {"fwp_exists", cached}}.dump());
            dai::DeviceGate gate(usbDevice(platform));
            if(cached) {
                fixture.respond();
                fixture.respond(R"({"state":"RUNNING"})");
                REQUIRE(gate.createSession());
                REQUIRE(gate.startSession());
                REQUIRE(gate.getState() == dai::DeviceGate::SessionState::RUNNING);
                REQUIRE(fixture.writes
                        == std::vector<std::vector<uint8_t>>{
                            header(2, body.size()), bytes(body), header(3, 12), bytes("test-session"), header(10, 12), bytes("test-session")});
            } else {
                REQUIRE_THROWS(gate.createSession());
                REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
                REQUIRE(fixture.writes == std::vector<std::vector<uint8_t>>{header(2, body.size()), bytes(body)});
            }
            REQUIRE(fixture.reads.empty());
        }
    }
}
