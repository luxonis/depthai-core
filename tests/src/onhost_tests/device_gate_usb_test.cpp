#include <XLink/XLink.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
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
    bool failBegin = false;
    bool failRead = false;
    bool owned = false;
    unsigned begins = 0;
    std::vector<unsigned> writeTransactions;
    std::vector<XLinkGateOutcome> outcomes;

    UsbFixture() {
        REQUIRE(activeUsb == nullptr);
        activeUsb = this;
    }

    ~UsbFixture() {
        CHECK_FALSE(owned);
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

extern "C" XLinkError_t __wrap_XLinkGateBegin(const char* name, XLinkGateTransaction** transaction) {
    CHECK(std::string(name) == "gate-test");
    *transaction = nullptr;
    if(!activeUsb || activeUsb->failBegin || activeUsb->owned) return X_LINK_ERROR;
    activeUsb->owned = true;
    ++activeUsb->begins;
    *transaction = reinterpret_cast<XLinkGateTransaction*>(activeUsb);
    return X_LINK_SUCCESS;
}

extern "C" XLinkError_t __wrap_XLinkGateEnd(XLinkGateTransaction* transaction, XLinkGateOutcome outcome) {
    REQUIRE(transaction == reinterpret_cast<XLinkGateTransaction*>(activeUsb));
    REQUIRE(activeUsb->owned);
    activeUsb->owned = false;
    activeUsb->outcomes.push_back(outcome);
    return X_LINK_SUCCESS;
}

extern "C" XLinkError_t __wrap_XLinkGateTransactionWrite(XLinkGateTransaction* transaction, const void* data, int size, int) {
    if(!transaction) return X_LINK_ERROR;
    REQUIRE(transaction == reinterpret_cast<XLinkGateTransaction*>(activeUsb));
    REQUIRE(activeUsb->owned);
    if(size < 0) return X_LINK_ERROR;
    const auto* begin = static_cast<const uint8_t*>(data);
    activeUsb->writes.emplace_back(begin, begin + size);
    activeUsb->writeTransactions.push_back(activeUsb->begins);
    return activeUsb->failWrite ? X_LINK_ERROR : X_LINK_SUCCESS;
}

extern "C" XLinkError_t __wrap_XLinkGateTransactionRead(XLinkGateTransaction* transaction, void* data, int size, int) {
    if(!transaction) return X_LINK_ERROR;
    REQUIRE(transaction == reinterpret_cast<XLinkGateTransaction*>(activeUsb));
    REQUIRE(activeUsb->owned);
    if(activeUsb->failRead || activeUsb->reads.empty() || size < 0) return X_LINK_ERROR;
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
    REQUIRE(fixture.writeTransactions == std::vector<unsigned>{1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4});
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>(4, XLINK_GATE_COMPLETE));
    fixture.failWrite = true;
    dai::DeviceGate disconnected(usbDevice(X_LINK_RVC2NG));
    REQUIRE_FALSE(disconnected.createSession());
    REQUIRE(disconnected.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
    const auto start = std::chrono::steady_clock::now();
    disconnected.waitForSessionEnd();
    REQUIRE(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
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

TEST_CASE("Waiting for a failed USB Gate session returns without waiting for creation", "[gate][usb]") {
    for(const auto platform : {X_LINK_RVC3, X_LINK_RVC4}) {
        CAPTURE(platform);
        UsbFixture fixture;
        fixture.failWrite = true;
        dai::DeviceGate gate(usbDevice(platform));
        REQUIRE_FALSE(gate.createSession());
        REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
        const auto writes = fixture.writes.size();
        const auto start = std::chrono::steady_clock::now();
        gate.waitForSessionEnd();
        REQUIRE(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
        REQUIRE(fixture.writes.size() == writes);
        REQUIRE(gate.getState() == dai::DeviceGate::SessionState::NOT_CREATED);
    }
}

TEST_CASE("USB Gate aborts malformed replies before reading their body", "[gate][usb]") {
    for(const auto& response : {header(0, UINT32_MAX), header(0, 1024 * 1024 + 1), header(0, 0), header(2, 0), header(1, 1)}) {
        UsbFixture fixture;
        dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
        fixture.reads.push_back(response);
        REQUIRE_FALSE(gate.isOkay());
        REQUIRE(fixture.reads.empty());
        REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>{XLINK_GATE_ABORTED});
    }
}

TEST_CASE("USB Gate completes application errors and aborts parser failures", "[gate][usb]") {
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
    fixture.reads.push_back(header(1, 0));
    REQUIRE_FALSE(gate.isOkay());
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>{XLINK_GATE_COMPLETE});
    fixture.respond("invalid json");
    REQUIRE_THROWS(gate.isOkay());
    fixture.respond(R"({"status":"wrong type"})");
    REQUIRE_THROWS(gate.isOkay());
    fixture.respond(R"({"status":true})");
    REQUIRE(gate.isOkay());
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>{XLINK_GATE_COMPLETE, XLINK_GATE_ABORTED, XLINK_GATE_ABORTED, XLINK_GATE_COMPLETE});
    REQUIRE(fixture.reads.empty());
}

TEST_CASE("USB Gate releases transactions after transport failures", "[gate][usb]") {
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
    fixture.failBegin = true;
    REQUIRE_FALSE(gate.isOkay());
    REQUIRE(fixture.writes.empty());
    REQUIRE(fixture.outcomes.empty());
    fixture.failBegin = false;
    fixture.failWrite = true;
    REQUIRE_FALSE(gate.isOkay());
    fixture.failWrite = false;
    fixture.failRead = true;
    REQUIRE_FALSE(gate.isOkay());
    fixture.failRead = false;
    fixture.reads.push_back(header(0, 15));
    REQUIRE_FALSE(gate.isOkay());
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>(3, XLINK_GATE_ABORTED));
}

TEST_CASE("USB Gate rejects bodies on acknowledgments", "[gate][usb]") {
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
    fixture.respond(R"({"id":"test-session","fwp_exists":true})");
    REQUIRE(gate.createSession());
    fixture.reads.push_back(header(0, 1));
    REQUIRE_FALSE(gate.startSession());
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>{XLINK_GATE_COMPLETE, XLINK_GATE_ABORTED});
}

TEST_CASE("USB Gate reads version metadata and bounded file responses", "[gate][usb]") {
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
    fixture.respond(R"({"version_gate":"1.2.3","version_os":"test"})");
    auto version = gate.getAllVersion();
    REQUIRE(version.gate == "1.2.3");
    REQUIRE(version.os == "test");
    fixture.respond(R"({"version_gate":"1.2.3"})");
    auto file = std::string(R"({"filename":"dump.bin","data":[1,2,3]})");
    file.resize(1024 * 1024 + 1, ' ');
    fixture.respond(file);
    auto dump = gate.getCrashDump();
    REQUIRE(dump.has_value());
    REQUIRE(dump->data == std::vector<uint8_t>{1, 2, 3});
    fixture.respond(R"({"version_gate":"1.2.3"})");
    fixture.reads.push_back(header(0, UINT32_MAX));
    REQUIRE_FALSE(gate.getCrashDump().has_value());
    REQUIRE(fixture.outcomes
            == std::vector<XLinkGateOutcome>{XLINK_GATE_COMPLETE, XLINK_GATE_COMPLETE, XLINK_GATE_COMPLETE, XLINK_GATE_COMPLETE, XLINK_GATE_ABORTED});
    REQUIRE(fixture.reads.empty());
}

TEST_CASE("USB Gate preserves lifecycle acknowledgments", "[gate][usb]") {
    UsbFixture fixture;
    dai::DeviceGate gate(usbDevice(X_LINK_RVC4));
    fixture.respond(R"({"id":"test-session","fwp_exists":true})");
    REQUIRE(gate.createSession());
    fixture.respond(R"({"state":"RUNNING"})");
    fixture.respond();
    REQUIRE(gate.stopSession());
    fixture.respond(R"({"state":"STOPPED"})");
    fixture.respond(R"({"state":"STOPPED"})");
    fixture.respond();
    REQUIRE(gate.destroySession());
    fixture.respond(R"({"state":"DESTROYED"})");
    fixture.respond();
    REQUIRE(gate.deleteSession());
    REQUIRE(fixture.outcomes == std::vector<XLinkGateOutcome>(8, XLINK_GATE_COMPLETE));
    REQUIRE(fixture.reads.empty());
}
