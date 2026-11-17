#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/EepromError.hpp"

namespace {
using Socket = dai::CameraBoardSocket;
using namespace std::chrono_literals;

// B/C share an I2C bus on this RVC2 fixture. Device selection comes from the
// standard HIL environment; no device ID or firmware path is embedded here.
struct SharedCbaFixture {
    dai::Pipeline pipeline;
    std::shared_ptr<dai::Device> device = pipeline.getDefaultDevice();
    std::vector<Socket> sockets;
    std::map<Socket, std::shared_ptr<dai::MessageQueue>> queues;

    SharedCbaFixture() {
        if(device->getPlatform() != dai::Platform::RVC2 || device->getProductName() != "OAK-FFC-3P") {
            SKIP("Requires an RVC2 OAK-FFC-3P with a shared B/C I2C bus");
        }
        const auto features = device->getConnectedCameraFeatures();
        for(const auto socket : {Socket::CAM_A, Socket::CAM_B, Socket::CAM_C}) {
            const auto feature = std::find_if(features.begin(), features.end(), [socket](const auto& value) { return value.socket == socket; });
            if(feature == features.end()) continue;
            const auto& types = feature->supportedTypes;
            if(std::find(types.begin(), types.end(), dai::CameraSensorType::MONO) != types.end()
               || std::find(types.begin(), types.end(), dai::CameraSensorType::COLOR) != types.end()) {
                sockets.push_back(socket);
            }
        }
        if(!hasCamera(Socket::CAM_B) || !hasCamera(Socket::CAM_C)) SKIP("Requires cameras on both CAM_B and CAM_C");
        device->setMaxReconnectionAttempts(0);
        pipeline.setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    }

    bool hasCamera(Socket socket) const {
        return std::find(sockets.begin(), sockets.end(), socket) != sockets.end();
    }

    void readAllowingUnavailableCalibration(Socket socket) const {
        if(!device->isCBAEepromAvailable(socket)) {
            REQUIRE_THROWS_MATCHES(device->readCBACalibration2(socket),
                                   dai::EepromError,
                                   Catch::Matchers::MessageMatches(Catch::Matchers::ContainsSubstring("EEPROM_NOT_AVAILABLE")));
            return;
        }
        // The strict API exposes rejection. The convenience reader can hide
        // EepromError by returning an empty calibration handler.
        try {
            const auto calibration = device->readCBACalibration2(socket);
            REQUIRE(calibration.hasCalibrationData());
        } catch(const dai::EepromError& error) {
            INFO(error.what());
            REQUIRE_THAT(std::string(error.what()), Catch::Matchers::ContainsSubstring("EEPROM_INVALID_DATA"));
        }
    }

    void start(const std::vector<Socket>& activeSockets) {
        for(const auto socket : activeSockets) {
            auto camera = pipeline.create<dai::node::Camera>()->build(socket);
            auto* output = camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.0f, false);
            queues.emplace(socket, output->createOutputQueue(2, false));
        }
        pipeline.start();
    }

    void requireStreaming(std::chrono::seconds duration) const {
        std::map<Socket, std::size_t> counts;
        for(const auto& entry : queues) {
            entry.second->tryGetAll<dai::ImgFrame>();
            counts.emplace(entry.first, 0);
        }
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while(std::chrono::steady_clock::now() < deadline) {
            REQUIRE_FALSE(device->isClosed());
            for(const auto& entry : queues) counts.at(entry.first) += entry.second->tryGetAll<dai::ImgFrame>().size();
            std::this_thread::sleep_for(10ms);
        }
        for(const auto& entry : counts) {
            INFO("Camera socket " << static_cast<int>(entry.first) << ": " << entry.second << " frames");
            // More than the queue's two buffered frames must arrive after the
            // operation. Checking only device liveness misses a stopped sensor.
            REQUIRE(entry.second >= static_cast<std::size_t>(duration.count() * 10));
        }
        REQUIRE(pipeline.isRunning());
    }
};
}  // namespace

TEST_CASE("CBA EEPROM reads before startup preserve all camera streams", "[cba_eeprom]") {
    const auto order = GENERATE(0, 1, 2, 3);
    const std::array<std::vector<Socket>, 4> readOrders = {{{Socket::CAM_B}, {Socket::CAM_C}, {Socket::CAM_B, Socket::CAM_C}, {Socket::CAM_C, Socket::CAM_B}}};
    CAPTURE(order);
    SharedCbaFixture fixture;
    // Each direction and order starts with a fresh device. Missing or empty
    // EEPROMs must not prevent subsequent camera startup.
    for(const auto socket : readOrders.at(order)) {
        fixture.readAllowingUnavailableCalibration(socket);
    }
    fixture.start(fixture.sockets);
    fixture.requireStreaming(5s);
}

TEST_CASE("Live CBA access rejects unavailable EEPROMs or access on a streaming bus", "[cba_eeprom]") {
    const auto socket = GENERATE(Socket::CAM_B, Socket::CAM_C);
    SharedCbaFixture fixture;
    INFO("EEPROM socket " << static_cast<int>(socket));
    const auto eepromAvailable = fixture.device->isCBAEepromAvailable(socket);
    fixture.start(fixture.sockets);
    fixture.requireStreaming(3s);
    REQUIRE_THROWS_MATCHES(
        fixture.device->readCBACalibration2(socket),
        dai::EepromError,
        Catch::Matchers::MessageMatches(Catch::Matchers::ContainsSubstring(eepromAvailable ? "EEPROM_CAMERA_RUNNING" : "EEPROM_NOT_AVAILABLE")));
    fixture.requireStreaming(5s);
}

TEST_CASE("Independent CAM_A EEPROM access leaves B/C streams running", "[cba_eeprom]") {
    SharedCbaFixture fixture;
    if(!fixture.hasCamera(Socket::CAM_A)) SKIP("Requires a camera on CAM_A");
    fixture.start({Socket::CAM_B, Socket::CAM_C});
    fixture.requireStreaming(3s);
    fixture.readAllowingUnavailableCalibration(Socket::CAM_A);
    fixture.requireStreaming(5s);
}

TEST_CASE("CBA access is allowed when its shared neighbour is outside the running pipeline", "[cba_eeprom]") {
    const auto socket = GENERATE(Socket::CAM_B, Socket::CAM_C);
    SharedCbaFixture fixture;
    if(!fixture.hasCamera(Socket::CAM_A)) SKIP("Requires a camera on CAM_A");
    fixture.start({Socket::CAM_A});
    fixture.requireStreaming(3s);
    fixture.readAllowingUnavailableCalibration(socket);
    fixture.requireStreaming(5s);
}
