#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <thread>
#include <utility>

#include "depthai/depthai.hpp"

namespace {

double setting(const char* name, double fallback) {
    const auto* value = std::getenv(name);
    return value ? std::stod(value) : fallback;
}

}  // namespace

TEST_CASE("Full-resolution IMX378 transfer sustains the required payload rate", "[camera-hil][throughput]") {
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;
    const auto fps = setting("DEPTHAI_TRANSFER_FPS", 30);
    const auto seconds = setting("DEPTHAI_TRANSFER_SECONDS", 60);
    const auto minimumGbps = setting("DEPTHAI_TRANSFER_MIN_GBPS", 3);
    REQUIRE(std::isfinite(fps));
    REQUIRE(std::isfinite(seconds));
    REQUIRE(std::isfinite(minimumGbps));
    REQUIRE(fps > 0);
    REQUIRE(fps <= 30);
    REQUIRE(seconds >= 1);
    REQUIRE(minimumGbps >= 0);
    const auto* output = std::getenv("DEPTHAI_TRANSFER_OUTPUT");
    std::ofstream samples;
    if(output) {
        samples.open(std::string(output) + ".csv");
        REQUIRE(samples.is_open());
        samples << std::setprecision(17) << "host_seconds,sequence,device_seconds,width,height,bytes,sample_hash\n";
    }

    auto device = std::make_shared<dai::Device>();
    const auto protocol = device->getDeviceInfo().protocol;
    const bool usb = protocol == X_LINK_USB_EP;
    REQUIRE((usb || protocol == X_LINK_TCP_IP));
    if(usb) {
        const auto interfaces = device->getConnectionInterfaces();
        REQUIRE(std::find(interfaces.begin(), interfaces.end(), dai::ConnectionInterface::USB) != interfaces.end());
        const auto speed = device->getUsbSpeed();
        REQUIRE(speed >= dai::UsbSpeed::HIGH);
        std::cout << "usb_speed=" << speed << std::endl;
        if(minimumGbps >= 3) REQUIRE(speed >= dai::UsbSpeed::SUPER);
    }
    if(minimumGbps >= 3) {
        REQUIRE(usb);
        REQUIRE(seconds >= 60);
    }
    const auto features = device->getConnectedCameraFeatures();
    REQUIRE(std::any_of(
        features.begin(), features.end(), [](const auto& camera) { return camera.socket == dai::CameraBoardSocket::CAM_A && camera.sensorName == "IMX378"; }));
    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::make_pair(4032, 3024), 30);
    auto queue = camera->requestFullResolutionOutput(dai::ImgFrame::Type::NV12, fps)->createOutputQueue(8, false);
    pipeline.start();
    const auto start = Clock::now();
    auto lastFrame = start;
    auto measurementStart = start;
    std::uint64_t bytes = 0, frames = 0, gaps = 0;
    std::int64_t previousSequence = -1;
    auto previousTimestamp = std::chrono::steady_clock::time_point{};
    std::set<std::uint64_t> hashes;
    bool measuring = false, complete = false;
    while(pipeline.isRunning()) {
        const auto now = Clock::now();
        if(now - lastFrame >= 10s) FAIL("No camera frame arrived for ten seconds");
        auto frame = queue->tryGet<dai::ImgFrame>();
        if(!frame) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        lastFrame = now;
        const auto data = std::as_const(*frame).getData();
        REQUIRE(frame->getWidth() == 4032);
        REQUIRE(frame->getHeight() == 3024);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(data.size() == std::size_t(4032) * 3024 * 3 / 2);
        const auto sequence = frame->getSequenceNum();
        REQUIRE(sequence > previousSequence);
        REQUIRE(frame->getTimestampDevice() > previousTimestamp);
        previousTimestamp = frame->getTimestampDevice();
        if(now - start < 5s) {
            previousSequence = sequence;
            continue;
        }
        if(!measuring) {
            measuring = true;
            measurementStart = now;
            previousSequence = sequence;
            if(output) {
                std::ofstream image(std::string(output) + ".nv12", std::ios::binary);
                image.write(reinterpret_cast<const char*>(data.data()), data.size());
                REQUIRE(image.good());
            }
            std::cout << "measurement_start protocol=" << (usb ? "usb" : "tcp") << " requested_fps=" << fps << std::endl;
            continue;  // The boundary frame was received before the measured interval.
        }
        gaps += sequence - previousSequence - 1;
        previousSequence = sequence;
        bytes += data.size();
        ++frames;
        std::uint64_t hash = 14695981039346656037ULL;
        for(std::size_t offset = 0; offset < data.size(); offset += 4096) hash = (hash ^ data[offset]) * 1099511628211ULL;
        REQUIRE(hashes.insert(hash).second);
        const double elapsed = std::chrono::duration<double>(now - measurementStart).count();
        const double timestamp = std::chrono::duration<double>(previousTimestamp.time_since_epoch()).count();
        if(samples.is_open()) samples << elapsed << ',' << sequence << ',' << timestamp << ",4032,3024," << data.size() << ',' << hash << '\n';
        if(elapsed >= seconds) {
            const double gbps = double(bytes) * 8 / elapsed / 1e9;
            std::cout << "frames=" << frames << " bytes=" << bytes << " seconds=" << elapsed << " sequence_gaps=" << gaps << " payload_gbps=" << gbps
                      << std::endl;
            REQUIRE(hashes.size() > 1);
            REQUIRE(gbps >= minimumGbps);
            complete = true;
            break;
        }
    }
    REQUIRE(complete);
    pipeline.stop();
    pipeline.wait();
}
