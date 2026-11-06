// Camera stack sanity test for multi-port RVC4 devices (OAK 4 FFC).
//
// Connected sensors are enumerated at runtime, so the test runs unchanged on any combination
// of sensors mounted on the FFC ports. A sensor mode is one entry of CameraFeatures::configs;
// on RVC4 the device selects it by resolution plus the HDR flag.
//
// Every mode of every sensor streams on its own, and is frame synced against one other port.
// Running all ports concurrently in every mode isn't tested: which combinations fit is decided
// by the device's ISP allocator, a 48 MP sensor alongside four others doesn't.
//
// Optional environment:
//   DEPTHAI_TESTS_EXPECTED_CAMERAS - number of cameras that must be detected, catches a port
//                                    silently dropping out of camera discovery

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"

namespace {

using camera_test::deviceSnapshot;
using camera_test::isSelectable;
using camera_test::reconnect;

// Same FPS the device picks for a sensor mode when none is requested
constexpr float DEFAULT_FPS = 30.0f;
// Frames each stream has to deliver to count as streaming
constexpr uint32_t FRAMES_TO_RECEIVE = 10;
// Covers device connect, sensor power-up and ISP start
constexpr auto FIRST_FRAME_TIMEOUT = std::chrono::seconds(15);
// Covers link-limited transfer of the largest frames (a 48 MP NV12 frame is ~72 MB)
constexpr auto NEXT_FRAME_TIMEOUT = std::chrono::seconds(5);

// Frame sync limits on device timestamps, same as RVC4 limits in fsync_test.cpp. Cameras with
// the same sensor share readout timing, so only mixed sensors get the looser limit.
constexpr auto FSYNC_SAME_SENSOR_TOLERANCE = std::chrono::microseconds(5);
constexpr auto FSYNC_ALL_PORTS_TOLERANCE = std::chrono::milliseconds(1);
// Groups discarded while sensors lock onto the sync pulse
constexpr uint32_t FSYNC_WARMUP_GROUPS = 5;
constexpr uint32_t FSYNC_GROUPS = 100;
// Frames only carry timestamps to the host, so keep them small
constexpr std::pair<uint32_t, uint32_t> FSYNC_OUTPUT_SIZE{320, 240};

using Timestamp = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

struct SensorMode {
    dai::CameraBoardSocket socket = dai::CameraBoardSocket::AUTO;
    std::string sensorName;
    dai::CameraSensorConfig config;
    float fps = DEFAULT_FPS;
    // Index in CameraFeatures::configs, frames report it as their sensor mode
    int configIndex = -1;
};

void skipUnlessRvc4() {
    if(deviceSnapshot().platform != dai::Platform::RVC4) {
        SKIP("Sensor mode selection under test is RVC4 specific");
    }
}

float streamFps(const dai::CameraSensorConfig& config) {
    if(config.minFps <= DEFAULT_FPS && DEFAULT_FPS <= config.maxFps) {
        return DEFAULT_FPS;
    }
    return config.maxFps;
}

bool supportsFps(const dai::CameraFeatures& camera, float fps) {
    return std::any_of(
        camera.configs.begin(), camera.configs.end(), [&](const dai::CameraSensorConfig& config) { return config.minFps <= fps && fps <= config.maxFps; });
}

std::vector<SensorMode> sensorModes(const dai::CameraFeatures& camera) {
    std::vector<SensorMode> modes;
    for(size_t i = 0; i < camera.configs.size(); ++i) {
        if(isSelectable(camera.configs, i)) {
            modes.push_back({camera.socket, camera.sensorName, camera.configs[i], streamFps(camera.configs[i]), static_cast<int>(i)});
        }
    }
    return modes;
}

std::vector<SensorMode> allSensorModes() {
    std::vector<SensorMode> modes;
    for(const auto& camera : deviceSnapshot().cameras) {
        const auto cameraModes = sensorModes(camera);
        modes.insert(modes.end(), cameraModes.begin(), cameraModes.end());
    }
    return modes;
}

// Another port to compare timestamps against, one with the same sensor if available
std::optional<dai::CameraFeatures> referenceCamera(const SensorMode& mode) {
    std::optional<dai::CameraFeatures> reference;
    for(const auto& camera : deviceSnapshot().cameras) {
        if(camera.socket == mode.socket || !supportsFps(camera, mode.fps)) {
            continue;
        }
        if(camera.sensorName == mode.sensorName) {
            return camera;
        }
        if(!reference) {
            reference = camera;
        }
    }
    return reference;
}

std::string describe(const SensorMode& mode) {
    std::ostringstream out;
    out << dai::toString(mode.socket) << " " << mode.sensorName << " " << mode.config.width << "x" << mode.config.height << " @ " << mode.fps << " fps";
    if(mode.config.hdr) {
        out << " HDR";
    }
    if(mode.config.hfr) {
        out << " HFR";
    }
    return out.str();
}

std::pair<uint32_t, uint32_t> resolution(const SensorMode& mode) {
    return camera_test::resolution(mode.config);
}

std::shared_ptr<dai::node::Camera> createCamera(dai::Pipeline& pipeline, const SensorMode& mode) {
    return camera_test::buildCamera(pipeline, mode.socket, mode.config, mode.fps);
}

std::chrono::nanoseconds halfFramePeriod(float fps) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / fps));
}

std::chrono::nanoseconds spread(const std::vector<Timestamp>& timestamps) {
    const auto [earliest, latest] = std::minmax_element(timestamps.begin(), timestamps.end());
    return std::chrono::duration_cast<std::chrono::nanoseconds>(*latest - *earliest);
}

double toMicroseconds(std::chrono::nanoseconds duration) {
    return std::chrono::duration<double, std::micro>(duration).count();
}

// Middle timestamp of a group, a reference a single late or early port can't move
Timestamp median(std::vector<Timestamp> timestamps) {
    const auto middle = timestamps.begin() + static_cast<std::ptrdiff_t>((timestamps.size() - 1) / 2);
    std::nth_element(timestamps.begin(), middle, timestamps.end());
    return *middle;
}

// One port's timing over all groups, shows which port a timestamp spread comes from
struct PortTiming {
    std::chrono::nanoseconds minOffset = std::chrono::nanoseconds::max();
    std::chrono::nanoseconds maxOffset = std::chrono::nanoseconds::min();
    std::chrono::nanoseconds offsetSum{0};
    std::chrono::microseconds exposureSum{0};
    uint32_t count = 0;

    void add(std::chrono::nanoseconds offset, std::chrono::microseconds exposure) {
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
        offsetSum += offset;
        exposureSum += exposure;
        ++count;
    }
};

}  // namespace

TEST_CASE("Camera ports are detected", "[camera_ffc]") {
    skipUnlessRvc4();
    const auto& cameras = deviceSnapshot().cameras;
    // Every mode the device reports, the tests below cover each selectable one
    for(const auto& camera : cameras) {
        std::cout << camera << "\n";
        for(size_t i = 0; i < camera.configs.size(); ++i) {
            const auto& config = camera.configs[i];
            std::cout << "    " << config.width << "x" << config.height << (config.hdr ? " HDR" : "") << (config.hfr ? " HFR" : "") << ", " << config.minFps
                      << "-" << config.maxFps << " fps" << (isSelectable(camera.configs, i) ? "" : ", can't be selected") << "\n";
        }
    }
    std::cout << std::flush;

    REQUIRE_FALSE(cameras.empty());
    if(const char* expected = std::getenv("DEPTHAI_TESTS_EXPECTED_CAMERAS")) {
        REQUIRE(cameras.size() == std::stoul(expected));
    }
    for(const auto& camera : cameras) {
        INFO(dai::toString(camera.socket) << " " << camera.sensorName);
        CHECK_FALSE(camera.configs.empty());
        for(size_t i = 0; i < camera.configs.size(); ++i) {
            INFO("Mode " << camera.configs[i].width << "x" << camera.configs[i].height << " repeats the resolution and HDR flag of an earlier mode");
            CHECK(isSelectable(camera.configs, i));
        }
    }
}

TEST_CASE("Every sensor mode streams on every camera port", "[camera_ffc]") {
    skipUnlessRvc4();
    const auto allModes = allSensorModes();
    if(allModes.empty()) {
        SKIP("No camera sensor modes reported");
    }
    // One run per mode, so a failing mode doesn't hide results of the others
    const auto mode = GENERATE_COPY(from_range(allModes));
    INFO(describe(mode));
    std::cout << "Streaming " << describe(mode) << "\n" << std::flush;

    dai::Pipeline pipeline{reconnect()};
    // Output at sensor mode resolution, so frame size proves which mode the sensor runs in
    auto frames = createCamera(pipeline, mode)->requestOutput(resolution(mode), std::nullopt, dai::ImgResizeMode::CROP, mode.fps)->createOutputQueue(4, false);
    pipeline.start();

    std::shared_ptr<dai::ImgFrame> previous;
    for(uint32_t i = 0; i < FRAMES_TO_RECEIVE; ++i) {
        CAPTURE(i);
        bool timedOut = false;
        auto frame = frames->get<dai::ImgFrame>(previous ? NEXT_FRAME_TIMEOUT : FIRST_FRAME_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getInstanceNum() == static_cast<unsigned int>(mode.socket));
        REQUIRE(frame->getWidth() == static_cast<unsigned int>(mode.config.width));
        REQUIRE(frame->getHeight() == static_cast<unsigned int>(mode.config.height));
        // Tells HDR from non-HDR at the same resolution
        REQUIRE(frame->getSensorMode() == mode.configIndex);
        REQUIRE(frame->getData().size() > 0);
        if(previous) {
            // A stalled sensor or looping buffer repeats frames instead of producing new ones
            REQUIRE(frame->getSequenceNum() > previous->getSequenceNum());
            REQUIRE(frame->getTimestampDevice() > previous->getTimestampDevice());
        }
        previous = frame;
    }
}

TEST_CASE("Every sensor mode is frame synced", "[camera_ffc][fsync]") {
    skipUnlessRvc4();
    if(deviceSnapshot().cameras.size() < 2) {
        SKIP("Needs at least two cameras");
    }
    const auto mode = GENERATE_COPY(from_range(allSensorModes()));
    INFO(describe(mode));
    // Two streams are the fewest that show sync, and always fit the ISP
    const auto reference = referenceCamera(mode);
    if(!reference) {
        SKIP("No other camera runs at " << mode.fps << " fps");
    }
    const auto referenceName = dai::toString(reference->socket) + " " + reference->sensorName;
    std::cout << "Frame sync " << describe(mode) << " against " << referenceName << "\n" << std::flush;

    dai::Pipeline pipeline{reconnect()};
    auto sync = pipeline.create<dai::node::Sync>();
    // Groups only form within half a frame period, so lost sync shows up as a timeout
    sync->setSyncThreshold(halfFramePeriod(mode.fps));
    auto underTest = createCamera(pipeline, mode);
    // AUTO, the default, puts every camera running at the same FPS on the sync pulse
    underTest->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::AUTO);
    underTest->requestOutput(FSYNC_OUTPUT_SIZE, std::nullopt, dai::ImgResizeMode::CROP, mode.fps)->link(sync->inputs["mode"]);
    // Reference in the default mode for this FPS
    auto other = pipeline.create<dai::node::Camera>()->build(reference->socket, std::nullopt, mode.fps);
    other->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::AUTO);
    other->requestOutput(FSYNC_OUTPUT_SIZE, std::nullopt, dai::ImgResizeMode::CROP, mode.fps)->link(sync->inputs["reference"]);
    auto groups = sync->out.createOutputQueue(4, false);
    pipeline.start();

    PortTiming timing;
    bool sameMode = false;
    for(uint32_t i = 0; i < FSYNC_WARMUP_GROUPS + FSYNC_GROUPS; ++i) {
        CAPTURE(i);
        bool timedOut = false;
        auto group = groups->get<dai::MessageGroup>(i == 0 ? FIRST_FRAME_TIMEOUT : NEXT_FRAME_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(group != nullptr);
        auto frame = group->get<dai::ImgFrame>("mode");
        auto referenceFrame = group->get<dai::ImgFrame>("reference");
        REQUIRE(frame != nullptr);
        REQUIRE(referenceFrame != nullptr);
        if(i == 0) {
            // Output is downscaled, the reported sensor mode shows the sensor runs the mode under test
            REQUIRE(frame->getSensorMode() == mode.configIndex);
            sameMode = reference->sensorName == mode.sensorName && referenceFrame->getSensorMode() == frame->getSensorMode();
        }
        if(i < FSYNC_WARMUP_GROUPS) {
            continue;
        }
        // NONE means the device left that port free-running instead of on the sync pulse
        REQUIRE(frame->getFsync() == dai::ImgFrame::Fsync::INPUT);
        REQUIRE(referenceFrame->getFsync() == dai::ImgFrame::Fsync::INPUT);
        timing.add(std::chrono::duration_cast<std::chrono::nanoseconds>(frame->getTimestampDevice() - referenceFrame->getTimestampDevice()),
                   frame->getExposureTime());
    }

    std::cout << "Timestamp offset from " << referenceName << ", mean [min, max]: " << std::showpos << toMicroseconds(timing.offsetSum / timing.count) << " ["
              << toMicroseconds(timing.minOffset) << ", " << toMicroseconds(timing.maxOffset) << "] us\n"
              << std::noshowpos << std::flush;

    // Readout timing differs between sensors and modes, only the same mode on the same sensor gets the tight limit
    const auto tolerance =
        sameMode ? std::chrono::duration_cast<std::chrono::nanoseconds>(FSYNC_SAME_SENSOR_TOLERANCE) : std::chrono::nanoseconds(FSYNC_ALL_PORTS_TOLERANCE);
    CAPTURE(sameMode);
    CHECK(std::max(-timing.minOffset, timing.maxOffset) <= tolerance);
}

TEST_CASE("All camera ports are frame synced", "[camera_ffc][fsync]") {
    skipUnlessRvc4();
    const auto& cameras = deviceSnapshot().cameras;
    if(cameras.size() < 2) {
        SKIP("Needs at least two cameras");
    }
    // Same FPS values as fsync_test.cpp. RVC4 syncs cameras only at a single shared FPS.
    const auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f);
    CAPTURE(fps);
    for(const auto& camera : cameras) {
        INFO(dai::toString(camera.socket) << " " << camera.sensorName << " has no sensor mode for this FPS");
        REQUIRE(supportsFps(camera, fps));
    }
    std::cout << "Frame sync at " << fps << " fps\n" << std::flush;

    dai::Pipeline pipeline{reconnect()};
    auto sync = pipeline.create<dai::node::Sync>();
    // Groups only form within half a frame period, so lost sync shows up as a timeout
    sync->setSyncThreshold(halfFramePeriod(fps));
    for(const auto& camera : cameras) {
        auto node = pipeline.create<dai::node::Camera>()->build(camera.socket, std::nullopt, fps);
        // AUTO, the default, puts every camera running at the same FPS on the sync pulse
        node->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::AUTO);
        node->requestOutput(FSYNC_OUTPUT_SIZE, std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs[dai::toString(camera.socket)]);
    }
    auto groups = sync->out.createOutputQueue(4, false);
    pipeline.start();

    std::chrono::nanoseconds worstAllPorts{0};
    // Only sensors mounted on more than one port
    std::map<std::string, std::chrono::nanoseconds> worstPerSensor;
    // Same order as cameras
    std::vector<PortTiming> portTiming(cameras.size());
    for(uint32_t i = 0; i < FSYNC_WARMUP_GROUPS + FSYNC_GROUPS; ++i) {
        CAPTURE(i);
        bool timedOut = false;
        auto group = groups->get<dai::MessageGroup>(i == 0 ? FIRST_FRAME_TIMEOUT : NEXT_FRAME_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(group != nullptr);
        if(i < FSYNC_WARMUP_GROUPS) {
            continue;
        }

        std::vector<Timestamp> allPorts;
        std::vector<std::chrono::microseconds> exposures;
        std::map<std::string, std::vector<Timestamp>> perSensor;
        for(const auto& camera : cameras) {
            INFO(dai::toString(camera.socket) << " " << camera.sensorName);
            auto frame = group->get<dai::ImgFrame>(dai::toString(camera.socket));
            REQUIRE(frame != nullptr);
            // NONE means the device left this port free-running instead of on the sync pulse
            REQUIRE(frame->getFsync() == dai::ImgFrame::Fsync::INPUT);
            allPorts.push_back(frame->getTimestampDevice());
            exposures.push_back(frame->getExposureTime());
            perSensor[camera.sensorName].push_back(frame->getTimestampDevice());
        }
        const auto reference = median(allPorts);
        for(size_t port = 0; port < cameras.size(); ++port) {
            portTiming[port].add(std::chrono::duration_cast<std::chrono::nanoseconds>(allPorts[port] - reference), exposures[port]);
        }
        worstAllPorts = std::max(worstAllPorts, spread(allPorts));
        for(const auto& [sensor, timestamps] : perSensor) {
            if(timestamps.size() > 1) {
                worstPerSensor[sensor] = std::max(worstPerSensor[sensor], spread(timestamps));
            }
        }
    }

    // Worst case over all groups, so a failure shows how far off sync is
    std::cout << "Worst timestamp spread at " << fps << " fps: all ports " << toMicroseconds(worstAllPorts) << " us";
    for(const auto& [sensor, worst] : worstPerSensor) {
        std::cout << ", " << sensor << " " << toMicroseconds(worst) << " us";
    }
    std::cout << "\n";
    std::cout << "Port timestamp offset from group median at " << fps << " fps, mean [min, max]:\n";
    for(size_t port = 0; port < cameras.size(); ++port) {
        const auto& timing = portTiming[port];
        std::cout << "    " << dai::toString(cameras[port].socket) << " " << cameras[port].sensorName << ": " << std::showpos
                  << toMicroseconds(timing.offsetSum / timing.count) << " [" << toMicroseconds(timing.minOffset) << ", " << toMicroseconds(timing.maxOffset)
                  << "] us" << std::noshowpos << ", mean exposure " << (timing.exposureSum / timing.count).count() << " us\n";
    }
    std::cout << std::flush;

    CHECK(worstAllPorts <= FSYNC_ALL_PORTS_TOLERANCE);
    for(const auto& [sensor, worst] : worstPerSensor) {
        INFO(sensor << " ports");
        CHECK(worst <= FSYNC_SAME_SENSOR_TOLERANCE);
    }
}
