// Every selectable sensor config of every connected camera streams at its minimum, default and
// maximum FPS, at full resolution to a consumer on the device. Each run checks:
// - the rate at which frames reach the consumer
// - on RVC4, from each frame's metadata: the sensor runs the config (frames report its index as
//   their sensor mode), the sensor's frame rate from device timestamps and sequence numbers, and
//   no frames lost between the sensor and the consumer

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "XLink/XLinkPublicDefines.h"
#include "camera_test_utils.hpp"
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Script.hpp"

namespace {

// Minimum FPS is incorrectly advertised, so this test sets the minimum to 8 FPS.
constexpr float MINIMUM_FPS_RVC4 = 8.0f;  // TODO(Jakob) - remove this when fixed on device side
// Maximum FPS on RVC2 is 120, because of 3A limitations
constexpr float MAXIMUM_FPS_RVC2 = 120.0f;
// FPS the device picks when none is requested, tested in every config that supports it
constexpr float DEFAULT_FPS = 30.0f;

// The sensor's frame period sets the device timestamps, so its rate can be checked tightly
constexpr double SENSOR_FPS_TOLERANCE = 0.02;
// Mean consumer rate below the sensor's means frames got lost on the way. Report boundaries add
// up to about 0.2%, one lost frame in the three reports at 30 FPS is about 1%.
constexpr double LOST_FRAMES_TOLERANCE = 0.01;
// Arrival rate at the full resolution consumer, also includes ISP and queueing jitter
constexpr double PIPELINE_FPS_TOLERANCE = 0.15;
// Reports of about one second each, after one discarded while the stream starts
constexpr uint32_t PIPELINE_REPORTS = 3;
// Sends each frame's sequence number, device timestamp and sensor mode to the host. A second
// camera output for this isn't possible in every config, and full frames would saturate the link.
constexpr const char* FRAME_METADATA_SCRIPT = R"(
import depthai as dai
while True:
    frame = node.inputs["frames"].get()
    metadata = dai.Buffer()
    metadata.setSequenceNum(frame.getSequenceNum())
    metadata.setTimestampDevice(frame.getTimestampDevice())
    metadata.setData(list(frame.getSensorMode().to_bytes(4, "little", signed=True)))
    node.outputs["metadata"].send(metadata)
)";
// Covers device connect, sensor power-up and the first report
constexpr auto START_TIMEOUT = std::chrono::seconds(15);

struct FpsCase {
    dai::CameraBoardSocket socket = dai::CameraBoardSocket::AUTO;
    std::string sensorName;
    dai::CameraSensorConfig config;
    // Index in CameraFeatures::configs, RVC4 frames report it as their sensor mode
    int configIndex = -1;
    float fps = DEFAULT_FPS;
};

std::string describe(const FpsCase& testCase) {
    std::ostringstream out;
    out << dai::toString(testCase.socket) << " " << testCase.sensorName << " " << testCase.config.width << "x" << testCase.config.height
        << (testCase.config.hdr ? " HDR" : "") << (testCase.config.hfr ? " HFR" : "") << " @ " << testCase.fps << " fps";
    return out.str();
}

std::vector<float> fpsToTest(const dai::CameraSensorConfig& config, dai::Platform platform) {
    const float minimum = platform == dai::Platform::RVC4 ? std::max(MINIMUM_FPS_RVC4, config.minFps) : config.minFps;
    const float maximum = platform == dai::Platform::RVC2 ? std::min(MAXIMUM_FPS_RVC2, config.maxFps) : config.maxFps;
    if(maximum < minimum) {
        // A config that only runs above the platform cap
        return {};
    }
    std::vector<float> fps{maximum};
    if(minimum < DEFAULT_FPS && DEFAULT_FPS < maximum) {
        fps.push_back(DEFAULT_FPS);
    }
    if(minimum < maximum) {
        fps.push_back(minimum);
    }
    return fps;
}

std::vector<FpsCase> allFpsCases() {
    const auto& snapshot = camera_test::deviceSnapshot();
    std::vector<FpsCase> cases;
    for(const auto& camera : snapshot.cameras) {
        for(size_t i = 0; i < camera.configs.size(); ++i) {
            if(!camera_test::isSelectable(camera.configs, i)) {
                continue;
            }
            for(const float fps : fpsToTest(camera.configs[i], snapshot.platform)) {
                cases.push_back({camera.socket, camera.sensorName, camera.configs[i], static_cast<int>(i), fps});
            }
        }
    }
    return cases;
}

}  // namespace

TEST_CASE("Camera sensor configs configurations") {
    const auto& snapshot = camera_test::deviceSnapshot();
    const auto cases = allFpsCases();
    if(cases.empty()) {
        SKIP("No camera sensor configs reported");
    }
    // One run per config and FPS, so a failure doesn't hide the results of the others
    const auto testCase = GENERATE_COPY(from_range(cases));
    INFO(describe(testCase));
    std::cout << "Testing " << describe(testCase) << " on device " << snapshot.info.getDeviceId() << "\n" << std::flush;

    dai::Pipeline pipeline{camera_test::reconnect()};
    auto camera = camera_test::buildCamera(pipeline, testCase.socket, testCase.config, testCase.fps);
    auto* output = camera->requestOutput(camera_test::resolution(testCase.config), std::nullopt, dai::ImgResizeMode::CROP, testCase.fps);
    REQUIRE(output != nullptr);
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::max(1L, std::lround(testCase.fps))));
    output->link(benchmarkIn->input);
    auto reports = benchmarkIn->report.createOutputQueue();
    const bool isRvc4 = snapshot.platform == dai::Platform::RVC4;
    std::shared_ptr<dai::MessageQueue> metadataQueue;
    if(isRvc4) {
        auto script = pipeline.create<dai::node::Script>();
        script->setScript(FRAME_METADATA_SCRIPT);
        // Never holds frames back from the camera. Frames the script misses at high FPS only
        // shorten the timing sample, sequence numbers still count them.
        script->inputs["frames"].setBlocking(false);
        script->inputs["frames"].setMaxSize(4);
        output->link(script->inputs["frames"]);
        metadataQueue = script->outputs["metadata"].createOutputQueue(1024, false);
    }
    pipeline.start();

    // Both queues are drained together, so metadata isn't dropped while waiting for reports
    std::vector<std::shared_ptr<dai::Buffer>> metadata;
    std::vector<float> pipelineFps;
    bool firstReport = true;
    const auto deadline = std::chrono::steady_clock::now() + START_TIMEOUT + std::chrono::seconds(PIPELINE_REPORTS * 2);
    while(pipelineFps.size() < PIPELINE_REPORTS && std::chrono::steady_clock::now() < deadline) {
        if(metadataQueue) {
            for(auto& message : metadataQueue->tryGetAll<dai::Buffer>()) {
                metadata.push_back(std::move(message));
            }
        }
        if(auto report = reports->tryGet<dai::BenchmarkReport>()) {
            if(!firstReport) {
                pipelineFps.push_back(report->fps);
            }
            firstReport = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(pipelineFps.size() == PIPELINE_REPORTS);

    std::cout << "    consumer";
    for(const float fps : pipelineFps) {
        std::cout << " " << fps;
    }
    std::cout << " fps";
    double sensorFps = 0.0;
    if(isRvc4) {
        REQUIRE(metadata.size() > 2);
        for(const auto& message : metadata) {
            const auto& data = message->getData();
            REQUIRE(data.size() == 4);
            const int32_t sensorMode = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8)
                                                            | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
            // HDR and non-HDR configs share a resolution, only the index tells them apart
            REQUIRE(sensorMode == testCase.configIndex);
        }

        // The first second, while the stream starts, is left out of timing
        const auto timingStart = metadata.front()->getTimestampDevice() + std::chrono::seconds(1);
        const auto first = std::find_if(metadata.begin(), metadata.end(), [&](const auto& message) { return message->getTimestampDevice() >= timingStart; });
        REQUIRE(std::distance(first, metadata.end()) > 2);
        // A repeated or out of order frame means a looping or stalled buffer
        for(auto it = std::next(first); it != metadata.end(); ++it) {
            REQUIRE((*it)->getSequenceNum() > (*std::prev(it))->getSequenceNum());
        }
        const auto& last = metadata.back();
        const auto frames = last->getSequenceNum() - (*first)->getSequenceNum();
        sensorFps = static_cast<double>(frames) / std::chrono::duration<double>(last->getTimestampDevice() - (*first)->getTimestampDevice()).count();
        std::cout << ", sensor " << sensorFps << " fps";
    }
    std::cout << "\n" << std::flush;

    if(pipeline.isHolisticReplayEnabled()) {
        return;
    }
    for(const float fps : pipelineFps) {
        CHECK(fps == Catch::Approx(testCase.fps).epsilon(PIPELINE_FPS_TOLERANCE));
    }
    if(isRvc4) {
        CHECK(sensorFps == Catch::Approx(testCase.fps).epsilon(SENSOR_FPS_TOLERANCE));
        double meanPipelineFps = 0.0;
        for(const float fps : pipelineFps) {
            meanPipelineFps += fps / static_cast<double>(pipelineFps.size());
        }
        CHECK(meanPipelineFps >= sensorFps * (1.0 - LOST_FRAMES_TOLERANCE));
    }
}

TEST_CASE("Camera pool sizes") {
    auto firstDevice = dai::Device::getFirstAvailableDevice();
    auto isRvc4 = std::get<1>(firstDevice).platform == X_LINK_RVC4;
    for(const int overrideQueueSize : (isRvc4 ? std::vector<int>{-1, 2, 17, 3, 50, 4, 5} : std::vector<int>{-1, 2, 17, 3, 4, 5})) {
        std::cout << "Testing num frames = " << overrideQueueSize << "\n" << std::flush;
        dai::Pipeline pipeline;
        std::map<dai::CameraBoardSocket, std::vector<std::tuple<int, int, float>>> streamsRvc4{
            // Has to be (for now):
            // - without FpsRegulator (different fps per same sensor)(different fps on different sensors also doesn't work right now)
            // - without ManipResizer so size should be supported by ISP directly
            {dai::CameraBoardSocket::CAM_A, {{640, 480, 30.0f}, {1920, 1440, 30.0f}}},
            {dai::CameraBoardSocket::CAM_B, {{640, 400, 30.0f}, {1280, 800, 30.0f}}},
            {dai::CameraBoardSocket::CAM_C, {{640, 400, 30.0f}, {1280, 800, 30.0f}}},
        };
        // RVC2 is more RAM bound so use smaller sizes for the test
        std::map<dai::CameraBoardSocket, std::vector<std::tuple<int, int, float>>> streamsRvc2{
            // Has to be (for now):
            // - not a size supported directly by ISP as then isp is passed trough and the isp pool size value is used not the outputs pool size
            {dai::CameraBoardSocket::CAM_A, {{300, 300, 30.0f}}},
            {dai::CameraBoardSocket::CAM_B, {{300, 300, 30.0f}, {200, 200, 30.0f}}},
            {dai::CameraBoardSocket::CAM_C, {{200, 200, 30.0f}}},
        };
        auto streams = isRvc4 ? streamsRvc4 : streamsRvc2;
        std::vector<std::shared_ptr<dai::MessageQueue>> outQueues;
        std::vector<int> outQueuesCounter;
        std::vector<std::shared_ptr<dai::node::Camera>> cameras;
        auto script = pipeline.create<dai::node::Script>();
        // If startup AutoCalibration is enabled globally, default pool count is increased by 3.
        const char* autoCalibrationEnv = std::getenv("DEPTHAI_AUTOCALIBRATION");
        const bool autoCalibrationOnStartup =
            autoCalibrationEnv != nullptr && (std::string(autoCalibrationEnv) == "ON_START" || std::string(autoCalibrationEnv) == "CONTINUOUS");
        const int defaultQueueSizeBase = isRvc4 ? 7 : 3;
        const int defaultQueueSize = defaultQueueSizeBase + (autoCalibrationOnStartup ? 3 : 0);
        int queueSize = overrideQueueSize == -1 ? defaultQueueSize : overrideQueueSize;
        for(const auto& [socket, resolutions] : streams) {
            auto camera = pipeline.create<dai::node::Camera>()->build(socket);
            camera->properties.maxSizePoolOutputs = 1 * 1024 * 1024 * 1024;  // 1G size limit to only test num frames limitation
            if(overrideQueueSize != -1) {
                camera->properties.numFramesPoolOutputs = overrideQueueSize;
            }
            for(const auto& resolution : resolutions) {
                std::string theKey = std::to_string(outQueues.size());
                std::string inputName = "in" + theKey;
                std::string outputName = "out" + theKey;
                camera->requestOutput({std::get<0>(resolution), std::get<1>(resolution)}, std::nullopt, dai::ImgResizeMode::CROP, std::get<2>(resolution))
                    ->link(script->inputs[inputName]);
                script->inputs[inputName].setBlocking(false);
                script->inputs[inputName].setMaxSize(1000);
                outQueues.push_back(script->outputs[outputName].createOutputQueue());
                outQueuesCounter.push_back(0);
            }
            cameras.push_back(camera);
        }
        int timeToBlock = 20;
        std::string scriptContent = isRvc4 ? R"(
            from depthai import BenchmarkReport)"
                                           : "";
        scriptContent += R"(
            import time

            all_frames=[]
            max_id = )" + std::to_string(outQueues.size() - 1)
                         + R"(
            start_time = time.time()
            while time.time() - start_time < )"
                         + std::to_string(timeToBlock) + R"(:
                for idx in range(max_id + 1):
                    the_key = str(idx)
                    frame = node.inputs["in" + the_key].tryGet()
                    if frame is not None:
                        all_frames.append(frame)
                        out = BenchmarkReport()
                        node.outputs["out" + the_key].send(out)
            all_frames = []
            while True:
                for idx in range(max_id + 1):
                    the_key = str(idx)
                    frame = node.inputs["in" + the_key].tryGet()
                    if frame is not None:
                        out = BenchmarkReport()
                        node.outputs["out" + the_key].send(out)
        )";
        script->setScript(scriptContent);
        pipeline.start();
        auto startTime = std::chrono::steady_clock::now();
        // Keep frames in script node and check Camera node stops sending frames after buffer limit is hit
        while(std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() < timeToBlock - 5) {
            for(int idx = 0; idx < outQueues.size(); ++idx) {
                auto frame = outQueues[idx]->tryGet();
                if(frame) {
                    ++outQueuesCounter[idx];
                }
            }
        }
        std::cout << "Got the first part frames\n" << std::flush;
        for(const auto& count : outQueuesCounter) {
            REQUIRE(count == queueSize);
        }
        // Check stream still works after script node unblocks the Camera node
        while(std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() < timeToBlock + 30) {
            for(int idx = 0; idx < outQueues.size(); ++idx) {
                auto frame = outQueues[idx]->tryGet();
                if(frame) {
                    ++outQueuesCounter[idx];
                }
            }
        }
        for(const auto& count : outQueuesCounter) {
            REQUIRE(count > queueSize + 200);
        }
    }
}
