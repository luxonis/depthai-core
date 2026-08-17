#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "XLink/XLinkPublicDefines.h"
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

TEST_CASE("Camera pool sizes") {
    auto firstDevice = dai::Device::getFirstAvailableDevice();
    auto isRvc4 = std::get<1>(firstDevice).platform == X_LINK_RVC4;
    for(const int overrideQueueSize : (isRvc4 ? std::vector<int>{50, 4, 5} : std::vector<int>{3, 4, 5})) {
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
