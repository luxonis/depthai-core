#include "depthai/pipeline/node/Camera.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <chrono>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

TEST_CASE("Test raw camera output") {
    // Create pipeline
    dai::Pipeline p;
    auto fpsToRun = GENERATE(10.5, 20.0, 30.0);
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::AUTO, std::nullopt, fpsToRun);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    SECTION("RAW") {
        camera->raw.link(benchmarkIn->input);
    }
    SECTION("NORMAL") {
        auto* output = camera->requestFullResolutionOutput();
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
    }
    benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fpsToRun) * 2));
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();

    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(fpsToRun).margin(fpsToRun * 0.1));
    }
}

TEST_CASE("Test camera with multiple outputs with different FPS") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    std::map<float, std::shared_ptr<dai::MessageQueue>> fpsToReportQueue;
    bool haveRaw = false;
    std::shared_ptr<dai::MessageQueue> rawQueue;
    // for(auto fps : {10.5f, 30.0f}) { // Only one additional output in case raw is enabled is supported on RVC4
    for(auto fps : {30.0f}) {
        auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
        benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fps) * 2));
        auto* output = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps);
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
        fpsToReportQueue[fps] = benchmarkIn->report.createOutputQueue();
    }

    // TODO(Morato) - add back when fixed on the newest OSes
    // SECTION("With RAW") {
    //     auto benchmarkInRaw = p.create<dai::node::BenchmarkIn>();
    //     camera->raw.link(benchmarkInRaw->input);
    //     rawQueue = benchmarkInRaw->report.createOutputQueue();
    //     haveRaw = true;
    // }

    SECTION("No RAW") {
        haveRaw = false;
    }

    p.start();
    for(int i = 0; i < 3; i++) {
        for(auto& [fps, queue] : fpsToReportQueue) {
            auto benchmarkReport = queue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(fps).margin(fps * 0.1));
        }
        if(haveRaw) {
            auto benchmarkReport = rawQueue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(30.0).margin(30.0 * 0.1));
        }
    }
}

#ifndef _WIN32  // TODO(Jakob) - fix this test on Windows and bring it back on RVC4 when multi FPS is supported
// TEST_CASE("Test setting the center camera to a different FPS compared to left and right") {
//     // Create pipeline
//     dai::Pipeline p;
//     std::map<dai::CameraBoardSocket, float> socketToFps = {
//         {dai::CameraBoardSocket::CAM_A, 10.0},
//         {dai::CameraBoardSocket::CAM_B, 20.0},
//         {dai::CameraBoardSocket::CAM_C, 20.0},
//     };
//     std::map<dai::CameraBoardSocket, std::shared_ptr<dai::MessageQueue>> messageQueues;
//     for(auto& [socket, fps] : socketToFps) {
//         auto camera = p.create<dai::node::Camera>()->build(socket, std::nullopt, fps);
//         auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
//         benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fps) * 2));
//         auto* output = camera->requestFullResolutionOutput();
//         REQUIRE(output != nullptr);
//         output->link(benchmarkIn->input);
//         messageQueues[socket] = benchmarkIn->report.createOutputQueue();
//     }

//     p.start();
//     for(int i = 0; i < 3; i++) {
//         for(auto& [socket, queue] : messageQueues) {
//             auto benchmarkReport = queue->get<dai::BenchmarkReport>();
//             // Allow +-10% difference
//             REQUIRE(benchmarkReport->fps == Catch::Approx(socketToFps[socket]).margin(socketToFps[socket] * 0.1));
//         }
//     }
// }

TEST_CASE("Multiple outputs") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto* camOut1 = camera->requestOutput({500, 300}, dai::ImgFrame::Type::NV12);
    auto* camOut2 = camera->requestOutput({700, 500}, dai::ImgFrame::Type::RGB888i);
    auto* camOut3 = camera->requestOutput({900, 700}, dai::ImgFrame::Type::GRAY8);

    auto camOut1Queue = camOut1->createOutputQueue();
    auto camOut2Queue = camOut2->createOutputQueue();
    auto camOut3Queue = camOut3->createOutputQueue();

    p.start();
    for(int i = 0; i < 5; i++) {
        auto frame1 = camOut1Queue->get<dai::ImgFrame>();
        REQUIRE(frame1->getWidth() == 500);
        REQUIRE(frame1->getHeight() == 300);
        REQUIRE(frame1->getType() == dai::ImgFrame::Type::NV12);

        auto frame2 = camOut2Queue->get<dai::ImgFrame>();
        REQUIRE(frame2->getWidth() == 700);
        REQUIRE(frame2->getHeight() == 500);
        REQUIRE(frame2->getType() == dai::ImgFrame::Type::RGB888i);

        auto frame3 = camOut3Queue->get<dai::ImgFrame>();
        REQUIRE(frame3->getWidth() == 900);
        REQUIRE(frame3->getHeight() == 700);
        REQUIRE(frame3->getType() == dai::ImgFrame::Type::GRAY8);
    }
}
#endif

TEST_CASE("Test how default FPS is generated for a specific output") {
    constexpr float FPS_TO_SET = 20.0;
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    auto* output1 = camera->requestOutput(std::make_pair(640, 400), std::nullopt);
    REQUIRE(output1 != nullptr);
    auto* output2 = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS_TO_SET);
    REQUIRE(output2 != nullptr);
    output2->createOutputQueue();  // "Sink it"
    output1->link(benchmarkIn->input);
    benchmarkIn->sendReportEveryNMessages(FPS_TO_SET * 2);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();
    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(FPS_TO_SET).margin(FPS_TO_SET * 0.1));
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
        // Default size of the pool for RVC2 is 3 and RVC4 is 7
        int queueSize = overrideQueueSize == -1 ? (isRvc4 ? 7 : 3) : overrideQueueSize;
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

inline bool isRvc2(std::shared_ptr<dai::Device> d) {
    try {
        return d->getDeviceName().find("OAK-") != std::string::npos;
    } catch(...) {
        return false;
    }
}

std::pair<int, int> getResolutionSizeFromConfigName(const std::string& name) {
    static const std::unordered_map<std::string, std::pair<int, int>> resolutionSizeMap = {
        {"LCM48", {4056, 3040}},
        {"IMX378", {4056, 3040}},
        {"OV9282", {1280, 800}},
        {"IMX214", {4208, 3120}},
        {"IMX477", {4056, 3040}},
        {"OV9782", {1280, 800}},
        {"AR0234", {1920, 1200}},
        {"IMX582", {4000, 3000}},
        {"IMX462", {1920, 1080}},
        {"IMX577", {4056, 3040}},
        {"IMX586", {4056, 3040}},
        {"S5K33D", {1280, 800}},
        {"IMX334", {3840, 2160}},
        {"IMX412", {4056, 3040}},
        {"OV2735", {1920, 1080}},
        {"SC8238", {3840, 2160}},
    };

    auto it = resolutionSizeMap.find(name);
    if(it != resolutionSizeMap.end()) {
        return it->second;
    }

    std::cerr << "Warning: Unknown camera config name '" << name << "', defaulting to 1920x1080 (1080P)" << std::endl;
    return {1920, 1080};
}

TEST_CASE("Camera: Test isp output - Correct resolution - full resolution ") {
    auto device = std::make_shared<dai::Device>();
    if(!isRvc2(device)) {
        WARN("Skipping: not running on RVC2");
        return;
    }
    dai::Pipeline p(device);

    auto cameraFeatures = device->getConnectedCameraFeatures();

    std::vector<std::shared_ptr<dai::MessageQueue>> outputQueues;
    std::vector<std::shared_ptr<dai::MessageQueue>> outputIspQueues;

    for(const auto& feature : cameraFeatures) {
        auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
        int width = resolutionIspExpected.first - 100;
        int height = resolutionIspExpected.second - 100;

        auto camera = p.create<dai::node::Camera>()->build(feature.socket);

        outputQueues.push_back(camera->requestOutput({width, height})->createOutputQueue());
        outputIspQueues.push_back(camera->requestIspOutput()->createOutputQueue());
    }

    p.start();

    int i = 0;
    for(const auto& feature : cameraFeatures) {
        auto outputQueue = outputQueues[i];
        auto outputIspQueue = outputIspQueues[i];

        auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
        int width = resolutionIspExpected.first - 100;
        int height = resolutionIspExpected.second - 100;

        auto output = outputQueue->get<dai::ImgFrame>();
        auto outputIsp = outputIspQueue->get<dai::ImgFrame>();

        REQUIRE(output->getWidth() == width);
        REQUIRE(output->getHeight() == height);

        REQUIRE(outputIsp->getWidth() == resolutionIspExpected.first);
        REQUIRE(outputIsp->getHeight() == resolutionIspExpected.second);

        i++;
    }
}

TEST_CASE("Camera: Test isp output - Correct resolution - half resolution") {
    auto device = std::make_shared<dai::Device>();
    if(!isRvc2(device)) {
        WARN("Skipping: not running on RVC2");
        return;
    }
    dai::Pipeline p(device);

    auto cameraFeatures = device->getConnectedCameraFeatures();

    std::vector<std::shared_ptr<dai::MessageQueue>> outputQueues;
    std::vector<std::shared_ptr<dai::MessageQueue>> outputIspQueues;

    for(const auto& feature : cameraFeatures) {
        if(feature.socket == dai::CameraBoardSocket::CAM_A) {
            continue;  // skip CAM_A if desired
        }
        auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
        int width = resolutionIspExpected.first / 2 - 100;
        int height = resolutionIspExpected.second / 2 - 100;

        auto camera = p.create<dai::node::Camera>()->build(feature.socket);

        outputQueues.push_back(camera->requestOutput({width, height})->createOutputQueue());
        outputIspQueues.push_back(camera->requestIspOutput()->createOutputQueue());
    }

    p.start();

    int i = 0;
    for(const auto& feature : cameraFeatures) {
        if(feature.socket == dai::CameraBoardSocket::CAM_A) {
            continue;  // skip CAM_A if desired
        }

        auto outputQueue = outputQueues[i];
        auto outputIspQueue = outputIspQueues[i];

        auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
        int width = resolutionIspExpected.first / 2 - 100;
        int height = resolutionIspExpected.second / 2 - 100;

        auto output = outputQueue->get<dai::ImgFrame>();
        auto outputIsp = outputIspQueue->get<dai::ImgFrame>();

        REQUIRE(output->getWidth() == width);
        REQUIRE(output->getHeight() == height);

        REQUIRE(outputIsp->getWidth() == resolutionIspExpected.first / 2);
        REQUIRE(outputIsp->getHeight() == resolutionIspExpected.second / 2);

        i++;
    }
}

TEST_CASE("Camera: Test isp output - Correct fps2") {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline p(device);

    auto cameraFeatures = device->getConnectedCameraFeatures();

    if(cameraFeatures.size() == 0) {
        return;
    }

    const auto& feature = cameraFeatures[0];
    auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
    auto camera = p.create<dai::node::Camera>()->build(feature.socket);

    int width = 200;
    int height = 200;

    auto outputQueue = camera->requestOutput({width, height}, std::nullopt, dai::ImgResizeMode::CROP, 30., std::nullopt)->createOutputQueue();
    auto outputIspQueue = camera->requestIspOutput(10.)->createOutputQueue();

    p.start();
    int outCounter = 0;
    int outIspCounter = 0;

    // wait for the first img
    outputQueue->get<dai::ImgFrame>();
    outputIspQueue->get<dai::ImgFrame>();

    using clock = std::chrono::steady_clock;
    auto startTime = clock::now();
    auto testDuration = std::chrono::seconds(10);

    while(true) {
        auto out = outputQueue->tryGet<dai::ImgFrame>();
        auto outIsp = outputIspQueue->tryGet<dai::ImgFrame>();
        if(out) {
            outCounter += 1;
        }
        if(outIsp) {
            outIspCounter += 1;
        }

        auto now = clock::now();
        if(now - startTime > testDuration) {
            break;
        }
    }
    std::cout << testDuration.count() << "\n";
    double fps = static_cast<double>(outCounter) / testDuration.count();
    double fpsIsp = static_cast<double>(outIspCounter) / testDuration.count();
    REQUIRE(fps > 26.0);
    REQUIRE(fps < 34.0);
    REQUIRE(fpsIsp > 6.0);
    REQUIRE(fpsIsp < 14.0);
}

TEST_CASE("Camera: Test isp output - Correct fps") {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline p(device);

    auto cameraFeatures = device->getConnectedCameraFeatures();

    if(cameraFeatures.size() == 0) {
        return;
    }

    const auto& feature = cameraFeatures[0];
    auto resolutionIspExpected = getResolutionSizeFromConfigName(feature.sensorName);
    auto camera = p.create<dai::node::Camera>()->build(feature.socket);

    int width = 200;
    int height = 200;

    auto outputQueue = camera->requestOutput({width, height}, std::nullopt, dai::ImgResizeMode::CROP, 10., std::nullopt)->createOutputQueue();
    auto outputIspQueue = camera->requestIspOutput(30.)->createOutputQueue();

    p.start();
    int outCounter = 0;
    int outIspCounter = 0;

    // wait for the first img
    outputQueue->get<dai::ImgFrame>();
    outputIspQueue->get<dai::ImgFrame>();

    using clock = std::chrono::steady_clock;
    auto startTime = clock::now();
    auto testDuration = std::chrono::seconds(10);

    while(true) {
        auto out = outputQueue->tryGet<dai::ImgFrame>();
        auto outIsp = outputIspQueue->tryGet<dai::ImgFrame>();
        if(out) {
            outCounter += 1;
        }
        if(outIsp) {
            outIspCounter += 1;
        }

        auto now = clock::now();
        if(now - startTime > testDuration) {
            break;
        }
    }
    double fps = static_cast<double>(outCounter) / testDuration.count();
    double fpsIsp = static_cast<double>(outIspCounter) / testDuration.count();
    REQUIRE(fps > 6.0);
    REQUIRE(fps < 14.0);
    REQUIRE(fpsIsp > 6.0);
    REQUIRE(fpsIsp < 14.0);
}
