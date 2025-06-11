// This bug was detected and fixed in October 2024 on oak4-d:

#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"

TEST_CASE("camera_concurrency") {
    constexpr float FPS = 30.0f;
    constexpr int SECONDS_TO_RUN = 50;
    constexpr float MAX_FPS_OFFSET = 0.5;
    constexpr int MAX_SECONDS_OFFSET = 5;
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    std::vector<dai::Node::Output*> cameraOutputs = {
        cam->requestOutput(std::make_pair(640, 352), dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, FPS),
        cam->requestOutput(std::make_pair(1280, 704), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
        monoLeft->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
        monoRight->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS),
    };
    std::vector<std::shared_ptr<dai::node::BenchmarkIn>> benchmarkNodes;
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    const int numMessagesToGet = static_cast<int>(FPS * static_cast<float>(SECONDS_TO_RUN));
    for(auto* output : cameraOutputs) {
        auto node = pipeline.create<dai::node::BenchmarkIn>();
        output->link(node->input);
        node->sendReportEveryNMessages(numMessagesToGet);
        queues.push_back(node->report.createOutputQueue());
        benchmarkNodes.push_back(node);
    }
    pipeline.start();
    int idx = 0;
    for(auto& queue : queues) {
        bool timedOut = false;
        std::cout << "Getting output: " << idx << "\n" << std::flush;
        auto report = queue->get<dai::BenchmarkReport>(std::chrono::seconds(SECONDS_TO_RUN + MAX_SECONDS_OFFSET), timedOut);
        std::cout << "Getting output: " << idx << " timed out: " << (timedOut ? "true" : "false") << "\n" << std::flush;
        REQUIRE(!timedOut);
        REQUIRE(report);
        if(report) {
            REQUIRE(report->numMessagesReceived == numMessagesToGet);
            REQUIRE(report->fps >= FPS - MAX_FPS_OFFSET);
            REQUIRE(report->fps <= FPS + MAX_FPS_OFFSET);
        }
        ++idx;
    }
}
