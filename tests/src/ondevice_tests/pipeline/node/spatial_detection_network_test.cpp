#include <catch2/catch_all.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

TEST_CASE("SpatialDetectionNetwork sets device for all subnodes") {
    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::SpatialDetectionNetwork>();
    auto pipelineDevice = p.getDefaultDevice();

    REQUIRE(nn->getDevice() == pipelineDevice);
    REQUIRE(nn->neuralNetwork->getDevice() == pipelineDevice);
    REQUIRE(nn->detectionParser->getDevice() == pipelineDevice);
}

TEST_CASE("Detection network model description API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto nn = p.create<dai::node::DetectionNetwork>();
    auto detectionsQueue = nn->out.createOutputQueue();
    // Load NNArchive
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    REQUIRE_NOTHROW(nn->build(camera, modelDesc));
    p.start();

    for(int i = 0; i < 10; i++) {
        auto tensor = detectionsQueue->get<dai::ImgDetections>();
        REQUIRE(tensor != nullptr);
    }
}

TEST_CASE("Detection network NNArchive API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto nn = p.create<dai::node::DetectionNetwork>();
    auto detectionsQueue = nn->out.createOutputQueue();
    // Load NNArchive
    auto platform = p.getDefaultDevice()->getPlatformAsString();
    dai::NNModelDescription modelDesc{"yolov6-nano", platform};
    auto nnArchive = dai::NNArchive(dai::getModelFromZoo(modelDesc));
    REQUIRE_NOTHROW(nn->build(camera, nnArchive));
    p.start();

    for(int i = 0; i < 10; i++) {
        auto tensor = detectionsQueue->get<dai::ImgDetections>();
        REQUIRE(tensor != nullptr);
    }
}

TEST_CASE("SpatialDetectionNetwork model description API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = p.create<dai::node::StereoDepth>()->build(*left->requestOutput(std::make_pair(640, 400)), *right->requestOutput(std::make_pair(640, 400)));
    stereo->setSubpixel(false);
    if(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) stereo->setOutputSize(640, 400);

    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = p.create<dai::node::SpatialDetectionNetwork>();
    REQUIRE_NOTHROW(nn->build(camera, stereo, modelDesc));
    auto detectionsQueue = nn->out.createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        auto tensor = detectionsQueue->get<dai::SpatialImgDetections>();
        REQUIRE(tensor != nullptr);
    }
}

TEST_CASE("SpatialDetectionNetwork NNArchive API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = p.create<dai::node::StereoDepth>()->build(*left->requestOutput(std::make_pair(640, 400)), *right->requestOutput(std::make_pair(640, 400)));
    stereo->setSubpixel(false);
    if(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) stereo->setOutputSize(640, 400);
    // Load NNArchive
    auto platform = p.getDefaultDevice()->getPlatformAsString();
    dai::NNModelDescription modelDesc{"yolov6-nano", platform};
    auto nnArchive = dai::NNArchive(dai::getModelFromZoo(modelDesc));
    auto nn = p.create<dai::node::SpatialDetectionNetwork>();
    REQUIRE_NOTHROW(nn->build(camera, stereo, nnArchive));
    auto detectionsQueue = nn->out.createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        auto tensor = detectionsQueue->get<dai::SpatialImgDetections>();
        REQUIRE(tensor != nullptr);
    }
}
