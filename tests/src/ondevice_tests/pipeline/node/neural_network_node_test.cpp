#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/NNModelDescription.hpp"

TEST_CASE("Cross platform NeuralNetwork API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto nn = p.create<dai::node::NeuralNetwork>()->build(camera, dai::NNModelDescription{"yolov6-nano"});

    auto outputQueue = nn->out.createOutputQueue();

    // Start pipeline
    p.start();

    // Get 10 tensors out and verify that they are not nullptr
    for(int i = 0; i < 10; i++) {
        auto tensor = outputQueue->get<dai::NNData>();
        REQUIRE(tensor != nullptr);
        REQUIRE_NOTHROW(tensor->getFirstTensor<float>());
    }
}

TEST_CASE("NNArchive API") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    std::string platform = dai::Device(p).getPlatformAsString();
    auto description = dai::NNModelDescription{"yolov6-nano", platform};
    auto nnArchive = dai::NNArchive(dai::getModelFromZoo(description));
    auto nn = p.create<dai::node::NeuralNetwork>()->build(camera, nnArchive);

    auto outputQueue = nn->out.createOutputQueue();

    // Start pipeline
    p.start();

    // Get 10 tensors out and verify that they are not nullptr
    for(int i = 0; i < 10; i++) {
        auto tensor = outputQueue->get<dai::NNData>();
        REQUIRE(tensor != nullptr);
        REQUIRE_NOTHROW(tensor->getFirstTensor<float>());
    }
}
