#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

bool isIdentity(const std::array<std::array<float, 3>, 3>& mat) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            if(i == j && mat[i][j] != 1) return false;
            if(i != j && mat[i][j] != 0) return false;
        }
    }
    return true;
}

TEST_CASE("ImgTransformation in ImgFrame") {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>();
    auto camOut = cam->requestOutput({1300, 200}, dai::ImgFrame::Type::NV12);
    auto q = camOut->createOutputQueue();
    pipeline.start();
    auto frame = q->get<dai::ImgFrame>();
    REQUIRE(frame != nullptr);
    pipeline.stop();
    REQUIRE(frame->validateTransformations());
    REQUIRE(!isIdentity(frame->transformation.getMatrix()));
    REQUIRE(!isIdentity(frame->transformation.getMatrixInv()));
    REQUIRE(!isIdentity(frame->transformation.getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(frame->transformation.getSourceIntrinsicMatrixInv()));
}

TEST_CASE("ImgTransformation in SpatialDetectionNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>()->build(*left->requestFullResolutionOutput(), *right->requestFullResolutionOutput());
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::SpatialDetectionNetwork>();
    REQUIRE_NOTHROW(nn->build(camera, stereo, modelDesc));
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::SpatialImgDetections>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}

TEST_CASE("ImgTransformation in DetectionNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::DetectionNetwork>()->build(camera, modelDesc);
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::ImgDetections>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}

TEST_CASE("ImgTransformation in NeuralNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::NeuralNetwork>()->build(camera, modelDesc);
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::NNData>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}
