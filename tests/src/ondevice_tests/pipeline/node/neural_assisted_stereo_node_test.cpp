#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

TEST_CASE("[NeuralAssistedStereo] Check that I am getting output from the subnodes") {
    // Create pipeline
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping NeuralAssistedStereo test: device doesn't support NeuralDepth.");
        return;
    }
    auto cameraLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto cameraLeftOut = cameraLeft->requestFullResolutionOutput();
    auto cameraRightOut = cameraRight->requestFullResolutionOutput();

    auto cameraLeftQueue = cameraLeftOut->createOutputQueue();
    auto cameraRightQueue = cameraRightOut->createOutputQueue();

    auto nn = p.create<dai::node::NeuralAssistedStereo>()->build(*cameraLeftOut, *cameraRightOut);

    auto outputDisparityQueue = nn->disparity.createOutputQueue();
    auto outputDepthQueue = nn->depth.createOutputQueue();

    auto outputVppLeftQueue = nn->vppLeft.createOutputQueue();
    auto outputVppRightQueue = nn->vppRight.createOutputQueue();

    auto outputRectifiedLeftQueue = nn->rectifiedLeft.createOutputQueue();
    auto outputRectifiedRightQueue = nn->rectifiedRight.createOutputQueue();

    // Start pipeline
    p.start();
    auto leftGot = cameraLeftQueue->get<dai::ImgFrame>();
    auto rightGot = cameraRightQueue->get<dai::ImgFrame>();

    auto disparityGot = outputDisparityQueue->get<dai::ImgFrame>();
    auto disparityGotCv = disparityGot->getCvFrame();
    auto depthGot = outputDepthQueue->get<dai::ImgFrame>();

    auto vppLeftGot = outputVppLeftQueue->get<dai::ImgFrame>();
    auto vppLeftGotCv = vppLeftGot->getCvFrame();
    auto vppRightGot = outputVppRightQueue->get<dai::ImgFrame>();
    auto vppRightGotCv = vppRightGot->getCvFrame();

    auto rectifiedLeftGot = outputRectifiedLeftQueue->get<dai::ImgFrame>();
    auto rectifiedLeftGotCv = rectifiedLeftGot->getCvFrame();
    auto rectifiedRightGot = outputRectifiedRightQueue->get<dai::ImgFrame>();
    auto rectifiedRightGotCv = rectifiedRightGot->getCvFrame();

    p.stop();

    REQUIRE(rectifiedLeftGotCv.rows == leftGot->getCvFrame().rows);
    REQUIRE(rectifiedRightGotCv.rows == rightGot->getCvFrame().rows);
    REQUIRE(vppLeftGotCv.rows == rectifiedLeftGotCv.rows);
    REQUIRE(disparityGotCv.rows == disparityGotCv.rows);

    REQUIRE(rectifiedLeftGotCv.cols == leftGot->getCvFrame().cols);
    REQUIRE(rectifiedRightGotCv.cols == rightGot->getCvFrame().cols);
    REQUIRE(vppLeftGotCv.cols == rectifiedLeftGotCv.cols);
    REQUIRE(disparityGotCv.cols == disparityGotCv.cols);
}

TEST_CASE("[NeuralAssistedStereo] Case without rectification") {
    // Create pipeline
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping NeuralAssistedStereo test: device doesn't support NeuralDepth.");
        return;
    }
    auto cameraLeft = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto cameraLeftOut = cameraLeft->requestFullResolutionOutput();
    auto cameraRightOut = cameraRight->requestFullResolutionOutput();

    auto nn = p.create<dai::node::NeuralAssistedStereo>()->build(*cameraLeftOut, *cameraRightOut, dai::DeviceModelZoo::NEURAL_DEPTH_NANO, false);
    auto outputRectifiedLeftQueue = nn->rectifiedLeft.createOutputQueue();
    auto outputRectifiedRightQueue = nn->rectifiedRight.createOutputQueue();

    // Start pipeline
    p.start();

    for(int i = 0; i < 100; i++) {
        auto leftOut = outputRectifiedLeftQueue->tryGet<dai::ImgFrame>();
        auto rightOut = outputRectifiedRightQueue->tryGet<dai::ImgFrame>();
        REQUIRE(leftOut == nullptr);
        REQUIRE(rightOut == nullptr);
    }
    p.stop();
}
