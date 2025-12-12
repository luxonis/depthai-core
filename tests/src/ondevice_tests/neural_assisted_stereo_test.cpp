#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

#include <catch2/catch_all.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

using namespace dai;

std::shared_ptr<ImgFrame> createTestFrame(const cv::Mat& mat, ImgFrame::Type type) {
    auto frame = std::make_shared<ImgFrame>();
    frame->setType(type);
    frame->setWidth(mat.cols);
    frame->setHeight(mat.rows);

    size_t totalBytes = mat.total() * mat.elemSize();

    if(mat.isContinuous()) {
        frame->setData(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(mat.data), reinterpret_cast<uint8_t*>(mat.data) + totalBytes));
    } else {
        std::vector<uint8_t> data;
        data.reserve(totalBytes);
        for(int i = 0; i < mat.rows; ++i) {
            const uint8_t* rowPtr = reinterpret_cast<const uint8_t*>(mat.ptr(i));
            data.insert(data.end(), rowPtr, rowPtr + mat.cols * mat.elemSize());
        }
        frame->setData(data);
    }

    return frame;
}

TEST_CASE("NeuralAssistedStereo basic pipeline") {
    Pipeline pipeline;
    
    // Create camera nodes as input sources (Camera is supported on RVC4; MonoCamera is deprecated)
    auto camLeft = pipeline.create<node::Camera>();
    auto camRight = pipeline.create<node::Camera>();
    camLeft->build(CameraBoardSocket::CAM_B);
    camRight->build(CameraBoardSocket::CAM_C);

    // Request RAW16 at the native OV9282 resolution to avoid unsupported sensor modes
    auto leftOut = camLeft->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    auto rightOut = camRight->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    
    // Create and build NeuralAssistedStereo node
    auto nas = pipeline.create<node::NeuralAssistedStereo>();
    
    // Build the composite node with camera outputs
    //nas->build(*leftOut, *rightOut, DeviceModelZoo::NEURAL_DEPTH_NANO);
    
    // Configure VPP (start with injection disabled to avoid FW crash on RVC4)
    nas->vppConfig->maxPatchSize = 20;
    nas->vppConfig->patchColoringType = VppConfig::PatchColoringType::MAXDIST;
    nas->vppConfig->blending = 0.5f;
    nas->vppConfig->uniformPatch = true;
    nas->vppConfig->injectionParameters.textureThreshold = 0.0f;
    nas->vppConfig->injectionParameters.useInjection = false;

    // Configure StereoDepth
    nas->stereoConfig->setMedianFilter(StereoDepthConfig::MedianFilter::KERNEL_7x7);
    nas->stereoConfig->setConfidenceThreshold(200);
    
    nas->build(*leftOut, *rightOut, DeviceModelZoo::NEURAL_DEPTH_NANO);


    // Create output queues
    auto depthQueue = nas->depth.createOutputQueue();
    auto disparityQueue = nas->disparity.createOutputQueue();

    pipeline.start();

    // Try to receive outputs (with timeout)
    auto depthOut = depthQueue->tryGet<ImgFrame>();
    auto disparityOut = disparityQueue->tryGet<ImgFrame>();

    bool gotOutput = (depthOut && disparityOut);

    REQUIRE(gotOutput);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("NeuralAssistedStereo with MonoCamera") {
    Pipeline pipeline;
    
    // Create camera nodes
    auto camLeft = pipeline.create<node::Camera>();
    auto camRight = pipeline.create<node::Camera>();
    camLeft->build(CameraBoardSocket::CAM_B);
    camRight->build(CameraBoardSocket::CAM_C);

    auto leftOut = camLeft->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    auto rightOut = camRight->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    
    // Create NeuralAssistedStereo
    auto nas = pipeline.create<node::NeuralAssistedStereo>();
    nas->build(*leftOut, *rightOut);
    
    // Configure with minimal settings
    nas->vppConfig->maxPatchSize = 20;
    nas->vppConfig->blending = 0.5f;
    nas->vppConfig->uniformPatch = true;
    nas->vppConfig->injectionParameters.textureThreshold = 0.0f;
    nas->vppConfig->injectionParameters.useInjection = false;
    
    auto depthQueue = nas->depth.createOutputQueue();
    auto disparityQueue = nas->disparity.createOutputQueue();

    pipeline.start();

    auto depthOut = depthQueue->get<ImgFrame>();
    auto disparityOut = disparityQueue->get<ImgFrame>();

    bool gotOutput = (depthOut && disparityOut);

    REQUIRE(gotOutput);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("NeuralAssistedStereo multiple configs") {
    Pipeline pipeline;
    
    // Create camera nodes
    auto camLeft = pipeline.create<node::Camera>();
    auto camRight = pipeline.create<node::Camera>();
    camLeft->build(CameraBoardSocket::CAM_B);
    camRight->build(CameraBoardSocket::CAM_C);

    auto leftOut = camLeft->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    auto rightOut = camRight->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    
    // Create NeuralAssistedStereo
    auto nas = pipeline.create<node::NeuralAssistedStereo>();
    nas->build(*leftOut, *rightOut);
    
    // Configure VPP
    nas->vppConfig->maxPatchSize = 30;
    nas->vppConfig->blending = 0.7f;
    nas->vppConfig->uniformPatch = false;
    nas->vppConfig->injectionParameters.textureThreshold = 0.0f;
    nas->vppConfig->injectionParameters.useInjection = false;
    
    // Configure stereo
    nas->stereoConfig->setMedianFilter(StereoDepthConfig::MedianFilter::KERNEL_7x7);
    nas->stereoConfig->setConfidenceThreshold(200);
    
    auto depthQueue = nas->depth.createOutputQueue();

    pipeline.start();

    // Get first frame
    auto depthOut1 = depthQueue->get<ImgFrame>();
    REQUIRE(depthOut1 != nullptr);
    
    auto depthCV = depthOut1->getCvFrame();
    REQUIRE(depthCV.rows > 0);
    REQUIRE(depthCV.cols > 0);

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("NeuralAssistedStereo intermediate outputs") {
    Pipeline pipeline;

    auto device = pipeline.getDefaultDevice();
    if(device && device->getPlatform() == Platform::RVC4) {
        WARN("Skipping VPP intermediate output test on RVC4 (VPP injection disabled to avoid FW crash)");
        return;
    }
    
    // Create camera nodes
    auto camLeft = pipeline.create<node::Camera>();
    auto camRight = pipeline.create<node::Camera>();
    camLeft->build(CameraBoardSocket::CAM_B);
    camRight->build(CameraBoardSocket::CAM_C);

    auto leftOut = camLeft->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    auto rightOut = camRight->requestOutput({1280, 800}, ImgFrame::Type::RAW16);
    
    // Create NeuralAssistedStereo
    auto nas = pipeline.create<node::NeuralAssistedStereo>();
    nas->build(*leftOut, *rightOut);
    
    // Access intermediate outputs
    auto rectifiedLeftQueue = nas->rectifiedLeft.createOutputQueue();
    auto rectifiedRightQueue = nas->rectifiedRight.createOutputQueue();
    auto vppLeftQueue = nas->vppLeft.createOutputQueue();
    auto vppRightQueue = nas->vppRight.createOutputQueue();
    auto neuralDisparityQueue = nas->neuralDisparity.createOutputQueue();
    auto neuralConfidenceQueue = nas->neuralConfidence.createOutputQueue();
    auto depthQueue = nas->depth.createOutputQueue();
    
    pipeline.start();

    // Check all intermediate outputs are produced
    auto rectLeftOut = rectifiedLeftQueue->get<ImgFrame>();
    auto rectRightOut = rectifiedRightQueue->get<ImgFrame>();
    auto vppLeftOut = vppLeftQueue->get<ImgFrame>();
    auto vppRightOut = vppRightQueue->get<ImgFrame>();
    auto neuralDispOut = neuralDisparityQueue->get<ImgFrame>();
    auto neuralConfOut = neuralConfidenceQueue->get<ImgFrame>();
    auto depthOut = depthQueue->get<ImgFrame>();

    REQUIRE(rectLeftOut != nullptr);
    REQUIRE(rectRightOut != nullptr);
    REQUIRE(vppLeftOut != nullptr);
    REQUIRE(vppRightOut != nullptr);
    REQUIRE(neuralDispOut != nullptr);
    REQUIRE(neuralConfOut != nullptr);
    REQUIRE(depthOut != nullptr);

    // Verify neural confidence is 8-bit (as per DFS spec)
    auto neuralConfCV = neuralConfOut->getCvFrame();
    REQUIRE(neuralConfCV.type() == CV_8UC1);  // 8-bit unsigned confidence map

    pipeline.stop();
    pipeline.wait();
}
