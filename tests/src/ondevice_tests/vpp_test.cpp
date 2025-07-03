#include "depthai/pipeline/node/Vpp.hpp"

#include <catch2/catch_all.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

using namespace dai;

std::shared_ptr<ImgFrame> openCvToFrame(const cv::Mat& mat, ImgFrame::Type type) {
    auto frame = std::make_shared<ImgFrame>();
    frame->setType(type);

    // Set width and height
    frame->setWidth(mat.cols);
    frame->setHeight(mat.rows);

    // Determine number of bytes per pixel
    size_t totalBytes = mat.total() * mat.elemSize();

    // Copy data
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

TEST_CASE("DepthAI VPP RAW8") {
    // Connect to the device
    auto device = std::make_shared<Device>();
    // Configure VPP
    auto vppConfig = std::make_shared<VppConfig>();
    vppConfig->maxPatchSize = 20;
    vppConfig->patchColoringType = VppConfig::PatchColoringType::MAXDIST;
    vppConfig->blending = 0.5f;
    vppConfig->uniformPatch = true;

    // Modify nested injection parameters
    vppConfig->injectionParameters.textureThreshold = 4.0;
    vppConfig->injectionParameters.useInjection = true;

    // Build the pipeline
    Pipeline pipeline;
    auto vpp = pipeline.create<node::Vpp>();
    auto syncQueue = vpp->syncedInputs.createInputQueue();
    auto configQueue = vpp->inputConfig.createInputQueue();
    auto outLeftQueue = vpp->leftOut.createOutputQueue();
    auto outRightQueue = vpp->rightOut.createOutputQueue();

    // Send config
    configQueue->send(vppConfig);

    // Create fake disparity and confidence frames
    cv::Mat disparity(16, 16, CV_16UC1, cv::Scalar(32));
    cv::Mat confidence(16, 16, CV_16UC1, cv::Scalar(0));

    cv::Mat left(1280, 800, CV_8UC1, cv::Scalar(0));
    cv::Mat right(1280, 800, CV_8UC1, cv::Scalar(0));

    auto leftFrame = std::make_shared<ImgFrame>();
    leftFrame->setCvFrame(left, ImgFrame::Type::RAW8);

    auto rightFrame = std::make_shared<ImgFrame>();
    rightFrame->setCvFrame(right, ImgFrame::Type::RAW8);

    auto disparityFrame = std::make_shared<ImgFrame>();
    disparityFrame->setCvFrame(disparity, ImgFrame::Type::RAW16);

    auto confidenceFrame = std::make_shared<ImgFrame>();
    confidenceFrame->setCvFrame(confidence, ImgFrame::Type::RAW16);

    pipeline.start();
    // Send message group
    auto messageGroup = std::make_shared<MessageGroup>();
    messageGroup->add("left", leftFrame);
    messageGroup->add("right", rightFrame);
    messageGroup->add("disparity", disparityFrame);
    messageGroup->add("confidence", confidenceFrame);
    syncQueue->send(messageGroup);

    // Try to read outputs
    auto leftOut = outLeftQueue->get<ImgFrame>();
    auto rightOut = outRightQueue->get<ImgFrame>();

    bool gotOutput = (leftOut && rightOut);

    REQUIRE(gotOutput);  // ✅ Pass if any output frame was received

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DepthAI VPP gray8") {
    using namespace dai;

    // Connect to the device
    auto device = std::make_shared<Device>();
    // Configure VPP
    auto vppConfig = std::make_shared<VppConfig>();
    vppConfig->maxPatchSize = 20;
    vppConfig->patchColoringType = VppConfig::PatchColoringType::MAXDIST;
    vppConfig->blending = 0.5f;
    vppConfig->uniformPatch = true;

    // Modify nested injection parameters
    vppConfig->injectionParameters.textureThreshold = 4.0;
    vppConfig->injectionParameters.useInjection = true;

    // Build the pipeline
    Pipeline pipeline;
    auto vpp = pipeline.create<node::Vpp>();
    auto syncQueue = vpp->syncedInputs.createInputQueue();
    auto configQueue = vpp->inputConfig.createInputQueue();
    auto outLeftQueue = vpp->leftOut.createOutputQueue();
    auto outRightQueue = vpp->rightOut.createOutputQueue();

    // Send config
    configQueue->send(vppConfig);

    // Create fake disparity and confidence frames
    cv::Mat disparity(16, 16, CV_16UC1, cv::Scalar(32));
    cv::Mat confidence(16, 16, CV_16UC1, cv::Scalar(0));

    cv::Mat left(1280, 800, CV_8UC1, cv::Scalar(0));
    cv::Mat right(1280, 800, CV_8UC1, cv::Scalar(0));

    auto leftFrame = std::make_shared<ImgFrame>();
    leftFrame->setCvFrame(left, ImgFrame::Type::GRAY8);

    auto rightFrame = std::make_shared<ImgFrame>();
    rightFrame->setCvFrame(right, ImgFrame::Type::GRAY8);

    auto disparityFrame = std::make_shared<ImgFrame>();
    disparityFrame->setCvFrame(disparity, ImgFrame::Type::RAW16);

    auto confidenceFrame = std::make_shared<ImgFrame>();
    confidenceFrame->setCvFrame(confidence, ImgFrame::Type::RAW16);

    pipeline.start();
    // Send message group
    auto messageGroup = std::make_shared<MessageGroup>();
    messageGroup->add("left", leftFrame);
    messageGroup->add("right", rightFrame);
    messageGroup->add("disparity", disparityFrame);
    messageGroup->add("confidence", confidenceFrame);
    syncQueue->send(messageGroup);

    // Try to read outputs
    auto leftOut = outLeftQueue->get<ImgFrame>();
    auto rightOut = outRightQueue->get<ImgFrame>();

    bool gotOutput = (leftOut && rightOut);

    REQUIRE(gotOutput);  // ✅ Pass if any output frame was received

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DepthAI VPP multiple configs without recreating pipeline") {
    using namespace dai;

    // Create pipeline once
    Pipeline pipeline;
    auto vpp = pipeline.create<node::Vpp>();
    auto syncQueue = vpp->syncedInputs.createInputQueue();
    auto configQueue = vpp->inputConfig.createInputQueue();
    auto outLeftQueue = vpp->leftOut.createOutputQueue();
    auto outRightQueue = vpp->rightOut.createOutputQueue();
    pipeline.start();

    // Define parameter combinations
    struct VppParams {
        int maxPatchSize;
        float blending;
        bool uniformPatch;
        float textureThreshold;
        bool useInjection;
        VppConfig::PatchColoringType patchColoring;
    };

    std::vector<VppParams> paramSets = {
        {10, 0.1f, true, 1.0f, false, VppConfig::PatchColoringType::RANDOM},
        {20, 0.5f, false, 4.0f, false, VppConfig::PatchColoringType::MAXDIST},
        {30, 1.0f, true, 8.0f, true, VppConfig::PatchColoringType::MAXDIST},
        {100, 1.0f, true, 8.0f, false, VppConfig::PatchColoringType::MAXDIST},
    };

    for(auto& params : paramSets) {
        // Configure VPP
        auto vppConfig = std::make_shared<VppConfig>();
        vppConfig->maxPatchSize = params.maxPatchSize;
        vppConfig->blending = params.blending;
        vppConfig->uniformPatch = params.uniformPatch;
        vppConfig->patchColoringType = params.patchColoring;
        vppConfig->injectionParameters.textureThreshold = params.textureThreshold;
        vppConfig->injectionParameters.useInjection = params.useInjection;

        // Fake input frames
        cv::Mat left(800, 1280, CV_8UC1, cv::Scalar(0));
        cv::Mat right(800, 1280, CV_8UC1, cv::Scalar(0));
        cv::Mat disparity(16, 16, CV_16UC1, cv::Scalar(32));
        cv::Mat confidence(16, 16, CV_16UC1, cv::Scalar(1));

        // Send config and frames
        configQueue->send(vppConfig);
        auto leftFrame = openCvToFrame(left, ImgFrame::Type::GRAY8);
        auto rightFrame = openCvToFrame(right, ImgFrame::Type::GRAY8);
        auto disparityFrame = openCvToFrame(disparity, ImgFrame::Type::RAW16);
        auto confidenceFrame = openCvToFrame(confidence, ImgFrame::Type::RAW16);

        auto messageGroup = std::make_shared<MessageGroup>();
        messageGroup->add("left", leftFrame);
        messageGroup->add("right", rightFrame);
        messageGroup->add("disparity", disparityFrame);
        messageGroup->add("confidence", confidenceFrame);

        syncQueue->send(messageGroup);

        // Check outputs
        auto leftOut = outLeftQueue->get<ImgFrame>();
        auto rightOut = outRightQueue->get<ImgFrame>();
        REQUIRE(leftOut != nullptr);
        REQUIRE(rightOut != nullptr);
        auto leftOutCV = leftOut->getCvFrame();
        auto rightOutCV = rightOut->getCvFrame();

        REQUIRE(leftOutCV.rows == left.rows);
        REQUIRE(leftOutCV.cols == left.cols);
        REQUIRE(rightOutCV.rows == right.rows);
        REQUIRE(rightOutCV.cols == right.cols);

        // Assert that output types match the input frames
        REQUIRE(leftOutCV.type() == left.type());
        REQUIRE(rightOutCV.type() == right.type());
        REQUIRE(cv::countNonZero(leftOutCV) > 0);
        REQUIRE(cv::countNonZero(rightOutCV) > 0);
    }

    pipeline.stop();
    pipeline.wait();
}
