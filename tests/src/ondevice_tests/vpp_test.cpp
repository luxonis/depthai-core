#include "depthai/pipeline/node/Vpp.hpp"

#include <catch2/catch_all.hpp>
#include <chrono>
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
    const bool groupedInput = GENERATE(true, false);
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
    vpp->sync->setTimestampSource(node::Sync::TimestampSource::HOST);
    auto syncQueue = groupedInput ? vpp->syncedInputs.createInputQueue() : nullptr;
    auto leftQueue = groupedInput ? nullptr : vpp->left.createInputQueue();
    auto rightQueue = groupedInput ? nullptr : vpp->right.createInputQueue();
    auto disparityQueue = groupedInput ? nullptr : vpp->disparity.createInputQueue();
    auto confidenceQueue = groupedInput ? nullptr : vpp->confidence.createInputQueue();
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
    const auto timestamp = std::chrono::steady_clock::now();
    leftFrame->setTimestamp(timestamp);
    rightFrame->setTimestamp(timestamp);
    disparityFrame->setTimestamp(timestamp);
    confidenceFrame->setTimestamp(timestamp);
    if(groupedInput) {
        auto group = std::make_shared<MessageGroup>();
        group->add("left", leftFrame);
        group->add("right", rightFrame);
        group->add("disparity", disparityFrame);
        group->add("confidence", confidenceFrame);
        syncQueue->send(group);
    } else {
        leftQueue->send(leftFrame);
        rightQueue->send(rightFrame);
        disparityQueue->send(disparityFrame);
        confidenceQueue->send(confidenceFrame);
    }

    // Try to read outputs
    bool timedOut = false;
    auto leftOut = outLeftQueue->get<ImgFrame>(std::chrono::seconds(10), timedOut);
    REQUIRE_FALSE(timedOut);
    auto rightOut = outRightQueue->get<ImgFrame>(std::chrono::seconds(10), timedOut);
    REQUIRE_FALSE(timedOut);

    bool gotOutput = (leftOut && rightOut);

    REQUIRE(gotOutput);  // ✅ Pass if any output frame was received

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("DepthAI VPP gray8") {
    using namespace dai;

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
    vpp->sync->setTimestampSource(node::Sync::TimestampSource::HOST);
    auto leftQueue = vpp->left.createInputQueue();
    auto rightQueue = vpp->right.createInputQueue();
    auto disparityQueue = vpp->disparity.createInputQueue();
    auto confidenceQueue = vpp->confidence.createInputQueue();
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
    const auto timestamp = std::chrono::steady_clock::now();
    leftFrame->setTimestamp(timestamp);
    rightFrame->setTimestamp(timestamp);
    disparityFrame->setTimestamp(timestamp);
    confidenceFrame->setTimestamp(timestamp);
    leftQueue->send(leftFrame);
    rightQueue->send(rightFrame);
    disparityQueue->send(disparityFrame);
    confidenceQueue->send(confidenceFrame);

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
    vpp->sync->setTimestampSource(node::Sync::TimestampSource::HOST);
    auto leftQueue = vpp->left.createInputQueue();
    auto rightQueue = vpp->right.createInputQueue();
    auto disparityQueue = vpp->disparity.createInputQueue();
    auto confidenceQueue = vpp->confidence.createInputQueue();
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
        vppConfig->injectionParameters.confidenceThreshold = 1.5;

        // Fake input frames
        cv::Mat left(800, 1280, CV_8UC1, cv::Scalar(0));
        cv::Mat right(800, 1280, CV_8UC1, cv::Scalar(0));
        cv::Mat disparity(16, 16, CV_16UC1, cv::Scalar(32));
        cv::Mat confidence(16, 16, CV_16UC1, cv::Scalar(32));

        // Send config and frames
        configQueue->send(vppConfig);
        auto leftFrame = openCvToFrame(left, ImgFrame::Type::GRAY8);
        auto rightFrame = openCvToFrame(right, ImgFrame::Type::GRAY8);
        auto disparityFrame = openCvToFrame(disparity, ImgFrame::Type::RAW16);
        auto confidenceFrame = openCvToFrame(confidence, ImgFrame::Type::RAW16);

        const auto timestamp = std::chrono::steady_clock::now();
        leftFrame->setTimestamp(timestamp);
        rightFrame->setTimestamp(timestamp);
        disparityFrame->setTimestamp(timestamp);
        confidenceFrame->setTimestamp(timestamp);
        leftQueue->send(leftFrame);
        rightQueue->send(rightFrame);
        disparityQueue->send(disparityFrame);
        confidenceQueue->send(confidenceFrame);

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

TEST_CASE("DepthAI VPP rejects depth and disparity together", "[vpp-host]") {
    auto source = node::ImageManip::create();
    auto vpp = node::Vpp::create();
    source->out.link(vpp->left);
    source->out.link(vpp->right);
    source->out.link(vpp->depth);
    REQUIRE(vpp->depth.isConnected());
    REQUIRE_FALSE(vpp->disparity.isConnected());

    source->out.link(vpp->disparity);
    REQUIRE_THROWS_AS(vpp->postBuildStage(), std::invalid_argument);
}

TEST_CASE("DepthAI VPP rejects neither depth nor disparity", "[vpp-host]") {
    auto source = node::ImageManip::create();
    auto vpp = node::Vpp::create();
    source->out.link(vpp->left);
    source->out.link(vpp->right);

    vpp->buildStage1();
    REQUIRE_THROWS_AS(vpp->postBuildStage(), std::invalid_argument);
}

TEST_CASE("DepthAI VPP accepts direct synchronized groups", "[vpp-host]") {
    auto source = node::Sync::create();
    auto vpp = node::Vpp::create();
    source->out.link(vpp->syncedInputs);
    for(int build = 0; build < 2; ++build) {
        REQUIRE_NOTHROW(vpp->buildStage1());
        REQUIRE_NOTHROW(vpp->postBuildStage());
    }

    // The internal Sync connection alone must not count as a direct producer.
    source->out.unlink(vpp->syncedInputs);
    vpp->buildStage1();
    REQUIRE_THROWS_AS(vpp->postBuildStage(), std::invalid_argument);
}

TEST_CASE("DepthAI VPP preserves unlinked optional inputs", "[vpp-host]") {
    const bool useDepth = GENERATE(true, false);
    auto source = node::ImageManip::create();
    auto vpp = node::Vpp::create();
    source->out.link(vpp->left);
    source->out.link(vpp->right);
    source->out.link(useDepth ? vpp->depth : vpp->disparity);
    auto* depth = &vpp->depth;
    auto* disparity = &vpp->disparity;
    auto* confidence = &vpp->confidence;

    // Starting again must preserve the public inputs as well as the active Sync inputs.
    for(int build = 0; build < 2; ++build) {
        REQUIRE_NOTHROW(vpp->postBuildStage());
        REQUIRE(vpp->sync->inputs.has("depth") == useDepth);
        REQUIRE(vpp->sync->inputs.has("disparity") == !useDepth);
        REQUIRE_FALSE(vpp->sync->inputs.has("confidence"));
        REQUIRE(&vpp->depth == depth);
        REQUIRE(&vpp->disparity == disparity);
        REQUIRE(&vpp->confidence == confidence);
        REQUIRE(vpp->depth.isConnected() == useDepth);
        REQUIRE(vpp->disparity.isConnected() == !useDepth);
        REQUIRE_FALSE(vpp->confidence.isConnected());
        REQUIRE(vpp->depth.getName() == "depth");
        REQUIRE(vpp->disparity.getName() == "disparity");
        REQUIRE(vpp->confidence.getName() == "confidence");
    }

    // A previously inactive public input can be connected before the next build stage.
    source->out.link(vpp->confidence);
    REQUIRE_NOTHROW(vpp->postBuildStage());
    REQUIRE(vpp->sync->inputs.has("confidence"));
    REQUIRE(&vpp->confidence == confidence);
    REQUIRE(vpp->confidence.isConnected());
}

TEST_CASE("DepthAI VPP accepts depth or disparity without confidence", "[depth-fw]") {
    const bool useDepth = GENERATE(true, false);
    Pipeline pipeline;
    auto vpp = pipeline.create<node::Vpp>();
    vpp->initialConfig->injectionParameters.useInjection = false;
    vpp->initialConfig->patchColoringType = VppConfig::PatchColoringType::RANDOM;
    vpp->initialConfig->blending = 1.0f;
    vpp->sync->setTimestampSource(node::Sync::TimestampSource::HOST);
    auto leftQueue = vpp->left.createInputQueue();
    auto rightQueue = vpp->right.createInputQueue();
    auto priorQueue = (useDepth ? vpp->depth : vpp->disparity).createInputQueue();
    auto outLeftQueue = vpp->leftOut.createOutputQueue();
    auto outRightQueue = vpp->rightOut.createOutputQueue();

    cv::Mat image(800, 1280, CV_8UC1, cv::Scalar(0));
    // 800 px focal length * 75 mm baseline / 1000 mm = 60 px (960 in Q4).
    cv::Mat prior(image.size(), CV_16UC1, cv::Scalar(useDepth ? 1000 : 960));
    auto leftFrame = std::make_shared<ImgFrame>();
    leftFrame->setCvFrame(image, ImgFrame::Type::GRAY8);
    auto rightFrame = std::make_shared<ImgFrame>();
    rightFrame->setCvFrame(image, ImgFrame::Type::GRAY8);
    auto priorFrame = std::make_shared<ImgFrame>();
    priorFrame->setCvFrame(prior, ImgFrame::Type::RAW16);

    // Depth conversion needs rectified intrinsics and a nonzero stereo baseline.
    ImgTransformation leftTransform(image.cols, image.rows);
    leftTransform.setIntrinsicMatrix({{{800.0f, 0.0f, 640.0f}, {0.0f, 800.0f, 400.0f}, {0.0f, 0.0f, 1.0f}}});
    Extrinsics leftExtrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, CameraBoardSocket::CAM_B, LengthUnit::MILLIMETER);
    leftTransform.setExtrinsics(leftExtrinsics);
    auto rightTransform = leftTransform;
    auto rightExtrinsics = leftExtrinsics;
    rightExtrinsics.translation.x = 75.0f;
    rightTransform.setExtrinsics(rightExtrinsics);
    leftFrame->setTransformation(leftTransform);
    rightFrame->setTransformation(rightTransform);
    priorFrame->setTransformation(leftTransform);
    const auto timestamp = std::chrono::steady_clock::now();
    leftFrame->setTimestamp(timestamp);
    rightFrame->setTimestamp(timestamp);
    priorFrame->setTimestamp(timestamp);

    pipeline.start();
    leftQueue->send(leftFrame);
    rightQueue->send(rightFrame);
    priorQueue->send(priorFrame);

    bool timedOut = false;
    auto leftOut = outLeftQueue->get<ImgFrame>(std::chrono::seconds(10), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(leftOut != nullptr);
    auto rightOut = outRightQueue->get<ImgFrame>(std::chrono::seconds(10), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(rightOut != nullptr);
    REQUIRE(leftOut->getCvFrame().size() == image.size());
    REQUIRE(rightOut->getCvFrame().size() == image.size());
    // With a nonzero prior and injection disabled, VPP must project a pattern.
    REQUIRE(cv::countNonZero(leftOut->getCvFrame()) > 0);
    REQUIRE(cv::countNonZero(rightOut->getCvFrame()) > 0);

    pipeline.stop();
    pipeline.wait();
}
