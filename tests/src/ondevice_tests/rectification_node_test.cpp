#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>

#include "depthai/depthai.hpp"

namespace {

std::shared_ptr<dai::ImgFrame> receive(const std::shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame);
    return frame;
}

}  // namespace

TEST_CASE("RVC2 standalone Rectification matches StereoDepth", "[Rectification][ondevice]") {
    const auto enabled = GENERATE(true, false);
    const auto backend = GENERATE(dai::node::Rectification::Backend::AUTO, dai::node::Rectification::Backend::HW);
    CAPTURE(enabled, backend);
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC2) SKIP("Requires RVC2");
    const auto pairs = pipeline.getDefaultDevice()->getAvailableStereoPairs();
    if(pairs.empty()) SKIP("Requires a calibrated stereo pair");
    const auto pair = pairs.front();
    auto left = pipeline.create<dai::node::Camera>()->build(pair.left);
    auto right = pipeline.create<dai::node::Camera>()->build(pair.right);
    auto leftOut = left->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 5.0f);
    auto rightOut = right->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 5.0f);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setRectification(enabled);
    stereo->initialConfig->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
    auto rectify = pipeline.create<dai::node::Rectification>();
    rectify->setBackend(backend).setOutputSize(640, 400).enableRectification(enabled);
    REQUIRE_FALSE(rectify->runOnHost());
    leftOut->link(stereo->left);
    rightOut->link(stereo->right);
    leftOut->link(rectify->input1);
    rightOut->link(rectify->input2);

    std::array<std::shared_ptr<dai::MessageQueue>, 2> reference = {stereo->rectifiedLeft.createOutputQueue(16, true),
                                                                   stereo->rectifiedRight.createOutputQueue(16, true)};
    std::array<std::shared_ptr<dai::MessageQueue>, 2> outputs = {rectify->output1.createOutputQueue(16, true), rectify->output2.createOutputQueue(16, true)};
    std::array<std::shared_ptr<dai::MessageQueue>, 2> passthrough = {rectify->passthrough1.createOutputQueue(16, true),
                                                                     rectify->passthrough2.createOutputQueue(16, true)};
    std::array<std::shared_ptr<dai::MessageQueue>, 2> synced = {stereo->syncedLeft.createOutputQueue(16, true),
                                                                stereo->syncedRight.createOutputQueue(16, true)};
    pipeline.start();
    REQUIRE_FALSE(stereo->properties.enableRectification);
    REQUIRE(stereo->getNodeMap().size() == (enabled ? 2 : 1));
    for(int sequence = 0; sequence < 10; ++sequence) {
        for(size_t side = 0; side < outputs.size(); ++side) {
            auto actual = receive(outputs[side]);
            auto expected = receive(reference[side]);
            // Both branches synchronize independently and may discard different startup frames.
            while(expected->getSequenceNum() < actual->getSequenceNum()) expected = receive(reference[side]);
            while(actual->getSequenceNum() < expected->getSequenceNum()) actual = receive(outputs[side]);
            REQUIRE(actual->getSequenceNum() == expected->getSequenceNum());
            REQUIRE(actual->getWidth() == 640);
            REQUIRE(actual->getHeight() == 400);
            REQUIRE(std::equal(actual->getData().begin(), actual->getData().end(), expected->getData().begin(), expected->getData().end()));
            REQUIRE(actual->getTimestampDevice() == expected->getTimestampDevice());
            REQUIRE(actual->transformation.isEqualTransformation(expected->transformation));
            auto original = receive(passthrough[side]);
            while(original->getSequenceNum() < actual->getSequenceNum()) original = receive(passthrough[side]);
            REQUIRE(original->getSequenceNum() == actual->getSequenceNum());
            auto synchronized = receive(synced[side]);
            while(synchronized->getSequenceNum() < expected->getSequenceNum()) synchronized = receive(synced[side]);
            REQUIRE(synchronized->getSequenceNum() == expected->getSequenceNum());
            REQUIRE(std::equal(synchronized->getData().begin(), synchronized->getData().end(), original->getData().begin(), original->getData().end()));
            REQUIRE(synchronized->transformation.isEqualTransformation(original->transformation));
            if(enabled) {
                for(float coefficient : actual->transformation.getDistortionCoefficients()) REQUIRE(coefficient == 0.0f);
            } else {
                REQUIRE(std::equal(actual->getData().begin(), actual->getData().end(), original->getData().begin(), original->getData().end()));
                REQUIRE(actual->transformation.isEqualTransformation(original->transformation));
            }
        }
    }
}

TEST_CASE("RVC2 StereoDepth uses Sync to discard unmatched input frames", "[StereoDepth][Sync][ondevice]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC2) SKIP("Requires RVC2");
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setRectification(false);
    auto left = stereo->left.createInputQueue();
    auto right = stereo->right.createInputQueue();
    auto syncedLeft = stereo->syncedLeft.createOutputQueue();
    auto syncedRight = stereo->syncedRight.createOutputQueue();
    auto rectified = stereo->rectifiedLeft.createOutputQueue();
    pipeline.start();
    auto frame = [](dai::CameraBoardSocket socket, int sequence, int timestampMs) {
        auto image = std::make_shared<dai::ImgFrame>();
        image->setType(dai::ImgFrame::Type::RAW8);
        image->setWidth(640);
        image->setHeight(400);
        image->setInstanceNum(static_cast<int>(socket));
        image->setSequenceNum(sequence);
        image->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(timestampMs)));
        image->setData(std::vector<uint8_t>(640 * 400, sequence));
        return image;
    };
    left->send(frame(dai::CameraBoardSocket::CAM_B, 0, 1000));
    left->send(frame(dai::CameraBoardSocket::CAM_B, 1, 1100));
    right->send(frame(dai::CameraBoardSocket::CAM_C, 0, 1100));
    auto actualLeft = receive(syncedLeft);
    auto actualRight = receive(syncedRight);
    REQUIRE(actualLeft->getSequenceNum() == 1);
    REQUIRE(actualRight->getSequenceNum() == 0);
    REQUIRE(actualLeft->getTimestampDevice() == actualRight->getTimestampDevice());
}

TEST_CASE("RVC2 Rectification follows changing input transformations", "[Rectification][ondevice]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC2) SKIP("Requires RVC2");
    auto rectify = pipeline.create<dai::node::Rectification>();
    rectify->setBackend(dai::node::Rectification::Backend::HW);
    auto left = rectify->input1.createInputQueue();
    auto right = rectify->input2.createInputQueue();
    auto outputLeft = rectify->output1.createOutputQueue();
    auto outputRight = rectify->output2.createOutputQueue();
    pipeline.start();

    const std::array<std::array<float, 3>, 3> intrinsic = {{{800, 0, 640}, {0, 800, 400}, {0, 0, 1}}};
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for(int stage = 0; stage < 5; ++stage) {
        CAPTURE(stage);
        auto frame = [&](bool rightSide) {
            dai::ImgTransformation transformation(1280, 800, intrinsic);
            transformation.addCrop(!rightSide && stage > 0 ? 120 : 100, 50, 640, 400);
            dai::Extrinsics pose(identity, {rightSide ? 7.5f : 0.f, rightSide && stage == 3 ? 1.f : 0.f, 0}, dai::CameraBoardSocket::CAM_A);
            pose.specTranslation = {rightSide ? 15.f : 0.f, rightSide ? 3.f : 0.f, 0};
            pose.toDeviceId = "rectification-test-origin";
            if(rightSide && stage == 2) {
                const float angle = 0.08f;
                pose.rotationMatrix = {{std::cos(angle), -std::sin(angle), 0}, {std::sin(angle), std::cos(angle), 0}, {0, 0, 1}};
            }
            transformation.setExtrinsics(pose);
            if(stage == 4) transformation.setDistortionCoefficients({-0.1f, 0, 0, 0, 0, 0, 0, 0});
            auto image = std::make_shared<dai::ImgFrame>();
            image->setType(dai::ImgFrame::Type::RAW8);
            image->setSize(640, 400);
            image->setSourceSize(1280, 800);
            // The supplied geometry must take precedence over any calibration for these sockets.
            image->setInstanceNum(static_cast<int>(rightSide ? dai::CameraBoardSocket::CAM_E : dai::CameraBoardSocket::CAM_D));
            image->setSequenceNum(stage);
            image->setTransformation(transformation);
            std::vector<uint8_t> pixels(640 * 400);
            for(size_t index = 0; index < pixels.size(); ++index) pixels[index] = (index % 640) % 200;
            image->setData(pixels);
            return image;
        };
        auto inputLeft = frame(false);
        left->send(inputLeft);
        right->send(frame(true));
        auto actualLeft = receive(outputLeft);
        auto actualRight = receive(outputRight);
        const auto expectedIntrinsics = inputLeft->getTransformation().getIntrinsicMatrix();
        for(const auto& output : {actualLeft, actualRight}) {
            const auto actualIntrinsics = output->getTransformation().getIntrinsicMatrix();
            REQUIRE(output->getSequenceNum() == stage);
            REQUIRE(output->validateTransformations());
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col) {
                    REQUIRE(actualIntrinsics[row][col] == Catch::Approx(expectedIntrinsics[row][col]).margin(1e-4));
                }
            const auto expectedSourceSize = stage == 0 ? std::make_pair<size_t, size_t>(1280, 800) : std::make_pair<size_t, size_t>(640, 400);
            REQUIRE(output->getTransformation().getSourceSize() == expectedSourceSize);
        }
        if(stage < 2 || stage == 4) {
            const auto value = actualRight->getData()[100 * 640 + 100];
            float expected = stage == 0 ? 100 : 120;
            if(stage == 4) {
                const float x = (100.f - expectedIntrinsics[0][2]) / expectedIntrinsics[0][0];
                const float y = (100.f - expectedIntrinsics[1][2]) / expectedIntrinsics[1][1];
                expected = 800.f * x * (1.f - 0.1f * (x * x + y * y)) + 540.f;
            }
            REQUIRE(static_cast<int>(value) == Catch::Approx(expected).margin(2));
        }
        const auto relative = actualLeft->getTransformation().getExtrinsicsTransformationMatrixTo(actualRight->getTransformation());
        for(int row = 0; row < 3; ++row)
            for(int col = 0; col < 3; ++col) {
                REQUIRE(relative[row][col] == Catch::Approx(row == col ? 1 : 0).margin(1e-4));
            }
        REQUIRE(relative[1][3] == Catch::Approx(0).margin(1e-4));
        REQUIRE(relative[2][3] == Catch::Approx(0).margin(1e-4));
    }
}
