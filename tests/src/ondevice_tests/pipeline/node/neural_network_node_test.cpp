#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <opencv2/videoio.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/NNModelDescription.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

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
    std::string platform = p.getDefaultDevice()->getPlatformAsString();
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

TEST_CASE("Multi-Input NeuralNetwork API") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto platformStr = p.getDefaultDevice()->getPlatformAsString();
    auto platform = p.getDefaultDevice()->getPlatform();
    auto inputType = dai::ImgFrame::Type::RGB888p;
    if(platform == dai::Platform::RVC2 || platform == dai::Platform::RVC3) {
        inputType = dai::ImgFrame::Type::BGR888p;
    } else if(platform == dai::Platform::RVC4) {
        inputType = dai::ImgFrame::Type::BGR888i;
    } else {
        FAIL("Unknown platform");
    }
    auto description = dai::NNModelDescription{"simple-concatenate-model", platformStr};
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setModelPath(dai::getModelFromZoo(description));

    auto* cameraInput = camera->requestOutput(std::make_pair(256, 256), inputType);
    cameraInput->link(nn->inputs["image1"]);
    auto lennaInputQueue = nn->inputs["image2"].createInputQueue();

    // Load and prepare Lenna image (assuming path provided in a macro IMAGE_PATH)
    cv::Mat lenaImage = cv::imread(LENNA_PATH, cv::IMREAD_COLOR);
    REQUIRE(!lenaImage.empty());  // Ensure the image is loaded correctly
    cv::resize(lenaImage, lenaImage, cv::Size(256, 256));

    // Convert the image to dai::ImgFrame
    auto daiLenaImage = std::make_shared<dai::ImgFrame>();
    daiLenaImage->setCvFrame(lenaImage, inputType);

    // Create output queue
    auto outputQueue = nn->out.createOutputQueue();

    // Reuse the second input image to avoid sending every time
    nn->inputs["image2"].setReusePreviousMessage(true);

    // Start the pipeline
    p.start();

    // Send the Lenna image to the second input queue
    lennaInputQueue->send(daiLenaImage);

    // Process output for 10 tensors and verify results
    for(int i = 0; i < 10; i++) {
        auto tensor = outputQueue->get<dai::NNData>();

        REQUIRE(tensor != nullptr);
        REQUIRE(tensor->getAllLayerNames().size() == 1);
        auto firstTensor = tensor->getFirstTensor<float>();
        REQUIRE(firstTensor.shape().size() == 4);
        REQUIRE(firstTensor.shape()[0] == 1);
    }
}
