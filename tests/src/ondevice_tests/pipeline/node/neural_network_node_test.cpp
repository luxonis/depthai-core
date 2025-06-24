#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <magic_enum/magic_enum.hpp>
#include <opencv2/videoio.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
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
    auto description = dai::NNModelDescription{"depthai-test-models/simple-concatenate-model", platformStr};
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
    auto passThroughQueue = nn->passthroughs["image2"].createOutputQueue();

    // Start the pipeline
    p.start();

    // Send the Lenna image to the second input queue
    lennaInputQueue->send(daiLenaImage);

    // Process output for 10 tensors and verify results
    for(int i = 0; i < 10; i++) {
        auto tensor = outputQueue->get<dai::NNData>();
        auto passThroughTensor = passThroughQueue->get<dai::ImgFrame>();

        REQUIRE(tensor != nullptr);
        REQUIRE(tensor->getAllLayerNames().size() == 1);
        auto firstTensor = tensor->getFirstTensor<float>();
        REQUIRE(firstTensor.shape().size() == 4);
        REQUIRE(firstTensor.shape()[0] == 1);

        // Verify the pass-through tensor came through
        REQUIRE(passThroughTensor != nullptr);
    }
}

TEST_CASE("Combined Input NeuralNetwork API") {
    dai::Pipeline p;

    // Convert image to appropriate format based on platform
    auto device = p.getDefaultDevice();
    auto platform = device->getPlatform();
    dai::TensorInfo::DataType nnTensorType = dai::TensorInfo::DataType::FP16;

    if(platform == dai::Platform::RVC2 || platform == dai::Platform::RVC3) {
        nnTensorType = dai::TensorInfo::DataType::U8F;
    } else if(platform == dai::Platform::RVC4) {
        nnTensorType = dai::TensorInfo::DataType::FP16;
    } else {
        FAIL("Unknown platform");
    }

    // Create NNData and add tensors
    auto inputNNData = std::make_shared<dai::NNData>();
    inputNNData->addTensor("image1", xt::xarray<uint8_t>({1, 256, 256, 3}, 0), nnTensorType);
    inputNNData->addTensor("image2", xt::xarray<uint8_t>({1, 256, 256, 3}, 100), nnTensorType);

    // Set up the neural network node
    auto description = dai::NNModelDescription{"depthai-test-models/simple-concatenate-model", device->getPlatformAsString()};
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setModelPath(dai::getModelFromZoo(description));

    // Create input and output queues
    auto nnDataInputQueue = nn->input.createInputQueue();
    auto outputQueue = nn->out.createOutputQueue();

    // Start the pipeline
    p.start();

    // Send the input data
    nnDataInputQueue->send(inputNNData);

    // Process output and verify results
    auto tensor = outputQueue->get<dai::NNData>();
    REQUIRE(tensor != nullptr);
    REQUIRE(tensor->getAllLayerNames().size() == 1);

    // Retrieve the first tensor and process it
    auto firstTensor = tensor->getFirstTensor<float>(dai::TensorInfo::StorageOrder::NHWC);
    auto tensorShape = firstTensor.shape();
    REQUIRE(tensorShape.size() == 4);
    REQUIRE(tensorShape[0] == 1);
    REQUIRE(tensorShape[1] == 256);
    REQUIRE(tensorShape[2] == 512);
    REQUIRE(tensorShape[3] == 3);

    // Iterate over the tensor and verify the values
    bool leftSideOK = true;
    for(int i = 0; i < tensorShape[1]; i++) {
        for(int j = 0; j < 256; j++) {
            for(int k = 0; k < tensorShape[3]; k++) {
                float val = firstTensor(0, i, j, k);
                if(val > 0.1) {
                    leftSideOK = false;
                    FAIL(std::string("Left side is not OK") + std::to_string(val));
                    break;
                }
            }
        }
    }

    REQUIRE(leftSideOK);

    bool rightSideOK = true;
    for(int i = 0; i < tensorShape[1]; i++) {
        for(int j = 256; j < 512; j++) {
            for(int k = 0; k < tensorShape[3]; k++) {
                float val = firstTensor(0, i, j, k);
                if(val < 99.9) {
                    rightSideOK = false;
                    FAIL(std::string("Right side is not OK") + std::to_string(val));
                    break;
                }
            }
        }
    }

    REQUIRE(rightSideOK);
}

TEST_CASE("Multi threaded test") {
    // Create pipeline
    dai::Pipeline p;
    auto benchmarkOut = p.create<dai::node::BenchmarkOut>();
    benchmarkOut->setFps(-1);  // As fast as possible
    auto inputBenchmarkQueue = benchmarkOut->input.createInputQueue();
    auto modelPath = dai::getModelFromZoo(dai::NNModelDescription{"yolov6-nano", p.getDefaultDevice()->getPlatformAsString()});
    auto modelArchive = dai::NNArchive(modelPath);
    auto inputSize = modelArchive.getInputSize();
    auto type = modelArchive.getConfig<dai::nn_archive::v1::Config>().model.inputs[0].preprocessing.daiType;
    auto daiType = dai::ImgFrame::Type::BGR888p;

    if(type.has_value()) {
        auto convertedInputType = magic_enum::enum_cast<dai::ImgFrame::Type>(type.value());
        if(!convertedInputType.has_value()) {
            FAIL("Unsupported type");
        } else {
            daiType = convertedInputType.value();
        }
    } else {
        if(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2 || p.getDefaultDevice()->getPlatform() == dai::Platform::RVC3) {
            daiType = dai::ImgFrame::Type::BGR888p;
        } else if(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC4) {
            daiType = dai::ImgFrame::Type::BGR888i;
        } else {
            FAIL("Unsupported platform");
        }
    }
    auto nn = p.create<dai::node::NeuralNetwork>()->build(benchmarkOut->out, modelArchive);
    nn->setNumInferenceThreads(2);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    nn->out.link(benchmarkIn->input);
    benchmarkIn->sendReportEveryNMessages(100);
    benchmarkIn->logReportsAsWarnings(false);
    auto outputQueue = benchmarkIn->report.createOutputQueue();
    // Start pipeline
    p.start();

    auto platform = p.getDefaultDevice()->getPlatform();
    auto reportsToGet = 0;
    if(platform == dai::Platform::RVC2) {
        reportsToGet = 100;  // Run the test for ~3 minutes
    } else if(platform == dai::Platform::RVC4) {
        reportsToGet = 1000;  // Run the test for ~3 minutes
    } else {
        FAIL("Unknown platform");
    }

    auto inputFrame = std::make_shared<dai::ImgFrame>();
    if(!inputSize.has_value()) {
        FAIL("Input size not available");
    }
    cv::Mat cvFrame = cv::Mat(inputSize->second, inputSize->first, CV_8UC3, cv::Scalar(0, 255, 0));
    inputFrame->setCvFrame(cvFrame, daiType);
    inputBenchmarkQueue->send(inputFrame);
    for(int i = 0; i < reportsToGet; i++) {
        auto report = outputQueue->get<dai::BenchmarkReport>();
        REQUIRE(report != nullptr);
    }
}
