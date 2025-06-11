#include <iostream>
#include <opencv2/opencv.hpp>
#include <xtensor/containers/xarray.hpp>

#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"

int main() {
    // Decode the image using OpenCV
    cv::Mat lenaImage = cv::imread(LENNA_PATH);
    cv::resize(lenaImage, lenaImage, cv::Size(256, 256));

    // Create pipeline
    dai::Pipeline pipeline;

    // Create model description
    dai::NNModelDescription model;
    model.model = "depthai-test-models/simple-concatenate-model";
    model.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive archive(dai::getModelFromZoo(model));

    dai::ImgFrame::Type daiType;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        daiType = dai::ImgFrame::Type::RGB888p;
    } else {
        daiType = dai::ImgFrame::Type::RGB888i;
    }

    // Create and set up nodes
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput(std::make_pair(256, 256), daiType);

    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    neuralNetwork->setNNArchive(archive);
    camOut->link(neuralNetwork->inputs["image1"]);

    auto lennaInputQueue = neuralNetwork->inputs["image2"].createInputQueue();
    // No need to send the second image everytime
    neuralNetwork->inputs["image2"].setReusePreviousMessage(true);

    auto qNNData = neuralNetwork->out.createOutputQueue();

    // Stt pipeline
    pipeline.start();
    // Create and set the image frame
    auto daiLenaImage = std::make_shared<dai::ImgFrame>();
    daiLenaImage->setCvFrame(lenaImage, daiType);
    lennaInputQueue->send(daiLenaImage);

    // Main loop
    while(pipeline.isRunning()) {
        auto inNNData = qNNData->get<dai::NNData>();
        auto tensor = inNNData->getFirstTensor<float>();
        auto tensor_uint8 = xt::eval(xt::squeeze(xt::cast<uint8_t>(tensor), 0));

        cv::Mat output;
        if(tensor_uint8.shape()[0] == 3) {
            tensor_uint8 = xt::transpose(tensor_uint8, {1, 2, 0});
        }
        output = cv::Mat(tensor_uint8.shape()[0], tensor_uint8.shape()[1], CV_8UC3);
        std::memcpy(output.data, tensor_uint8.data(), tensor_uint8.size());

        cv::imshow("Combined", output);

        char key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}