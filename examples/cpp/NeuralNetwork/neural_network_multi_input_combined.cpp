#include <iostream>
#include <opencv2/opencv.hpp>
#include <xtensor/containers/xadapt.hpp>
#include <xtensor/containers/xarray.hpp>

#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"

int main() {
    // Decode the image using OpenCV
    cv::Mat lenaImageCv = cv::imread(LENNA_PATH);
    cv::resize(lenaImageCv, lenaImageCv, cv::Size(256, 256));
    cv::cvtColor(lenaImageCv, lenaImageCv, cv::COLOR_BGR2RGB);

    // Create xt::xarray from cv::Mat
    std::vector<uint8_t> lenaImageData(lenaImageCv.data, lenaImageCv.data + lenaImageCv.total() * lenaImageCv.channels());
    xt::xarray<uint8_t> lenaImage = xt::adapt(lenaImageData);

    // Create pipeline
    dai::Pipeline pipeline;

    // Create model description
    dai::NNModelDescription model;
    model.model = "depthai-test-models/simple-concatenate-model";
    model.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive archive(dai::getModelFromZoo(model));

    // Create and set up nodes
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    neuralNetwork->setNNArchive(archive);
    auto nnDataInputQueue = neuralNetwork->input.createInputQueue();
    auto qNNData = neuralNetwork->out.createOutputQueue();

    // Prepare input data
    auto inputNNData = std::make_shared<dai::NNData>();
    auto platform = pipeline.getDefaultDevice()->getPlatform();

    if(platform == dai::Platform::RVC2) {
        // Transpose to CHW format
        lenaImage = xt::transpose(lenaImage, {2, 0, 1});
        inputNNData->addTensor("image1", lenaImage, dai::TensorInfo::DataType::U8F);
        inputNNData->addTensor("image2", lenaImage, dai::TensorInfo::DataType::U8F);
    } else {
        // Add empty dimension at front (NHWC format)
        lenaImage = xt::expand_dims(lenaImage, 0);
        inputNNData->addTensor("image1", lenaImage, dai::TensorInfo::DataType::FP16);
        inputNNData->addTensor("image2", lenaImage, dai::TensorInfo::DataType::FP16);
    }

    // Start pipeline
    pipeline.start();

    // Main loop
    while(pipeline.isRunning()) {
        nnDataInputQueue->send(inputNNData);
        auto inNNData = qNNData->get<dai::NNData>();
        auto tensor = inNNData->getFirstTensor<float>();
        auto tensor_uint8 = xt::eval(xt::squeeze(xt::cast<uint8_t>(tensor), 0));

        cv::Mat output;
        if(tensor_uint8.shape()[0] == 3) {
            tensor_uint8 = xt::transpose(tensor_uint8, {1, 2, 0});
        }
        output = cv::Mat(tensor_uint8.shape()[0], tensor_uint8.shape()[1], CV_8UC3);
        std::memcpy(output.data, tensor_uint8.data(), tensor_uint8.size());

        cv::imshow("Combined image", output);

        char key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}