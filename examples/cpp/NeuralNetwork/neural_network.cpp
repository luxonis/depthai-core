#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create nodes
    auto cameraNode = pipeline.create<dai::node::Camera>();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();

    // Build nodes
    cameraNode->build();
    // Longer form - useful in case of a local NNArchive
    // auto modelDescription = dai::NNModelDescription("yolov6-nano", pipeline.getDefaultDevice()->getPlatformAsString());
    // auto archive = dai::NNArchive(dai::getModelFromZoo(modelDescription));
    // neuralNetwork->build(cameraNode, archive);
    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    neuralNetwork->build(cameraNode, modelDescription);

    // Create output queue
    auto qNNData = neuralNetwork->out.createOutputQueue();

    // Start pipeline
    pipeline.start();

    // Main loop
    while(pipeline.isRunning()) {
        auto inNNData = qNNData->get<dai::NNData>();
        auto tensor = inNNData->getFirstTensor<float>();
        std::cout << "Received NN data: " << tensor.shape()[0] << "x" << tensor.shape()[1] << std::endl;
    }
    return 0;
}