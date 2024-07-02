#include <iostream>
#include <string>
#include <chrono>

#include "depthai/depthai.hpp"

int main(int argc, char* argv[]) {

    dai::Pipeline pipeline(true);

    // Download model from zoo
    dai::NNModelDescription modelDescription;
    modelDescription.modelSlug = "ales-test";
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    std::string modelPath = dai::getModelFromZoo(modelDescription, true);  // True means use cached model if available
    std::cout << "Model path: " << modelPath << std::endl;

    // Color camera node
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(256, 256);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setInterleaved(false);

    // Neural network node
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    neuralNetwork->setBlobPath(modelPath);
    neuralNetwork->setNumInferenceThreads(2);

    // Linking
    camRgb->preview.link(neuralNetwork->input);

    auto nnDetectionQueue = neuralNetwork->out.createOutputQueue();
    auto nnPassthroughQueue = neuralNetwork->passthrough.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {

        auto detection = nnDetectionQueue->get<dai::NNData>();
        auto passthrough = nnPassthroughQueue->get<dai::ImgFrame>();

        // Do something with the data
        // ...

        std::cout << "Detected" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return EXIT_SUCCESS;
}