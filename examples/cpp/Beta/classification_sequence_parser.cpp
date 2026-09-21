#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/paddle-text-recognition:320x48";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::ClassificationSequenceParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: 't' toggle remove duplicates, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Classifications>();
        cv::Mat frame = frameMessage->getCvFrame();
        const bool characterSequence =
            std::all_of(parserOutput->classes.begin(), parserOutput->classes.end(), [](const auto& label) { return label.size() <= 1; });
        std::ostringstream decodedText;
        for(std::size_t index = 0; index < parserOutput->classes.size(); ++index) {
            if(index > 0 && !characterSequence) decodedText << ' ';
            decodedText << parserOutput->classes[index];
        }
        cv::putText(frame, decodedText.str(), cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);

        cv::imshow("ClassificationSequenceParser", frame);
        const int key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == 't') {
            config->removeDuplicates = !config->removeDuplicates;
            configQueue->send(config);
            std::cout << "Remove duplicates: " << config->removeDuplicates << std::endl;
        }
    }

    return 0;
}
