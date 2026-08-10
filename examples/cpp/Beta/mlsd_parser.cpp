#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/m-lsd:512x512";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::MLSDParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: '+' increase score threshold, '-' decrease it, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Lines>();
        cv::Mat frame = frameMessage->getCvFrame();
        for(const auto& line : parserOutput->lines) {
            const cv::Point startPoint(static_cast<int>(line.startPoint.x * frame.cols), static_cast<int>(line.startPoint.y * frame.rows));
            const cv::Point endPoint(static_cast<int>(line.endPoint.x * frame.cols), static_cast<int>(line.endPoint.y * frame.rows));
            cv::line(frame, startPoint, endPoint, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("MLSDParser", frame);
        const int key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == '+') {
            config->scoreThreshold = std::min(1.0f, config->scoreThreshold + 0.1f);
            configQueue->send(config);
            std::cout << "Score threshold: " << config->scoreThreshold << std::endl;
        } else if(key == '-') {
            config->scoreThreshold = std::max(0.0f, config->scoreThreshold - 0.1f);
            configQueue->send(config);
            std::cout << "Score threshold: " << config->scoreThreshold << std::endl;
        }
    }

    return 0;
}
