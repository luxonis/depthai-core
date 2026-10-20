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

    const std::string modelSlug = "luxonis/fastsam-s:512x288";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::FastSAMParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: '+' increase confidence threshold, '-' decrease it, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::SegmentationMask>();
        cv::Mat frame = frameMessage->getCvFrame();
        cv::Mat mask = parserOutput->getCvMask();
        cv::resize(mask, mask, frame.size(), 0.0, 0.0, cv::INTER_NEAREST);
        cv::Mat scaledMask;
        mask.convertTo(scaledMask, CV_8U, 37.0);
        cv::Mat coloredMask;
        cv::applyColorMap(scaledMask, coloredMask, cv::COLORMAP_TURBO);
        coloredMask.setTo(cv::Scalar(0, 0, 0), mask == 255);
        cv::addWeighted(frame, 0.6, coloredMask, 0.4, 0.0, frame);

        cv::imshow("FastSAMParser", frame);
        const int key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == '+') {
            config->confidenceThreshold = std::min(1.0f, config->confidenceThreshold + 0.1f);
            configQueue->send(config);
            std::cout << "Confidence threshold: " << config->confidenceThreshold << std::endl;
        } else if(key == '-') {
            config->confidenceThreshold = std::max(0.0f, config->confidenceThreshold - 0.1f);
            configQueue->send(config);
            std::cout << "Confidence threshold: " << config->confidenceThreshold << std::endl;
        }
    }

    return 0;
}
