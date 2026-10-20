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

    const std::string modelSlug = "luxonis/dm-count:sha-426x240";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::MapOutputParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: 't' toggle min/max scaling, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Map2D>();
        cv::Mat frame = frameMessage->getCvFrame();
        const auto mapValues = parserOutput->getMap();
        cv::Mat map(static_cast<int>(parserOutput->getHeight()), static_cast<int>(parserOutput->getWidth()), CV_32F, const_cast<float*>(mapValues.data()));
        cv::Mat normalizedMap;
        cv::normalize(map, normalizedMap, 0, 255, cv::NORM_MINMAX);
        normalizedMap.convertTo(normalizedMap, CV_8U);
        cv::applyColorMap(normalizedMap, frame, cv::COLORMAP_INFERNO);
        cv::resize(frame, frame, frameMessage->getCvFrame().size());

        cv::imshow("MapOutputParser", frame);
        const int key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == 't') {
            config->minMaxScaling = !config->minMaxScaling;
            configQueue->send(config);
            std::cout << "Min/max scaling: " << config->minMaxScaling << std::endl;
        }
    }

    return 0;
}
