#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/superanimal-landmarker:256x256";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::SuperAnimalParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: '+' increase score threshold, '-' decrease it, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::beta::Keypoints>();
        cv::Mat frame = frameMessage->getCvFrame();
        std::vector<cv::Point> points;
        for(const auto& point : parserOutput->getPoints2f()) {
            points.emplace_back(static_cast<int>(point.x * frame.cols), static_cast<int>(point.y * frame.rows));
        }
        for(const auto& edge : parserOutput->getEdges()) {
            cv::line(frame, points.at(edge[0]), points.at(edge[1]), cv::Scalar(0, 255, 0), 2);
        }
        for(const auto& point : points) cv::circle(frame, point, 3, cv::Scalar(0, 0, 255), -1);

        cv::imshow("SuperAnimalParser", frame);
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
