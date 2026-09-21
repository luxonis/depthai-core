#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    const std::string modelSlug = "luxonis/paddle-text-detection:256x256";
    dai::NNModelDescription modelDescription{modelSlug};
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive modelArchive(dai::getModelFromZoo(modelDescription));

    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>()->build(cameraNode, modelArchive);
    auto parserNode = pipeline.create<dai::beta::node::PPTextDetectionParser>()->build(neuralNetwork->out, modelArchive);

    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto outputQueue = parserNode->out.createOutputQueue();
    auto configQueue = parserNode->inputConfig.createInputQueue();
    auto config = parserNode->initialConfig;

    pipeline.start();
    std::cout << "Controls: '+' increase confidence threshold, '-' decrease it, 'q' quit." << std::endl;

    while(pipeline.isRunning()) {
        auto frameMessage = frameQueue->get<dai::ImgFrame>();
        auto parserOutput = outputQueue->get<dai::ImgDetections>();
        cv::Mat frame = frameMessage->getCvFrame();
        for(const auto& detection : parserOutput->detections) {
            const auto boundingBox = detection.getBoundingBox().denormalize(frame.cols, frame.rows);
            std::vector<cv::Point> points;
            for(const auto& point : boundingBox.getPoints()) points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
            cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);

            const std::string label = detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName;
            std::ostringstream text;
            text << label << ": " << std::fixed << std::setprecision(2) << detection.confidence;
            cv::putText(frame, text.str(), points.front(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            for(const auto& point : detection.getKeypoints2f()) {
                cv::circle(frame, cv::Point(static_cast<int>(point.x * frame.cols), static_cast<int>(point.y * frame.rows)), 3, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::imshow("PPTextDetectionParser", frame);
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
