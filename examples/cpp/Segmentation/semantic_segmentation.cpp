#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {
cv::Mat showColoredSegmentationFrame(dai::SegmentationMask& segMask, float colorScalingFactor) {
    cv::Mat segFrame = segMask.getCvMask();
    if(segFrame.empty()) {
        return {};
    }

    if(segFrame.type() != CV_8UC1) {
        cv::Mat segFrameU8;
        segFrame.convertTo(segFrameU8, CV_8UC1);
        segFrame = segFrameU8;
    }

    cv::Mat scaledAll;
    segFrame.convertTo(scaledAll, CV_32F);
    scaledAll *= colorScalingFactor;
    scaledAll.convertTo(scaledAll, CV_8UC1);

    cv::Mat scaledMask = segFrame.clone();
    scaledAll.copyTo(scaledMask, segFrame < 255);

    cv::Mat coloredSegFrame;
    cv::applyColorMap(scaledMask, coloredSegFrame, cv::COLORMAP_JET);
    coloredSegFrame.setTo(cv::Scalar(0, 0, 0), segFrame == 255);

    const auto labels = segMask.getLabels();
    if(!labels.empty()) {
        auto uniqueIndices = segMask.getUniqueIndices();
        for(int idx = 0; idx < uniqueIndices.size(); ++idx) {
            const std::string& label = labels[uniqueIndices[idx]];
            uint8_t scaledValue = cv::saturate_cast<uint8_t>(static_cast<int>(uniqueIndices[idx] * colorScalingFactor));
            cv::Mat colorValue(1, 1, CV_8UC1, cv::Scalar(scaledValue));
            cv::Mat textColorMat;
            cv::applyColorMap(colorValue, textColorMat, cv::COLORMAP_JET);
            auto textColor = textColorMat.at<cv::Vec3b>(0, 0);

            cv::putText(coloredSegFrame,
                        std::to_string(static_cast<int>(uniqueIndices[idx])) + ": " + label,
                        cv::Point(10, 20 + idx * 20),
                        cv::FONT_HERSHEY_TRIPLEX,
                        0.5,
                        cv::Scalar(textColor[0], textColor[1], textColor[2]),
                        1);
        }
    }

    return coloredSegFrame;
}
}  // namespace

int main() {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline{device};
    std::string modelName = "luxonis/deeplab-v3-plus:512x512";
    if(device && device->getPlatform() == dai::Platform::RVC2) {
        modelName = "luxonis/deeplab-v3-plus:person-256x256";
    }

    std::cout << "Creating pipeline..." << std::endl;

    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    auto neuralNetwork = pipeline.create<dai::node::NeuralNetwork>();
    neuralNetwork->build(cameraNode, modelName);

    auto segParser = pipeline.create<dai::node::SegmentationParser>();
    segParser->build(neuralNetwork->out, modelName);

    auto maskQueue = segParser->out.createOutputQueue();
    auto frameQueue = neuralNetwork->passthrough.createOutputQueue();
    auto configQueue = segParser->inputConfig.createInputQueue();
    auto config = segParser->initialConfig;
    const auto labels = segParser->getLabels();
    const float colorScalingFactor = 255.0f / (static_cast<float>(labels.size()) + 1.0f);

    pipeline.start();
    std::cout << "Pipeline created." << std::endl;
    std::cout << "Controls: 'l' increase threshold, 'j' decrease threshold, 'q' quit." << std::endl;

    auto startTime = std::chrono::steady_clock::now();
    int frames = 0;

    while(pipeline.isRunning()) {
        auto outSegMask = maskQueue->get<dai::SegmentationMask>();
        auto frameMsg = frameQueue->get<dai::ImgFrame>();

        cv::Mat frame = frameMsg->getCvFrame();
        cv::Mat coloredSegFrame = cv::Mat::zeros(frame.size(), frame.type());

        cv::Mat segColored = showColoredSegmentationFrame(*outSegMask, colorScalingFactor);
        if(!segColored.empty()) {
            coloredSegFrame = segColored;
        }
        cv::imshow("Segmentation Mask", coloredSegFrame);

        cv::Mat copyColoredFrame = coloredSegFrame.clone();
        if(!copyColoredFrame.empty()) {
            cv::Mat mask = copyColoredFrame == 255;
            std::vector<cv::Mat> maskChannels;
            std::vector<cv::Mat> coloredChannels;
            std::vector<cv::Mat> frameChannels;
            cv::split(mask, maskChannels);
            cv::split(copyColoredFrame, coloredChannels);
            cv::split(frame, frameChannels);
            for(size_t i = 0; i < coloredChannels.size() && i < frameChannels.size(); ++i) {
                frameChannels[i].copyTo(coloredChannels[i], maskChannels[i]);
            }
            cv::merge(coloredChannels, copyColoredFrame);
        }
        cv::addWeighted(frame, 0.5, copyColoredFrame, 0.5, 0, frame);

        ++frames;
        auto now = std::chrono::steady_clock::now();
        float fps = frames / std::max(1e-6f, std::chrono::duration<float>(now - startTime).count());
        std::ostringstream fpsStream;
        fpsStream << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, "NN fps: " + fpsStream.str(), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255));
        cv::imshow("Frame", frame);

        auto key = cv::waitKey(1) & 0xFF;
        if(key == 'q') {
            pipeline.stop();
            break;
        } else if(key == 'l') {
            float current = config->getConfidenceThreshold();
            config->setConfidenceThreshold(current + 0.1f);
            configQueue->send(config);
            std::cout << "Increased confidence threshold to " << std::fixed << std::setprecision(2) << config->getConfidenceThreshold() << std::endl;
        } else if(key == 'j') {
            float current = config->getConfidenceThreshold();
            config->setConfidenceThreshold(current - 0.1f);
            configQueue->send(config);
            std::cout << "Decreased confidence threshold to " << std::fixed << std::setprecision(2) << config->getConfidenceThreshold() << std::endl;
        }
    }

    return 0;
}
