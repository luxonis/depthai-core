#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/node/SegmentationParser.hpp"

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
    modelDescription.model = "luxonis/pp-liteseg:1024x512:436ba4d";
    modelDescription.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    dai::NNArchive archive = dai::NNArchive(dai::getModelFromZoo(modelDescription));
    neuralNetwork->build(cameraNode, modelDescription);

    // auto segParser = pipeline.create<dai::node::SegmentationParser>()->build(neuralNetwork->out, archive.getHeadConfig(0));
    auto segParser = pipeline.create<dai::node::SegmentationParser>()->build(neuralNetwork->out, archive);
    // neuralNetwork->out.link(segParser->input);
    segParser->setRunOnHost(true);
    segParser->initialConfig->setConfidenceThreshold(0.0f);

    auto outQ = segParser->out.createOutputQueue();
    auto frameQ = neuralNetwork->passthrough.createOutputQueue();

    auto configInputQ = segParser->inputConfig.createInputQueue();
    auto config = segParser->initialConfig;

    // Start pipeline
    printf("Starting pipeline...\n");
    pipeline.start();

    // Build a fixed color palette (19 classes + background at index 255)
    std::vector<cv::Vec3b> palette = {
        {0, 0, 0},     {0, 0, 255},   {0, 255, 0},   {255, 0, 0},  {0, 255, 255}, {255, 0, 255}, {255, 255, 0}, {128, 0, 0},  {0, 128, 0},  {0, 0, 128},
        {128, 128, 0}, {128, 0, 128}, {0, 128, 128}, {192, 64, 0}, {64, 192, 0},  {0, 192, 64},  {192, 0, 64},  {64, 0, 192}, {0, 64, 192},
    };
    cv::Mat lutB(1, 256, CV_8UC1, cv::Scalar(0));
    cv::Mat lutG(1, 256, CV_8UC1, cv::Scalar(0));
    cv::Mat lutR(1, 256, CV_8UC1, cv::Scalar(0));
    for(size_t i = 0; i < palette.size(); ++i) {
        lutB.at<uint8_t>(0, static_cast<int>(i)) = palette[i][0];
        lutG.at<uint8_t>(0, static_cast<int>(i)) = palette[i][1];
        lutR.at<uint8_t>(0, static_cast<int>(i)) = palette[i][2];
    }
    // background value 255 stays black; adjust here if a different background color is desired

    // Main loop
    while(pipeline.isRunning()) {
        auto outSegMask = outQ->get<dai::SegmentationMask>();
        auto frame = frameQ->get<dai::ImgFrame>();
        auto key = cv::waitKey(1);

        std::optional<cv::Mat> segMaskCV = outSegMask->getCvMask();
        if(segMaskCV) {  // valid mask returned
            cv::Mat maskU8;
            if(segMaskCV->type() != CV_8UC1) {
                segMaskCV->convertTo(maskU8, CV_8UC1);
            } else {
                maskU8 = *segMaskCV;
            }
            cv::Mat b, g, r;
            cv::LUT(maskU8, lutB, b);
            cv::LUT(maskU8, lutG, g);
            cv::LUT(maskU8, lutR, r);
            cv::Mat channels[] = {b, g, r};
            cv::Mat displayMask;
            cv::merge(channels, 3, displayMask);
            cv::imshow("Segmentation Mask Colored", displayMask);
        }
        cv::imshow("Frame", frame->getCvFrame());
        if(key == 'k') {
            float currentThr = segParser->inConfig->getConfidenceThreshold();
            config->setConfidenceThreshold(std::min(1.0f, currentThr + 0.1f));
            configInputQ->send(config);
            std::cout << "Increased confidence threshold to " << segParser->inConfig->getConfidenceThreshold() << std::endl;
        } else if(key == 'l') {
            float currentThr = segParser->inConfig->getConfidenceThreshold();
            config->setConfidenceThreshold(std::max(0.0f, currentThr - 0.1f));
            configInputQ->send(config);
            std::cout << "Decreased confidence threshold to " << segParser->inConfig->getConfidenceThreshold() << std::endl;
            break;
        } else if(key == 'q') {
            break;
        }
        if(auto labels = outSegMask->getLabels()) {
            std::cout << "Labels: ";
            for(std::size_t i = 0; i < labels->size(); ++i) {
                if(i > 0) {
                    std::cout << ", ";
                }
                std::cout << labels->at(i);
            }
            std::cout << std::endl;
        }
    }
    return 0;
}
