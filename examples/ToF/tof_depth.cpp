#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <depthai/depthai.hpp>
#include <vector>
#include <string>
#include "depthai-shared/datatype/RawToFConfig.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"

constexpr auto SHAPE = 720;

std::shared_ptr<dai::ToFConfig> createConfig(dai::RawToFConfig configRaw) {
    auto config = std::make_shared<dai::ToFConfig>();
    config->set(std::move(configRaw));
    return config;
}

cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    cv::Mat invalidMask = (frameDepth == 0);
    cv::Mat depthFrameColor;
    try {
        double minDepth = 0.0;
        double maxDepth = 0.0;
        cv::minMaxIdx(frameDepth, &minDepth, &maxDepth, nullptr, nullptr, ~invalidMask);
        if(minDepth == maxDepth) {
            depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
            return depthFrameColor;
        }
        cv::Mat logDepth;
        frameDepth.convertTo(logDepth, CV_32F);
        cv::log(logDepth, logDepth);
        logDepth.setTo(log(minDepth), invalidMask);
        cv::normalize(logDepth, logDepth, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(logDepth, depthFrameColor, cv::COLORMAP_JET);
        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);
    } catch(const std::exception& e) {
        depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }
    return depthFrameColor;
}

int main() {
    dai::Pipeline pipeline;
    cv::Mat cvColorMap;

    auto tof = pipeline.create<dai::node::ToF>();

    // Configure the ToF node
    auto tofConfig = tof->initialConfig.get();

    tofConfig.enableOpticalCorrection = true;
    tofConfig.enablePhaseShuffleTemporalFilter = true;
    tofConfig.phaseUnwrappingLevel = 4;
    tofConfig.phaseUnwrapErrorThreshold = 300;

    tofConfig.enableTemperatureCorrection = false; // Not yet supported

    auto xinTofConfig = pipeline.create<dai::node::XLinkIn>();
    xinTofConfig->setStreamName("tofConfig");
    xinTofConfig->out.link(tof->inputConfig);

    tof->initialConfig.set(tofConfig);

    auto camTof = pipeline.create<dai::node::Camera>();
    camTof->setFps(30);
    camTof->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camTof->raw.link(tof->input);

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("depth");
    tof->depth.link(xout->input);

    tofConfig = tof->initialConfig.get();


    dai::Device device(pipeline);
    std::cout << "Connected cameras: " << device.getConnectedCameraFeatures().size() << std::endl;
    auto qDepth = device.getOutputQueue("depth");

    auto tofConfigInQueue = device.getInputQueue("tofConfig");

    int counter = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        int key = cv::waitKey(1);
        if (key == 'f') {
            tofConfig.enableFPPNCorrection = !tofConfig.enableFPPNCorrection;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == 'o') {
            tofConfig.enableOpticalCorrection = !tofConfig.enableOpticalCorrection;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == 'w') {
            tofConfig.enableWiggleCorrection = !tofConfig.enableWiggleCorrection;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == 't') {
            tofConfig.enableTemperatureCorrection = !tofConfig.enableTemperatureCorrection;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == 'q') {
            break;
        } else if (key == '0') {
            tofConfig.enablePhaseUnwrapping = false;
            tofConfig.phaseUnwrappingLevel = 0;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == '1') {
            tofConfig.enablePhaseUnwrapping = true;
            tofConfig.phaseUnwrappingLevel = 1;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == '2') {
            tofConfig.enablePhaseUnwrapping = true;
            tofConfig.phaseUnwrappingLevel = 2;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == '3') {
            tofConfig.enablePhaseUnwrapping = true;
            tofConfig.phaseUnwrappingLevel = 3;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == '4') {
            tofConfig.enablePhaseUnwrapping = true;
            tofConfig.phaseUnwrappingLevel = 4;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == '5') {
            tofConfig.enablePhaseUnwrapping = true;
            tofConfig.phaseUnwrappingLevel = 5;
            tofConfigInQueue->send(createConfig(tofConfig));
        } else if (key == 'm') {
            std::vector<dai::MedianFilter> medianSettings = { dai::MedianFilter::MEDIAN_OFF, dai::MedianFilter::KERNEL_3x3, dai::MedianFilter::KERNEL_5x5, dai::MedianFilter::KERNEL_7x7 };
            auto currentMedian = tofConfig.median;
            auto nextMedian = medianSettings[(std::find(medianSettings.begin(), medianSettings.end(), currentMedian) - medianSettings.begin() + 1) % medianSettings.size()];
            tofConfig.median = nextMedian;
            tofConfigInQueue->send(createConfig(tofConfig));
        }

        auto imgFrame = qDepth->get<dai::ImgFrame>(); // blocking call, will wait until new data has arrived
        auto depthColorized = colorizeDepth(imgFrame->getFrame());

        cv::imshow("Colorized depth", depthColorized);
        counter++;
    }

    return 0;
}
