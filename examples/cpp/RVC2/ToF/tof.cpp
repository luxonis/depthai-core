#include "depthai/depthai.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>

cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    cv::Mat invalidMask = (frameDepth == 0);
    cv::Mat depthFrameColor;
    
    try {
        // Create a mask for non-zero values
        cv::Mat nonZeroMask = (frameDepth != 0);
        
        // Calculate percentiles for non-zero values
        std::vector<float> nonZeroValues;
        for(int i = 0; i < frameDepth.rows; i++) {
            for(int j = 0; j < frameDepth.cols; j++) {
                if(nonZeroMask.at<uchar>(i, j)) {
                    nonZeroValues.push_back(frameDepth.at<float>(i, j));
                }
            }
        }
        std::sort(nonZeroValues.begin(), nonZeroValues.end());
        
        float minDepth = nonZeroValues[nonZeroValues.size() * 0.03];
        float maxDepth = nonZeroValues[nonZeroValues.size() * 0.95];
        
        // Calculate log depth
        cv::Mat logDepth;
        frameDepth.convertTo(logDepth, CV_32F);
        for(int i = 0; i < logDepth.rows; i++) {
            for(int j = 0; j < logDepth.cols; j++) {
                if(nonZeroMask.at<uchar>(i, j)) {
                    logDepth.at<float>(i, j) = std::log(logDepth.at<float>(i, j));
                }
            }
        }
        
        float logMinDepth = std::log(minDepth);
        float logMaxDepth = std::log(maxDepth);
        
        // Clip values
        cv::threshold(logDepth, logDepth, logMaxDepth, logMaxDepth, cv::THRESH_TRUNC);
        cv::threshold(logDepth, logDepth, logMinDepth, logMinDepth, cv::THRESH_TOZERO);
        
        // Interpolate to 0-255 range
        cv::Mat depthFrameColor8U;
        logDepth.convertTo(depthFrameColor8U, CV_8U, 255.0/(logMaxDepth - logMinDepth), -logMinDepth * 255.0/(logMaxDepth - logMinDepth));
        
        // Apply color map
        cv::applyColorMap(depthFrameColor8U, depthFrameColor, cv::COLORMAP_JET);
        
        // Set invalid pixels to black
        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);
        
    } catch(const std::exception& e) {
        // Frame is likely empty or error occurred
        depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }
    
    return depthFrameColor;
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;
    
    // Define source and output
    auto tof = pipeline.create<dai::node::ToF>()->build();
    auto depthQueue = tof->depth.createOutputQueue();
    
    // Start pipeline
    pipeline.start();
    
    // Main loop
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::Mat depthFrame = depth->getCvFrame();
        cv::Mat visualizedDepth = colorizeDepth(depthFrame);
        
        cv::imshow("depth", visualizedDepth);
        
        char key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
    
    return 0;
} 