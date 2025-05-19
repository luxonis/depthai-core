#include <iostream>
#include <chrono>
#include "depthai/depthai.hpp"
#include <opencv2/opencv.hpp>

// Helper function to colorize depth frame
cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    cv::Mat depthFrameColor;
    try {
        // Create invalid depth mask
        cv::Mat invalidMask = (frameDepth == 0);
        
        // Calculate min and max depth using percentiles
        std::vector<float> nonZeroDepth;
        nonZeroDepth.reserve(frameDepth.rows * frameDepth.cols);
        for(int i = 0; i < frameDepth.rows; i++) {
            for(int j = 0; j < frameDepth.cols; j++) {
                float depth = frameDepth.at<float>(i, j);
                if(depth > 0) nonZeroDepth.push_back(depth);
            }
        }
        
        if(nonZeroDepth.empty()) {
            return cv::Mat::zeros(frameDepth.size(), CV_8UC3);
        }
        
        std::sort(nonZeroDepth.begin(), nonZeroDepth.end());
        float minDepth = nonZeroDepth[static_cast<int>(nonZeroDepth.size() * 0.03)]; // 3rd percentile
        float maxDepth = nonZeroDepth[static_cast<int>(nonZeroDepth.size() * 0.95)]; // 95th percentile
        
        // Convert to log scale and normalize
        cv::Mat logDepth;
        cv::log(frameDepth, logDepth);
        float logMinDepth = std::log(minDepth);
        float logMaxDepth = std::log(maxDepth);
        
        // Normalize to 0-255 range
        cv::Mat normalized;
        logDepth = cv::max(logDepth, logMinDepth);
        logDepth = cv::min(logDepth, logMaxDepth);
        cv::normalize(logDepth, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        
        // Apply colormap
        cv::applyColorMap(normalized, depthFrameColor, cv::COLORMAP_JET);
        
        // Set invalid pixels to black
        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);
        
    } catch(const std::exception& e) {
        std::cerr << "Error in colorizeDepth: " << e.what() << std::endl;
        depthFrameColor = cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }
    
    return depthFrameColor;
}

// Helper function to display frame with detections
void displayFrame(const std::string& name, 
                 dai::ImgFrame& frame,
                 dai::ImgDetections& detections,
                 const std::vector<std::string>& labelMap) {
    cv::Scalar color(0, 255, 0);
    cv::Scalar textColor(255, 255, 255);
    
    // Get frame data
    cv::Mat cvFrame;
    if(frame.getType() == dai::ImgFrame::Type::RAW16) {
        cvFrame = colorizeDepth(frame.getFrame());
    } else {
        cvFrame = frame.getCvFrame();
    }
    
    // Process detections
    if(detections.transformation) {
        for(const auto& detection : detections.detections) {
            // Get the shape of the frame from which the detections originated
            auto normShape = detections.transformation.value().getSize();
            
            // Create rotated rectangle to remap
            dai::RotatedRect rotRect(
                dai::Rect(
                    dai::Point2f(detection.xmin, detection.ymin),
                    dai::Point2f(detection.xmax, detection.ymax)
                ).denormalize(normShape.first, normShape.second),
                0
            );
            
            // Remap the detection rectangle to target frame
            auto remapped = detections.transformation.value().remapRectTo(frame.transformation, rotRect);
            
            // Get the bounding box from remapped rectangle
            auto bbox = remapped.getOuterRect();
            
            // Draw label
            cv::putText(cvFrame,
                       labelMap[detection.label],
                       cv::Point(bbox[0] + 10, bbox[1] + 20),
                       cv::FONT_HERSHEY_TRIPLEX,
                       0.5,
                       textColor);
            
            // Draw confidence
            cv::putText(cvFrame,
                       std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                       cv::Point(bbox[0] + 10, bbox[1] + 40),
                       cv::FONT_HERSHEY_TRIPLEX,
                       0.5,
                       textColor);
            
            // Draw rectangle
            cv::rectangle(cvFrame, 
                         cv::Point(bbox[0], bbox[1]), 
                         cv::Point(bbox[2], bbox[3]), 
                         color, 
                         2);
        }
    }
    
    // Show the frame
    cv::imshow(name, cvFrame);
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create and configure nodes
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(cameraNode, modelDescription);
    auto labelMap = detectionNetwork->getClasses();

    auto monoLeft = pipeline.create<dai::node::Camera>();
    monoLeft->build(dai::CameraBoardSocket::CAM_B);
    
    auto monoRight = pipeline.create<dai::node::Camera>();
    monoRight->build(dai::CameraBoardSocket::CAM_C);
    
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // Configure mono cameras output
    auto monoLeftOut = monoLeft->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12);
    auto monoRightOut = monoRight->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12);
    
    // Link mono cameras to stereo
    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    // Configure stereo
    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(true);

    // Create output queues
    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qDet = detectionNetwork->out.createOutputQueue();
    auto qDepth = stereo->disparity.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();
        auto inDepth = qDepth->get<dai::ImgFrame>();

        if(inRgb != nullptr) {
            displayFrame("rgb", *inRgb, *inDet, labelMap.value());
        }
        
        if(inDepth != nullptr) {
            displayFrame("depth", *inDepth, *inDet, labelMap.value());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
} 