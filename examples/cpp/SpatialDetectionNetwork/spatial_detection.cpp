#include <atomic>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

// Custom host node for spatial visualization
class SpatialVisualizer : public dai::NodeCRTP<dai::node::HostNode, SpatialVisualizer> {
   public:
    Input& depthInput = inputs["depth"];
    Input& detectionsInput = inputs["detections"];
    Input& rgbInput = inputs["rgb"];

    std::vector<std::string> labelMap;

    std::shared_ptr<SpatialVisualizer> build(Output& depth, Output& detections, Output& rgb) {
        depth.link(depthInput);
        detections.link(detectionsInput);
        rgb.link(rgbInput);
        return std::static_pointer_cast<SpatialVisualizer>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto depthFrame = in->get<dai::ImgFrame>("depth");
        auto detections = in->get<dai::SpatialImgDetections>("detections");
        auto rgbFrame = in->get<dai::ImgFrame>("rgb");

        cv::Mat depthCv = depthFrame->getCvFrame();
        cv::Mat rgbCv = rgbFrame->getCvFrame();
        cv::Mat depthFrameColor = processDepthFrame(depthCv);
        displayResults(rgbCv, depthFrameColor, detections->detections);

        return nullptr;
    }

   private:
    cv::Mat processDepthFrame(const cv::Mat& depthFrame) {
        // Downscale depth frame
        cv::Mat depth_downscaled;
        cv::resize(depthFrame, depth_downscaled, cv::Size(), 0.25, 0.25);

        // Find min and max depth values
        double min_depth = 0, max_depth = 0;
        cv::Mat mask = (depth_downscaled != 0);
        if(cv::countNonZero(mask) > 0) {
            cv::minMaxLoc(depth_downscaled, &min_depth, &max_depth, nullptr, nullptr, mask);
        }

        // Normalize depth frame
        cv::Mat depthFrameColor;
        depthFrame.convertTo(depthFrameColor, CV_8UC1, 255.0 / (max_depth - min_depth), -min_depth * 255.0 / (max_depth - min_depth));

        // Apply color map
        cv::Mat colorized;
        cv::applyColorMap(depthFrameColor, colorized, cv::COLORMAP_HOT);
        return colorized;
    }

    void displayResults(cv::Mat& rgbFrame, cv::Mat& depthFrameColor, const std::vector<dai::SpatialImgDetection>& detections) {
        int height = rgbFrame.rows;
        int width = rgbFrame.cols;

        for(const auto& detection : detections) {
            drawBoundingBoxes(depthFrameColor, detection);
            drawDetections(rgbFrame, detection, width, height);
        }

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rgb", rgbFrame);

        if(cv::waitKey(1) == 'q') {
            stopPipeline();
        }
    }

    void drawBoundingBoxes(cv::Mat& depthFrameColor, const dai::SpatialImgDetection& detection) {
        auto roi = detection.boundingBoxMapping.roi;
        roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
        auto topLeft = roi.topLeft();
        auto bottomRight = roi.bottomRight();
        cv::rectangle(depthFrameColor,
                      cv::Point(static_cast<int>(topLeft.x), static_cast<int>(topLeft.y)),
                      cv::Point(static_cast<int>(bottomRight.x), static_cast<int>(bottomRight.y)),
                      cv::Scalar(255, 255, 255),
                      1);
    }

    void drawDetections(cv::Mat& frame, const dai::SpatialImgDetection& detection, int frameWidth, int frameHeight) {
        int x1 = static_cast<int>(detection.xmin * frameWidth);
        int x2 = static_cast<int>(detection.xmax * frameWidth);
        int y1 = static_cast<int>(detection.ymin * frameHeight);
        int y2 = static_cast<int>(detection.ymax * frameHeight);

        std::string label;
        try {
            label = labelMap[detection.label];
        } catch(...) {
            label = std::to_string(detection.label);
        }

        cv::Scalar color(255, 255, 255);
        cv::putText(frame, label, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        cv::putText(frame, std::to_string(detection.confidence * 100), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        cv::putText(frame,
                    "X: " + std::to_string(static_cast<int>(detection.spatialCoordinates.x)) + " mm",
                    cv::Point(x1 + 10, y1 + 50),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    color);
        cv::putText(frame,
                    "Y: " + std::to_string(static_cast<int>(detection.spatialCoordinates.y)) + " mm",
                    cv::Point(x1 + 10, y1 + 65),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    color);
        cv::putText(frame,
                    "Z: " + std::to_string(static_cast<int>(detection.spatialCoordinates.z)) + " mm",
                    cv::Point(x1 + 10, y1 + 80),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    color);
        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 1);
    }
};

int main() {
    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        auto camRgb = pipeline.create<dai::node::Camera>();
        camRgb->build(dai::CameraBoardSocket::CAM_A);

        auto monoLeft = pipeline.create<dai::node::Camera>();
        monoLeft->build(dai::CameraBoardSocket::CAM_B);

        auto monoRight = pipeline.create<dai::node::Camera>();
        monoRight->build(dai::CameraBoardSocket::CAM_C);

        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>();
        auto visualizer = pipeline.create<SpatialVisualizer>();

        // Configure stereo node
        stereo->setExtendedDisparity(true);
        auto platform = pipeline.getDefaultDevice()->getPlatform();
        if(platform == dai::Platform::RVC2) {
            stereo->setOutputSize(640, 400);
        }

        // Configure spatial detection network
        spatialDetectionNetwork->input.setBlocking(false);
        spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
        spatialDetectionNetwork->setDepthLowerThreshold(100);
        spatialDetectionNetwork->setDepthUpperThreshold(5000);

        // Set up model
        dai::NNModelDescription modelDesc;
        modelDesc.model = "yolov6-nano";
        spatialDetectionNetwork->build(camRgb, stereo, modelDesc, 30);  // 30 FPS

        // Set label map
        visualizer->labelMap = spatialDetectionNetwork->getClasses().value();

        // Linking
        monoLeft->requestOutput(std::make_pair(640, 400))->link(stereo->left);
        monoRight->requestOutput(std::make_pair(640, 400))->link(stereo->right);
        visualizer->build(spatialDetectionNetwork->passthroughDepth, spatialDetectionNetwork->out, spatialDetectionNetwork->passthrough);

        // Start pipeline
        pipeline.start();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}