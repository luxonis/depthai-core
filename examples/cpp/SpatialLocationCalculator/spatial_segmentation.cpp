#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace {

cv::Mat colorizeDepth(const cv::Mat& depthFrame) {
    cv::Mat depthAbs;
    cv::Mat colorizedDepth;
    cv::convertScaleAbs(depthFrame, depthAbs, 0.03);
    cv::applyColorMap(depthAbs, colorizedDepth, cv::COLORMAP_JET);
    return colorizedDepth;
}

void applySegmentationOverlay(cv::Mat& image, const cv::Mat& segmentationMask) {
    cv::Mat lut(1, 256, CV_8U);
    for(int i = 0; i < 256; ++i) {
        lut.at<uchar>(i) = (i == 255) ? 255 : cv::saturate_cast<uchar>(i * 25);
    }
    cv::Mat scaledMask;
    cv::LUT(segmentationMask, lut, scaledMask);

    cv::Mat coloredMask;
    cv::applyColorMap(scaledMask, coloredMask, cv::COLORMAP_JET);
    image.copyTo(coloredMask, (segmentationMask == 255));
    cv::addWeighted(image, 0.7, coloredMask, 0.3, 0, image);
}

void drawSpatialDetections(cv::Mat& image, const dai::SpatialImgDetections& detections) {
    for(const auto& det : detections.detections) {
        auto bbox = det.getBoundingBox().denormalize(image.cols, image.rows);
        auto points = bbox.getPoints();
        std::vector<cv::Point> contour;
        contour.resize(points.size());
        std::transform(points.begin(), points.end(), contour.begin(), [](const auto& pt) { return cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)); });

        if(contour.empty()) {
            continue;
        }

        cv::polylines(image, contour, true, cv::Scalar(0, 255, 0), 2);

        auto depthCoord = det.spatialCoordinates;
        std::string text = "X: " + std::to_string(static_cast<int>(depthCoord.x / 10)) + " cm, Y: " + std::to_string(static_cast<int>(depthCoord.y / 10))
                           + " cm, Z: " + std::to_string(static_cast<int>(depthCoord.z / 10)) + " cm";
        cv::putText(image, text, contour.front(), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(232, 36, 87), 2);
    }
}

cv::Mat makeTopPanel(int width, bool useSegmentation) {
    cv::Mat topPanel(100, width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::putText(topPanel, "Press 's' to toggle setUseSegmentation", cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 0.7, cv::Scalar(0, 0, 0), 1);
    cv::putText(topPanel,
                "Current setUseSegmentation: " + std::to_string(static_cast<int>(useSegmentation)),
                cv::Point(10, 60),
                cv::FONT_HERSHEY_TRIPLEX,
                0.7,
                cv::Scalar(0, 0, 0),
                1);
    return topPanel;
}

}  // namespace

int main() {
    try {
        auto device = std::make_shared<dai::Device>();
        std::string modelName = "luxonis/yolov8-instance-segmentation-large:coco-640x480";
        bool setRunOnHost = false;
        float fps = 30.0f;
        if(device->getPlatform() == dai::Platform::RVC2) {
            modelName = "luxonis/yolov8-instance-segmentation-nano:coco-512x288";
            setRunOnHost = true;
            fps = 10.0f;
        }
        dai::ImgFrameCapability cap{};
        cap.fps.value = fps;
        cap.enableUndistortion = true;

        dai::Pipeline pipeline{device};

        auto cameraNode = pipeline.create<dai::node::Camera>();
        cameraNode->build(dai::CameraBoardSocket::CAM_A);

        auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(cameraNode, modelName, cap);
        detectionNetwork->detectionParser->setRunOnHost(setRunOnHost);

        auto monoLeft = pipeline.create<dai::node::Camera>();
        monoLeft->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
        auto monoRight = pipeline.create<dai::node::Camera>();
        monoRight->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);

        auto stereo = pipeline.create<dai::node::StereoDepth>();

        monoLeft->requestFullResolutionOutput()->link(stereo->left);
        monoRight->requestFullResolutionOutput()->link(stereo->right);

        auto align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        detectionNetwork->passthrough.link(align->inputAlignTo);

        auto spatialCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();
        spatialCalculator->initialConfig->setUseSegmentation(true);
        align->outputAligned.link(spatialCalculator->inputDepth);
        detectionNetwork->out.link(spatialCalculator->inputDetections);

        auto camQueue = detectionNetwork->passthrough.createOutputQueue();
        auto outputDetectionsQueue = spatialCalculator->outputDetections.createOutputQueue();
        auto depthQueue = spatialCalculator->passthroughDepth.createOutputQueue();
        auto inputConfigQueue = spatialCalculator->inputConfig.createInputQueue();
        auto calculatorConfig = spatialCalculator->initialConfig;

        pipeline.start();

        while(pipeline.isRunning()) {
            auto inSpatialDet = outputDetectionsQueue->get<dai::SpatialImgDetections>();
            auto rgbFrame = camQueue->get<dai::ImgFrame>();
            auto depthFrame = depthQueue->get<dai::ImgFrame>();

            if(!inSpatialDet || !rgbFrame || !depthFrame) {
                continue;
            }
            cv::Mat depthCv = depthFrame->getCvFrame();
            cv::Mat colorizedDepth = colorizeDepth(depthCv);

            cv::Mat image = rgbFrame->getCvFrame();

            auto segmentationMask = inSpatialDet->getCvSegmentationMask();
            if(segmentationMask && calculatorConfig->getUseSegmentation()) {
                applySegmentationOverlay(image, *segmentationMask);
            }

            drawSpatialDetections(image, *inSpatialDet);

            cv::Mat topPanel = makeTopPanel(image.cols, calculatorConfig->getUseSegmentation());
            cv::Mat concatenated;
            cv::vconcat(topPanel, image, concatenated);

            cv::imshow("Depth", colorizedDepth);
            cv::imshow("Spatial detections", concatenated);

            auto key = cv::waitKey(1);
            if(key == 'q') {
                break;
            }
            if(key == 's') {
                calculatorConfig->setUseSegmentation(!calculatorConfig->getUseSegmentation());
                inputConfigQueue->send(calculatorConfig);
            }
        }

        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
