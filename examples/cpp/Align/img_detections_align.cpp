#include <atomic>
#include <csignal>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

constexpr float FPS = 30.0f;
const std::pair<uint32_t, uint32_t> SOURCE_SIZE{1280, 600};
constexpr int MASK_COLOR_SCALE = 37;
const cv::Scalar TARGET_COLOR(0, 200, 255);

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

std::string getLabel(const dai::ImgDetection& detection) {
    if(!detection.labelName.empty()) {
        return detection.labelName;
    }
    return std::to_string(detection.label);
}

cv::Mat overlayDetectionsMsg(const cv::Mat& frame, dai::ImgDetections& detectionsMsg) {
    cv::Mat output = frame.clone();

    auto segmentationMaskFrame = detectionsMsg.getSegmentationMask();
    if(segmentationMaskFrame.has_value()) {
        cv::Mat segmentationMask = segmentationMaskFrame->getFrame();
        cv::Mat validMask = segmentationMask != 255;

        cv::Mat lut(1, 256, CV_8U);
        for(int i = 0; i < 256; i++) {
            lut.at<uint8_t>(i) = static_cast<uint8_t>((i * MASK_COLOR_SCALE) % 256);
        }
        cv::Mat scaledMask;
        cv::LUT(segmentationMask, lut, scaledMask);
        scaledMask.setTo(0, ~validMask);

        cv::Mat coloredMask;
        cv::applyColorMap(scaledMask, coloredMask, cv::COLORMAP_JET);
        output.copyTo(coloredMask, ~validMask);

        cv::addWeighted(output, 0.7, coloredMask, 0.3, 0.0, output);
    }

    for(const auto& detection : detectionsMsg.detections) {
        const auto bbox = detection.getBoundingBox().denormalize(output.cols, output.rows);
        std::vector<cv::Point> points;
        for(const auto& point : bbox.getPoints()) {
            points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
        }
        const cv::Point anchor = points.front();

        cv::polylines(output, points, true, TARGET_COLOR, 2);
        cv::putText(output, getLabel(detection), {anchor.x + 8, anchor.y + 20}, cv::FONT_HERSHEY_TRIPLEX, 0.5, TARGET_COLOR);
        cv::putText(output,
                    std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                    {anchor.x + 8, anchor.y + 40},
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    TARGET_COLOR);
    }
    return output;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC2) {
        throw std::runtime_error("Align is not supported on the RVC2 platform.");
    }

    dai::NNModelDescription modelDescription;
    modelDescription.model = "luxonis/yolov8-instance-segmentation-large:coco-640x480";

    dai::ImgFrameCapability capability;
    capability.resizeMode = dai::ImgResizeMode::STRETCH;
    capability.enableUndistortion = false;

    dai::Pipeline pipeline(device);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(camera, modelDescription, capability);

    auto* alignToSource = camera->requestOutput(SOURCE_SIZE, std::nullopt, dai::ImgResizeMode::CROP, FPS, true);

    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->addRotateDeg(30.0f);
    manip->initialConfig->setOutputSize(800, 600);
    manip->initialConfig->setBackgroundColor(255, 255, 255);
    manip->setMaxOutputFrameSize(800 * 600 * 3);
    alignToSource->link(manip->inputImage);

    auto align = pipeline.create<dai::node::Align>();
    detectionNetwork->out.link(align->input);
    manip->out.link(align->inputAlignTo);

    auto sourceFrameQueue = detectionNetwork->passthrough.createOutputQueue();
    auto alignToFrameQueue = manip->out.createOutputQueue();
    auto alignedDetectionsQueue = align->outputAligned.createOutputQueue();
    auto sourceDetectionsQueue = detectionNetwork->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning() && !quitEvent) {
        auto sourceFrameMsg = sourceFrameQueue->get<dai::ImgFrame>();
        auto alignToFrameMsg = alignToFrameQueue->get<dai::ImgFrame>();
        auto sourceDetectionsMsg = sourceDetectionsQueue->get<dai::ImgDetections>();
        auto alignedDetectionsMsg = alignedDetectionsQueue->get<dai::ImgDetections>();
        if(sourceFrameMsg == nullptr || alignToFrameMsg == nullptr || sourceDetectionsMsg == nullptr || alignedDetectionsMsg == nullptr) {
            continue;
        }

        cv::Mat sourceImg = overlayDetectionsMsg(sourceFrameMsg->getCvFrame(), *sourceDetectionsMsg);
        cv::Mat alignedImg = overlayDetectionsMsg(alignToFrameMsg->getCvFrame(), *alignedDetectionsMsg);

        cv::imshow("Source frame", sourceImg);
        cv::imshow("AlignTo frame", alignedImg);

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
