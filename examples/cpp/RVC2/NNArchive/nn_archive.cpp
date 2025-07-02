#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Helper function to normalize bounding box coordinates
std::vector<int> frameNorm(const cv::Mat& frame, const std::vector<float>& bbox) {
    std::vector<int> normVals(bbox.size(), frame.rows);
    for(size_t i = 0; i < bbox.size(); i += 2) {
        normVals[i] = frame.cols;
    }

    std::vector<int> result;
    for(size_t i = 0; i < bbox.size(); i++) {
        float val = std::max(0.0f, std::min(1.0f, bbox[i]));
        result.push_back(static_cast<int>(val * normVals[i]));
    }
    return result;
}

// Helper function to display frame with detections
void displayFrame(const std::string& name, cv::Mat& frame, const std::vector<dai::ImgDetection>& detections, const std::vector<std::string>& labelMap) {
    const cv::Scalar color(255, 0, 0);
    const cv::Scalar textColor(255, 255, 255);

    for(const auto& detection : detections) {
        std::vector<float> bbox = {detection.xmin, detection.ymin, detection.xmax, detection.ymax};
        auto normBbox = frameNorm(frame, bbox);

        // Draw label
        cv::putText(frame, labelMap[detection.label], cv::Point(normBbox[0] + 10, normBbox[1] + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, textColor);

        // Draw confidence
        cv::putText(frame,
                    std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                    cv::Point(normBbox[0] + 10, normBbox[1] + 40),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    textColor);

        // Draw rectangle
        cv::rectangle(frame, cv::Point(normBbox[0], normBbox[1]), cv::Point(normBbox[2], normBbox[3]), color, 2);
    }

    cv::imshow(name, frame);
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Get model from zoo
        dai::NNModelDescription modelDesc;
        modelDesc.model = "yolov6-nano";
        modelDesc.platform = "RVC2";
        auto archivePath = dai::getModelFromZoo(modelDesc, true);

        // Create pipeline
        dai::Pipeline pipeline;

        auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto outRgb = camRgb->requestOutput(std::make_pair(416, 416), dai::ImgFrame::Type::BGR888p, dai::ImgResizeMode::CROP, 15);

        // Load NN archive
        dai::NNArchive nnArchive(archivePath);
        auto config = nnArchive.getConfig<dai::nn_archive::v1::Config>();
        int h = config.model.inputs[0].shape[2];
        int w = config.model.inputs[0].shape[3];

        // Create detection network
        auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
        detectionNetwork->setNumInferenceThreads(2);
        outRgb->link(detectionNetwork->input);
        detectionNetwork->setNNArchive(nnArchive);

        // Create output queues
        auto qRgb = detectionNetwork->passthrough.createOutputQueue();
        auto qDet = detectionNetwork->out.createOutputQueue();

        // Get label map
        auto labelMap = detectionNetwork->getClasses().value();

        // Start pipeline
        pipeline.start();

        cv::Mat frame;
        std::vector<dai::ImgDetection> detections;
        auto startTime = std::chrono::steady_clock::now();
        int counter = 0;

        while(pipeline.isRunning() && !quitEvent) {
            auto inRgb = qRgb->get<dai::ImgFrame>();
            auto inDet = qDet->get<dai::ImgDetections>();

            if(inRgb != nullptr) {
                frame = inRgb->getCvFrame();

                // Add FPS text
                auto currentTime = std::chrono::steady_clock::now();
                float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
                cv::putText(frame, "NN fps: " + std::to_string(fps), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255));
            }

            if(inDet != nullptr) {
                detections = inDet->detections;
                counter++;
            }

            if(!frame.empty()) {
                displayFrame("rgb", frame, detections, labelMap);
            }

            int key = cv::waitKey(1);
            if(key == 'q') {
                break;
            }
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}