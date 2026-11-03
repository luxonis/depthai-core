#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc > 1 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: img_detections_filter [COCO model zoo slug, default yolov6-nano]\n";
        return 0;
    }
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, 10);
    dai::NNModelDescription model;
    model.model = argc > 1 ? argv[1] : "yolov6-nano";
    model.platform = pipeline.getDefaultDevice()->getPlatformAsString();
    auto network = pipeline.create<dai::node::DetectionNetwork>()->build(camera, model, 10);
    auto filter = pipeline.create<dai::node::ImgDetectionsFilter>();
    filter->initialConfig->labelsToKeep = std::vector<std::uint32_t>{0};  // COCO person
    filter->initialConfig->setConfidenceRange(.6f);
    filter->initialConfig->maxDetections = 10;
    network->out.link(filter->inputs["cam"]);
    auto display = pipeline.create<dai::node::Sync>();
    display->setRunOnHost(true);
    display->setSyncThreshold(std::chrono::milliseconds(30));
    network->passthrough.link(display->inputs["image"]);
    filter->out.link(display->inputs["detections"]);
    auto queue = display->out.createOutputQueue();
    pipeline.start();
    while(pipeline.isRunning()) {
        auto group = queue->tryGet<dai::MessageGroup>();
        if(group) {
            auto frame = group->get<dai::ImgFrame>("image")->getCvFrame();
            for(const auto& detection : group->get<dai::ImgDetections>("detections")->detections) {
                const auto points = detection.getBoundingBox().denormalize(frame.cols, frame.rows).getPoints();
                for(std::size_t i = 0; i < points.size(); ++i) {
                    const auto& a = points[i];
                    const auto& b = points[(i + 1) % points.size()];
                    cv::line(frame, cv::Point(cvRound(a.x), cvRound(a.y)), cv::Point(cvRound(b.x), cvRound(b.y)), cv::Scalar(0, 255, 0), 2);
                }
            }
            cv::imshow("Filtered people", frame);
        }
        if(cv::waitKey(1) == 'q') break;
    }
    pipeline.stop();
}
