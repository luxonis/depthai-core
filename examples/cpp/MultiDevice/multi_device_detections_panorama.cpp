#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

namespace {
void drawDetections(cv::Mat& frame, const std::vector<dai::ImgDetection>& detections) {
    for(const auto& detection : detections) {
        const auto box = detection.getBoundingBox().denormalize(frame.cols, frame.rows);
        const auto points = box.getPoints();
        for(std::size_t i = 0; i < points.size(); ++i) {
            const auto& a = points[i];
            const auto& b = points[(i + 1) % points.size()];
            cv::line(frame, cv::Point(cvRound(a.x), cvRound(a.y)), cv::Point(cvRound(b.x), cvRound(b.y)), cv::Scalar(0, 255, 0), 2);
        }
        cv::putText(frame,
                    (detection.labelName.empty() ? std::to_string(detection.label) : detection.labelName) + ": " + std::to_string(detection.confidence),
                    cv::Point(cvRound(box.center.x), cvRound(box.center.y)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    .5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}
}  // namespace

int main(int argc, char** argv) {
    if(argc < 5) {
        std::cout << "Usage: multi_device_detections_panorama calibration.json model device1 device2 [device3 ...] [--average] [--projection "
                     "Perspective|Equirectangular|Cylindrical] [--sync-threshold-ms 100] [--panorama-scale 2 (1-4)] [--max-panorama-size WIDTH HEIGHT]\n"
                     "Keys: 1=Off, 2=NMS, 3=Average, W/S=IoU +/-0.05, Q=quit\n";
        return argc > 1 && std::string(argv[1]) == "--help" ? 0 : 1;
    }
    constexpr float FPS = 5;
    std::vector<std::string> identifiers;
    bool average = false;
    auto projection = dai::CameraModel::Cylindrical;
    auto syncThreshold = std::chrono::milliseconds(100);
    int panoramaScale = 2;
    int maxPanoramaWidth = 0, maxPanoramaHeight = 0;
    for(int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--average") {
            average = true;
            continue;
        }
        if(arg == "--panorama-scale") {
            if(++i == argc) throw std::invalid_argument("--panorama-scale needs a value");
            panoramaScale = std::stoi(argv[i]);
            if(panoramaScale < 1 || panoramaScale > 4) throw std::invalid_argument("Panorama scale must be between 1 and 4");
            continue;
        }
        if(arg == "--max-panorama-size") {
            if(i + 2 >= argc) throw std::invalid_argument("--max-panorama-size needs WIDTH HEIGHT");
            maxPanoramaWidth = std::stoi(argv[++i]);
            maxPanoramaHeight = std::stoi(argv[++i]);
            if(maxPanoramaWidth <= 0 || maxPanoramaHeight <= 0) throw std::invalid_argument("Maximum panorama width and height must be positive");
            continue;
        }
        if(arg == "--sync-threshold-ms") {
            if(++i == argc) throw std::invalid_argument("--sync-threshold-ms needs a value");
            const auto value = std::stoll(argv[i]);
            if(value <= 0 || value > 1000 / FPS) throw std::invalid_argument("Sync threshold must be positive and at most one frame period (200 ms at 5 FPS)");
            syncThreshold = std::chrono::milliseconds(value);
            continue;
        }
        if(arg == "--projection") {
            if(++i == argc) throw std::invalid_argument("--projection needs a camera model");
            const std::string value = argv[i];
            if(value == "Perspective")
                projection = dai::CameraModel::Perspective;
            else if(value == "Equirectangular")
                projection = dai::CameraModel::Equirectangular;
            else if(value == "Cylindrical")
                projection = dai::CameraModel::Cylindrical;
            else
                throw std::invalid_argument("Unknown panorama projection: " + value);
            continue;
        }
        identifiers.push_back(arg);
    }
    if(identifiers.size() < 2) throw std::invalid_argument("At least two devices are required");
    // Perspective stretches off-axis views vertically as well as horizontally.
    if(maxPanoramaWidth == 0) {
        maxPanoramaWidth = 1600 * panoramaScale;
        maxPanoramaHeight = (projection == dai::CameraModel::Perspective ? 1600 : 800) * panoramaScale;
    }
    dai::Pipeline pipeline(false);
    pipeline.setMultiDeviceCalibration(dai::beta::MultiDeviceCalibrationHandler(std::filesystem::path(argv[1])).getGraph());
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(syncThreshold);
    auto demux = pipeline.create<dai::node::MessageDemux>();
    demux->setRunOnHost(true);
    sync->out.link(demux->input);
    auto filter = pipeline.create<dai::node::ImgDetectionsFilter>();
    using OverlapMode = dai::ImgDetectionsFilterConfig::OverlapMode;
    const std::array overlapModes = {OverlapMode::OFF, OverlapMode::NMS, OverlapMode::AVERAGE};
    const std::array modeNames = {"OFF", "NMS", "AVERAGE"};
    int modeIndex = average ? 2 : 1;
    float iouThreshold = .4f;
    filter->initialConfig->setConfidenceRange(.5f);
    filter->initialConfig->overlapMode = overlapModes[modeIndex];
    filter->initialConfig->overlapIouThreshold = iouThreshold;
    auto configQueue = filter->inputConfig.createInputQueue(1, false);
    std::vector<dai::Node::Output*> views;
    std::vector<std::pair<std::string, std::shared_ptr<dai::MessageQueue>>> cameraQueues;
    cameraQueues.reserve(identifiers.size());
    for(const auto& identifier : identifiers) {
        auto device = pipeline.addDevice(dai::DeviceInfo(identifier));
        auto camera = pipeline.createForDevice<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);
        dai::NNModelDescription model;
        model.model = argv[2];
        model.platform = device->getPlatformAsString();
        auto network = pipeline.createForDevice<dai::node::DetectionNetwork>(device)->build(camera, model, FPS);
        // Pair each camera's detections with the image actually used for inference.
        auto cameraDisplay = pipeline.create<dai::node::Sync>();
        cameraDisplay->setRunOnHost(true);
        cameraDisplay->setSyncThreshold(std::chrono::milliseconds(1));
        network->passthrough.link(cameraDisplay->inputs["image"]);
        network->out.link(cameraDisplay->inputs["detections"]);
        cameraQueues.emplace_back("Camera " + identifier + " (CAM_A)", cameraDisplay->out.createOutputQueue(1, false));
        const auto key = "cam" + std::to_string(views.size());
        network->out.link(sync->inputs[key]);
        demux->outputs[key].setPossibleDatatypes({{dai::DatatypeEnum::ImgDetections, false}});
        demux->outputs[key].link(filter->inputs[key]);
        views.push_back(camera->requestOutput({640 * panoramaScale, 400 * panoramaScale}, dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, FPS, true));
    }
    auto stitching = pipeline.create<dai::node::Stitching>()->build(views);
    stitching->setMode(dai::node::Stitching::Mode::PANORAMA);
    stitching->setUseInputCalibration(true);
    stitching->setCameraModel(projection);
    stitching->setMaxPanoramaSize(maxPanoramaWidth, maxPanoramaHeight);
    stitching->setSyncThreshold(syncThreshold);
    stitching->out.link(filter->inputReference);
    auto display = pipeline.create<dai::node::Sync>();
    display->setRunOnHost(true);
    display->setSyncThreshold(syncThreshold);
    stitching->out.link(display->inputs["panorama"]);
    filter->out.link(display->inputs["detections"]);
    auto queue = display->out.createOutputQueue();
    pipeline.start();
    while(pipeline.isRunning()) {
        for(const auto& [windowName, cameraQueue] : cameraQueues) {
            if(auto cameraGroup = cameraQueue->tryGet<dai::MessageGroup>()) {
                auto frame = cameraGroup->get<dai::ImgFrame>("image")->getCvFrame();
                drawDetections(frame, cameraGroup->get<dai::ImgDetections>("detections")->detections);
                cv::imshow(windowName, frame);
            }
        }
        auto group = queue->tryGet<dai::MessageGroup>();
        if(group) {
            auto frame = group->get<dai::ImgFrame>("panorama")->getCvFrame();
            drawDetections(frame, group->get<dai::ImgDetections>("detections")->detections);
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols, 60), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(frame,
                        cv::format("Duplicates: %s | IoU: %.2f", modeNames[modeIndex], iouThreshold),
                        cv::Point(10, 23),
                        cv::FONT_HERSHEY_SIMPLEX,
                        .6,
                        cv::Scalar(255, 255, 255),
                        1);
            cv::putText(frame,
                        "1: Off | 2: NMS | 3: Average | W/S: IoU +/-0.05 | Q: Quit",
                        cv::Point(10, 47),
                        cv::FONT_HERSHEY_SIMPLEX,
                        .5,
                        cv::Scalar(255, 255, 255),
                        1);
            cv::imshow("Multi-device detections panorama", frame);
        }
        const int key = cv::waitKey(1) & 0xFF;
        if(key == 'q' || key == 'Q') break;
        if(key >= '1' && key <= '3') {
            modeIndex = key - '1';
        } else if(key == 'w' || key == 'W' || key == 's' || key == 'S') {
            const float step = key == 'w' || key == 'W' ? .05f : -.05f;
            iouThreshold = std::clamp(std::round((iouThreshold + step) * 100.f) / 100.f, 0.f, 1.f);
        } else {
            continue;
        }
        auto config = std::make_shared<dai::ImgDetectionsFilterConfig>();
        config->setConfidenceRange(.5f);
        config->overlapMode = overlapModes[modeIndex];
        config->overlapIouThreshold = iouThreshold;
        configQueue->send(config);
    }
    pipeline.stop();
    cv::destroyAllWindows();
}
