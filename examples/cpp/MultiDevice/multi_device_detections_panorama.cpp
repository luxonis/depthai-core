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

int main(int argc, char** argv) {
    if(argc < 5) {
        std::cout
            << "Usage: multi_device_detections_panorama calibration.json model device1 device2 [device3 ...] [--average] [--blend|--no-blend] [--projection "
               "Perspective|Equirectangular|Cylindrical] [--sync-threshold-ms 33] [--panorama-scale 2 (1-4)]\n"
               "Blending is enabled by default; --no-blend is faster.\n"
               "Keys: 1=Off, 2=NMS, 3=Average, W/S=IoU +/-0.05, Q=quit\n";
        return argc > 1 && std::string(argv[1]) == "--help" ? 0 : 1;
    }
    constexpr int FPS = 30;
    constexpr int PREVIEW_WIDTH = 640, PREVIEW_HEIGHT = 400, PANORAMA_WIDTH = 1280;
    std::vector<std::string> identifiers;
    bool average = false;
    bool blend = true;
    auto projection = dai::CameraModel::Cylindrical;
    auto syncThreshold = std::chrono::milliseconds(1000 / FPS);
    int panoramaScale = 2;
    for(int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "--average") {
            average = true;
            continue;
        }
        if(arg == "--blend" || arg == "--no-blend") {
            blend = arg == "--blend";
            continue;
        }
        if(arg == "--panorama-scale") {
            if(++i == argc) throw std::invalid_argument("--panorama-scale needs a value");
            panoramaScale = std::stoi(argv[i]);
            if(panoramaScale < 1 || panoramaScale > 4) throw std::invalid_argument("Panorama scale must be between 1 and 4");
            continue;
        }
        if(arg == "--sync-threshold-ms") {
            if(++i == argc) throw std::invalid_argument("--sync-threshold-ms needs a value");
            const auto value = std::stoll(argv[i]);
            if(value <= 0 || value > 1000 / FPS)
                throw std::invalid_argument("Sync threshold must be positive and at most one frame period (33.3 ms at 30 FPS)");
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
    std::vector<std::pair<std::string, std::shared_ptr<dai::MessageQueue>>> displayQueues;
    displayQueues.reserve(identifiers.size() + 1);
    for(const auto& identifier : identifiers) {
        auto device = pipeline.addDevice(dai::DeviceInfo(identifier));
        auto camera = pipeline.createForDevice<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);
        dai::NNModelDescription model;
        model.model = argv[2];
        model.platform = device->getPlatformAsString();
        auto network = pipeline.createForDevice<dai::node::DetectionNetwork>(device)->build(camera, model, FPS);
        const auto key = "cam" + std::to_string(views.size());
        network->out.link(sync->inputs[key]);
        demux->outputs[key].setPossibleDatatypes({{dai::DatatypeEnum::ImgDetections, false}});
        demux->outputs[key].link(filter->inputs[key]);
        // Share one NV12 transfer between stitching and preview.
        auto view = camera->requestOutput({640 * panoramaScale, 400 * panoramaScale}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS, true);
        auto cameraDisplay = pipeline.create<dai::node::Sync>();
        cameraDisplay->setRunOnHost(true);
        cameraDisplay->setSyncThreshold(std::chrono::milliseconds(1));
        view->link(cameraDisplay->inputs["image"]);
        network->out.link(cameraDisplay->inputs["detections"]);
        displayQueues.emplace_back("Camera " + std::to_string(views.size()) + ": " + identifier, cameraDisplay->out.createOutputQueue(1, false));
        views.push_back(view);
    }
    auto stitching = pipeline.create<dai::node::Stitching>()->build(views);
    stitching->setMode(dai::node::Stitching::Mode::PANORAMA);
    stitching->setUseInputCalibration(true);
    stitching->setSeamFinder(blend ? dai::node::Stitching::SeamFinder::GRAPHCUT_COLOR : dai::node::Stitching::SeamFinder::NONE);
    stitching->setCameraModel(projection);
    stitching->setMaxPanoramaSize(1600 * panoramaScale, 800 * panoramaScale);
    stitching->setSyncThreshold(syncThreshold);
    stitching->out.link(filter->inputReference);
    for(size_t i = 0; i < views.size(); ++i) {
        stitching->outSourceMasks["input" + std::to_string(i)].link(filter->inputSourceMasks["cam" + std::to_string(i)]);
    }
    auto display = pipeline.create<dai::node::Sync>();
    display->setRunOnHost(true);
    display->setSyncThreshold(syncThreshold);
    stitching->out.link(display->inputs["image"]);
    filter->out.link(display->inputs["detections"]);
    const std::string windowName = "Multi-device detections panorama";
    displayQueues.emplace_back(windowName, display->out.createOutputQueue(1, false));
    cv::Mat combined = cv::Mat::zeros(PREVIEW_HEIGHT * static_cast<int>(identifiers.size()), PREVIEW_WIDTH + PANORAMA_WIDTH, CV_8UC3);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    const double windowScale = std::min(1600.0 / combined.cols, 900.0 / combined.rows);
    cv::resizeWindow(windowName, cvRound(combined.cols * windowScale), cvRound(combined.rows * windowScale));
    pipeline.start();
    while(pipeline.isRunning()) {
        bool updated = false;
        for(size_t index = 0; index < displayQueues.size(); ++index) {
            auto group = displayQueues[index].second->tryGet<dai::MessageGroup>();
            if(!group) continue;
            const bool isPanorama = index == identifiers.size();
            const auto image = group->get<dai::ImgFrame>("image");
            auto frame = image->getCvFrame();
            auto detections = group->get<dai::ImgDetections>("detections");
            if(isPanorama) {
                const double scale = std::min(static_cast<double>(PANORAMA_WIDTH) / frame.cols, static_cast<double>(combined.rows) / frame.rows);
                cv::resize(frame, frame, cv::Size(cvRound(frame.cols * scale), cvRound(frame.rows * scale)), 0, 0, cv::INTER_AREA);
            } else {
                detections = std::make_shared<dai::ImgDetections>(detections->transformTo(image->getTransformation()));
                cv::resize(frame, frame, cv::Size(PREVIEW_WIDTH, PREVIEW_HEIGHT), 0, 0, cv::INTER_AREA);
            }
            for(const auto& detection : detections->detections) {
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
            int x = 0, y = static_cast<int>(index) * PREVIEW_HEIGHT;
            if(isPanorama) {
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
                combined(cv::Rect(PREVIEW_WIDTH, 0, PANORAMA_WIDTH, combined.rows)).setTo(cv::Scalar(0, 0, 0));
                x = PREVIEW_WIDTH + (PANORAMA_WIDTH - frame.cols) / 2;
                y = (combined.rows - frame.rows) / 2;
            } else {
                cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(frame, displayQueues[index].first, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 255, 255), 1);
            }
            frame.copyTo(combined(cv::Rect(x, y, frame.cols, frame.rows)));
            updated = true;
        }
        if(updated) cv::imshow(windowName, combined);
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
