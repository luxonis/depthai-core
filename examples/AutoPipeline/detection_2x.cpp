#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"

/*
The code is the same as for Tiny-yolo-V3, the only difference is the blob file.
The blob was compiled following this tutorial: https://github.com/TNTWEN/OpenVINO-YOLOV4
*/

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

static const std::atomic<bool> syncNN{true};

int main(int argc, char** argv) {  // NOLINT
    using namespace std;           // NOLINT
    using namespace std::chrono;   // NOLINT

    std::string nnArchivePath(NN_ARCHIVE_PATH);
    std::string nnArchivePath2(NN_ARCHIVE_PATH_2);

    std::cout << "Using archive at path: " << nnArchivePath << "\n";
    std::cout << "Using archive 2 at path: " << nnArchivePath2 << "\n";

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    // auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    // auto nnOut = pipeline.create<dai::node::XLinkOut>();

    // xoutRgb->setStreamName("rgb");
    // nnOut->setStreamName("detections");

    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);                                       // NOLINT
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);  // NOLINT
    camRgb->setFps(40);

    const dai::NNArchiveConfig config(nnArchivePath);
    const auto& configV1 = config.getConfigV1();
    if(!configV1) {
        throw std::runtime_error("Wrong config version");
    }
    const auto width = (*configV1).model.inputs[0].shape[2];
    const auto height = (*configV1).model.inputs[0].shape[3];
    if(width > 1920 || height > 1080) {
        // We could decide to load another NNArchive that has a smaller size instead of throwing ...
        // All without loading / reading to memory the whole blob
        throw std::runtime_error("Sorry that's to big");
    }
    camRgb->setPreviewSize(static_cast<int>(width), static_cast<int>(height));
    detectionNetwork->setNNArchive(dai::NNArchive(config, dai::NNArchiveBlob(config, nnArchivePath)));

    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);

    // Linking
    camRgb->preview.link(detectionNetwork->input);
    std::shared_ptr<dai::MessageQueue> queueFrames;
    if(syncNN) {
        queueFrames = detectionNetwork->passthrough.getQueue();
    } else {
        queueFrames = camRgb->preview.getQueue();
    }

    auto detectionQueue = detectionNetwork->out.getQueue();


    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qRgb = queueFrames;
    auto qDet = detectionQueue;
    pipeline.start();

    cv::Mat frame;
    std::vector<dai::ImgDetection> detections;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color2 = cv::Scalar(255, 255, 255);

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](const std::string& name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;  // NOLINT
            int y1 = detection.ymin * frame.rows;  // NOLINT
            int x2 = detection.xmax * frame.cols;  // NOLINT
            int y2 = detection.ymax * frame.rows;  // NOLINT

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    while(true) {
        std::shared_ptr<dai::ImgFrame> inRgb;
        std::shared_ptr<dai::ImgDetections> inDet;

        if(syncNN) {
            inRgb = qRgb->get<dai::ImgFrame>();
            inDet = qDet->get<dai::ImgDetections>();
        } else {
            inRgb = qRgb->tryGet<dai::ImgFrame>();
            inDet = qDet->tryGet<dai::ImgDetections>();
        }

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = static_cast<float>(counter) / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        if(inRgb) {
            frame = inRgb->getCvFrame();
            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(frame, fpsStr.str(), cv::Point(2, static_cast<int>(inRgb->getHeight()) - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color2);
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(!frame.empty()) {
            displayFrame("rgb", frame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    pipeline.stop();
    pipeline.wait();
    return 0;
}
