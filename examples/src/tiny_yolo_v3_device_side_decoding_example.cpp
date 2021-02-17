#include <cstdio>
#include <iostream>
#include <chrono>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static std::string label_map[] = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto nnOut = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    auto detectionNetwork = p.create<dai::node::YoloDetectionNetwork>();

    // network specific!!
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setNumClasses(80);
    detectionNetwork->setCoordinateSize(4);
    std::vector<float> anchors{10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319};
    detectionNetwork->setAnchors(anchors);
    std::vector<int> anchorMasks26{1, 2, 3};
    std::vector<int> anchorMasks13{3, 4, 5};
    std::map<std::string, std::vector<int>> anchorMasks;
    anchorMasks.insert(std::make_pair("side26", anchorMasks26));
    anchorMasks.insert(std::make_pair("side13", anchorMasks13));
    detectionNetwork->setAnchorMasks(anchorMasks);
    detectionNetwork->setIouThreshold(0.5f);

    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline p = createNNPipeline(nnPath);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    cv::Mat frame;
    auto preview = d.getOutputQueue("preview", 4, false);
    auto detections = d.getOutputQueue("detections", 4, false);
    
    auto start_time = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0;
    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto det = detections->get<dai::ImgDetections>();

        counter++;
        auto current_time = std::chrono::steady_clock::now();
        float elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(current_time-start_time).count() / 1000.;
        if (elapsed_s > 1) {
            fps = counter / elapsed_s;
            counter = 0;
            start_time = current_time;
        }

        if(imgFrame) {
            frame = toMat(imgFrame->getData(), imgFrame->getWidth(), imgFrame->getHeight(), 3, 1);
        }

        auto color = cv::Scalar(255, 255, 255);
        auto dets = det->detections;
        for(const auto& d : dets) {
            int x1 = d.xmin * frame.cols;
            int y1 = d.ymin * frame.rows;
            int x2 = d.xmax * frame.cols;
            int y2 = d.ymax * frame.rows;

            int label_index = d.label;
            std::string label_str = to_string(label_index);
            if(label_index < sizeof(label_map) / sizeof(label_map[0])) {
                label_str = label_map[label_index];
            }
            cv::putText(frame, label_str, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            char conf_str[10];
            snprintf(conf_str, sizeof conf_str, "%.2f", d.confidence*100);
            cv::putText(frame, conf_str, cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        char fps_str[15];
        snprintf(fps_str, sizeof fps_str, "NN fps: %.2f", fps);
        cv::putText(frame, fps_str, cv::Point(2, imgFrame->getHeight()-4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}