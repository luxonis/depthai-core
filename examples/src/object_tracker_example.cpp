#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto detectionNetwork = p.create<dai::node::MobileNetDetectionNetwork>();
    auto objectTracker = p.create<dai::node::ObjectTracker>();
    auto trackerOut = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    trackerOut->setStreamName("tracklets");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN) {
        objectTracker->passthroughTrackerFrame.link(xlinkOut->input);
    } else {
        colorCam->preview.link(xlinkOut->input);
    }

    objectTracker->setDetectionLabelsToTrack({15});  // track only person
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    objectTracker->setTrackerIdAssigmentPolicy(dai::TrackerIdAssigmentPolicy::SMALLEST_ID);

    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);
    objectTracker->out.link(trackerOut->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline p = createNNPipeline(nnPath);

    // Connect and start the pipeline
    dai::Device d(p);

    auto preview = d.getOutputQueue("preview", 4, false);
    auto tracklets = d.getOutputQueue("tracklets", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto track = tracklets->get<dai::Tracklets>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto color = cv::Scalar(255, 0, 0);
        cv::Mat trackletFrame = imgFrame->getCvFrame();
        auto trackletsData = track->tracklets;
        for(auto& t : trackletsData) {
            auto roi = t.roi.denormalize(trackletFrame.cols, trackletFrame.rows);
            int x1 = roi.topLeft().x;
            int y1 = roi.topLeft().y;
            int x2 = roi.bottomRight().x;
            int y2 = roi.bottomRight().y;

            int labelIndex = t.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(trackletFrame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream idStr;
            idStr << "ID: " << t.id;
            cv::putText(trackletFrame, idStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream statusStr;
            statusStr << "Status: " << t.status;
            cv::putText(trackletFrame, statusStr.str(), cv::Point(x1 + 10, y1 + 60), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(trackletFrame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(trackletFrame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("tracker", trackletFrame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}