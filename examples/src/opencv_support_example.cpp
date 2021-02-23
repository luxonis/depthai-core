#include <iostream>
#include <cstdio>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Include OpenCV
#include <opencv2/opencv.hpp>

dai::Pipeline createCameraFullPipeline(){

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto xlinkOut2 = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    xlinkOut2->setStreamName("preview");
    
    colorCam->setPreviewSize(320, 180);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
    colorCam->preview.link(xlinkOut2->input);
    
    return p;

}


int main(){
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline p = createCameraFullPipeline();

    // Connect to device with above created pipeline
    dai::Device d(p);

    // Start the pipeline
    d.startPipeline();

    auto video = d.getOutputQueue("video");
    auto preview = d.getOutputQueue("preview");

    while(1) {

        // Retrieves video ImgFrame and converts a cv::Mat copy in BGR format (suitable for opencv usage)
        auto videoFrame = video->get<dai::ImgFrame>();
        cv::imshow("video", videoFrame->getCvFrame());

        // Retrieves preview ImgFrame and returns (without copying deepCopy = false) cv::Mat
        auto previewFrame = preview->get<dai::ImgFrame>();
        cv::imshow("preview", previewFrame->getFrame());

        // Waits a bit and updates windows
        if(cv::waitKey(1) == 'q') break;

    }
    return 0;
}