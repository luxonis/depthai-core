
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    using namespace std;
    using namespace std::chrono;

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xout = p.create<dai::node::XLinkOut>();
    auto xout2 = p.create<dai::node::XLinkOut>();
    auto videnc = p.create<dai::node::VideoEncoder>();

    // XLinkOut
    xout->setStreamName("mjpeg");
    xout2->setStreamName("preview");

    // ColorCamera    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    //colorCam->setFps(5.0);
    colorCam->setInterleaved(true);

    // VideoEncoder
    videnc->setDefaultProfilePreset(1920, 1080, 30, dai::VideoEncoderProperties::Profile::MJPEG);

    // Link plugins CAM -> XLINK
    colorCam->video.link(videnc->input);
    colorCam->preview.link(xout2->input);
    videnc->bitstream.link(xout->input);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    // Sets queues size and behaviour
    d.getOutputQueue("mjpeg", 8, false);
    d.getOutputQueue("preview", 8, false);
    
    while(1){

        auto ev = d.getQueueEvent();
        if(ev == "preview"){
            auto preview = d.getOutputQueue(ev)->get<dai::ImgFrame>();
            cv::imshow("preview", cv::Mat(preview->getHeight(), preview->getWidth(), CV_8UC3, preview->getData().data()));
        } else if(ev == "mjpeg"){
            auto mjpeg = d.getOutputQueue(ev)->get<dai::ImgFrame>();
            cv::Mat decodedFrame = cv::imdecode(cv::Mat(mjpeg->getData()), cv::IMREAD_COLOR);
            cv::imshow("mjpeg", decodedFrame);
        }
        
        int key = cv::waitKey(1);
        if (key == 'q'){
            return 0;
        } 
    }

    return 0;
}
