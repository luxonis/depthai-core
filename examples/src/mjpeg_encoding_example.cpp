
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

    auto mjpegQueue = d.getOutputQueue("mjpeg", 8, false);
    auto previewQueue = d.getOutputQueue("preview", 8, false);
    while(1){

        auto t1 = steady_clock::now();            
        auto preview = previewQueue->get<dai::ImgFrame>();
        auto t2 = steady_clock::now();
        cv::imshow("preview", cv::Mat(preview->getHeight(), preview->getWidth(), CV_8UC3, preview->getData().data()));
        auto t3 = steady_clock::now();
        auto mjpeg = mjpegQueue->get<dai::ImgFrame>();
        auto t4 = steady_clock::now();
        cv::Mat decodedFrame = cv::imdecode( cv::Mat(mjpeg->getData()), cv::IMREAD_COLOR);
        auto t5 = steady_clock::now();
        cv::imshow("mjpeg", decodedFrame);

        
        int previewLatency = duration_cast<milliseconds>(steady_clock::now() - preview->getTimestamp()).count();
        int mjpegLatency = duration_cast<milliseconds>(steady_clock::now() - preview->getTimestamp()).count();

        int ms1 = duration_cast<milliseconds>(t2-t1).count();
        int ms2 = duration_cast<milliseconds>(t3-t2).count();
        int ms3 = duration_cast<milliseconds>(t4-t3).count();
        int ms4 = duration_cast<milliseconds>(t5-t4).count();
        int loop = duration_cast<milliseconds>(t5-t1).count();

        std::cout << ms1 << " " << ms2 << " " << ms3 << " " << ms4 << " loop: " << loop << " | Latencies - preview : " << previewLatency << " mjpeg: " << mjpegLatency << std::endl;
        int key = cv::waitKey(1);
        if (key == 'q'){
            return 0;
        } 
    }

    return 0;
}
