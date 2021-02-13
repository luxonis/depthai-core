
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


dai::Pipeline createCameraFullPipeline(){

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
    
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

    cv::Mat frame;
    auto video = d.getOutputQueue("video");

    while(1){

        auto imgFrame = video->get<dai::ImgFrame>();
        if(imgFrame){

            auto dur = steady_clock::now() - imgFrame->getTimestamp();
            
            printf("Frame - w: %d, h: %d, latency: %ldms\n", imgFrame->getWidth(), imgFrame->getHeight(), duration_cast<milliseconds>(dur).count());

            frame = cv::Mat(imgFrame->getHeight() * 3 / 2, imgFrame->getWidth(), CV_8UC1, imgFrame->getData().data());
            
            cv::Mat rgb(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3);

            cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_NV12);
            
            cv::imshow("video", rgb);
            int key = cv::waitKey(1);
            if (key == 'q'){
                return 0;
            } 
        } else {
            std::cout << "Not ImgFrame" << std::endl;
        }
        
    }
    return 0;
}