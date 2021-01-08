
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


dai::Pipeline createCameraPipeline(){
    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->input);
    
    return p;
}


int main(){
    using namespace std;

    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    cv::Mat frame;
    auto preview = d.getOutputQueue("preview");

    while(1){
        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame){
            printf("Frame - w: %d, h: %d\n", imgFrame->getWidth(), imgFrame->getHeight());
            frame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, imgFrame->getData().data());
            cv::imshow("preview", frame);
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

