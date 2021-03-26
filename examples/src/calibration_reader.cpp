
#include <string>
#include <iostream>
#include <cstdio>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"

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
    std::string filename("/home/sachin/Desktop/luxonis/depthai-core/examples/calib_data2.json");
    
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);

    dai::CalibrationHandler calibData = d.getCalibration();
    std::vector<std::vector<float>> intrinsics;
    int width, height;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics){
        for(auto val : row)
        std::cout << val << "  ";
        std::cout << std::endl;
    }

    std::cout << "Width -> " << width << std::endl;
    std::cout << "Height -> " << height << std::endl;
    
    d.startPipeline();
    auto preview = d.getOutputQueue("preview");
        cv::Mat frame;

    while(1){
        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame){
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

