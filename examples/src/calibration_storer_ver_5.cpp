
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
    std::string calibBinaryFile("/home/sachin/Desktop/bw1098obc_14442C10A14391D000.calib");
    std::string boardConfigFile("/home/sachin/Desktop/luxonis/depthai/resources/boards/BW1098OBC.json");
    dai::CalibrationHandler calibData(calibBinaryFile, boardConfigFile);
    std::string destFilename("/home/sachin/Desktop/luxonis/depthai-core/examples/calib_data_v53.json");
    
    calibData.eepromToJsonFile(destFilename);

    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    std::cout << "status ->" << d.storeCalibration(calibData) << std::endl;
    
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

