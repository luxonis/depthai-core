
#include <cstdio>
#include <iostream>
#include <string>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

dai::Pipeline createCameraPipeline() {
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

int main(int argc, char** argv) {
    std::string calibBinaryFile, boardConfigFile, destFilename;

    if(argc == 4) {
        calibBinaryFile = std::string(argv[1]);
        boardConfigFile = std::string(argv[2]);
        destFilename = std::string(argv[3]);
    } else {
        std::runtime_error("Required .calib file, board.json file path and destination file path argumnents in the same order");
    }
    dai::CalibrationHandler calibData(calibBinaryFile, boardConfigFile);

    calibData.eepromToJsonFile(destFilename);

    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    // std::cout << "status ->" << d.flashCalibration(calibData) << std::endl;

    d.startPipeline();
    auto preview = d.getOutputQueue("preview");
    cv::Mat frame;

    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame) {
            frame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, imgFrame->getData().data());
            cv::imshow("preview", frame);
            int key = cv::waitKey(1);
            if(key == 'q') {
                return 0;
            }
        } else {
            std::cout << "Not ImgFrame" << std::endl;
        }
    }
    return 0;
}
