#include <iostream>
#include <unistd.h>


#include "depthai/Device.hpp"
#include "depthai/DeviceBootloader.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#include "depthai-shared/Assets.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SPIOut.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/MyProducer.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/CommonObjDet.hpp"


#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include "opencv2/opencv.hpp"
#include "fp16/fp16.h"


#include "XLinkLog.h"

dai::Pipeline createNNPipelineYOLO(std::string nnPath, std::string nnConfigPath){
    dai::Pipeline p;

    // set up NN node
    auto nn1 = p.create<dai::node::NeuralNetwork>();
    nn1->setBlobPath(nnPath);

    // set up color camera and link to NN node
    auto colorCam = p.create<dai::node::ColorCamera>();
    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setCamId(0);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->preview.link(nn1->input);

    // testing CommonObjDet 
    auto commonObjDet = p.create<dai::node::CommonObjDet>();
    commonObjDet->setStreamName("yolo");
    commonObjDet->setNNConfigPath(nnConfigPath);
    nn1->out.link(commonObjDet->input);

    // set up SPI out node and link to nn1
    auto spiOut = p.create<dai::node::SPIOut>();
    spiOut->setStreamName("spimetaout");
    spiOut->setBusId(0);
    commonObjDet->out.link(spiOut->input);
//    nn1->out.link(spiOut->input);

    // Watch out for memory usage on the target SPI device. It turns out ESP32 often doesn't have enough contiguous memory to hold a full 300x300 RGB preview image.
//    auto spiOut2 = p.create<dai::node::SPIOut>();
//    spiOut2->setStreamName("spipreview");
//    spiOut2->setBusId(0);
//    colorCam->preview.link(spiOut2->input);

    return p;
}

void flashNNYOLO(std::string nnPath, std::string nnConfigPath){
    using namespace std;

    dai::Pipeline p = createNNPipelineYOLO(nnPath, nnConfigPath);

    bool found;
    dai::DeviceInfo devInfo;
    std::tie(found, devInfo) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(found) {
        dai::DeviceBootloader bootloader(devInfo);

        auto progress = [](float p){ printf("Flashing progress: %f\n", 100*p); };
        bootloader.flash(progress, p);

    } else {
        cout << "No booted (bootloader) devices found..." << endl;
    }
}

void startNNYOLO(std::string nnPath, std::string nnConfigPath){
    using namespace std;

    dai::Pipeline p = createNNPipelineYOLO(nnPath, nnConfigPath);

    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        bool pipelineStarted = d.startPipeline(p);

        while(1){
            usleep(1000000);
        }

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }

}


int main(int argc, char** argv){
    using namespace std;
    cout << "Hello Flashing Example!" << endl;

    std::string nnPath(argv[1]);
    std::string configPath(argv[2]);
//    startNNYOLO(nnPath, configPath);
    flashNNYOLO(nnPath, configPath);

    return 0;
}
