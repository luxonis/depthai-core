#include <fstream>
#include <iostream>

#include "depthai/Device.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/DeviceFlasher.hpp"

#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

std::string protocolToString(XLinkProtocol_t p){
    
    switch (p)
    {
    case X_LINK_USB_VSC : return {"X_LINK_USB_VSC"}; break;
    case X_LINK_USB_CDC : return {"X_LINK_USB_CDC"}; break;
    case X_LINK_PCIE : return {"X_LINK_PCIE"}; break;
    case X_LINK_IPC : return {"X_LINK_IPC"}; break;
    case X_LINK_NMB_OF_PROTOCOLS : return {"X_LINK_NMB_OF_PROTOCOLS"}; break;
    case X_LINK_ANY_PROTOCOL : return {"X_LINK_ANY_PROTOCOL"}; break;
    }    
    return {"UNDEFINED"};
}

std::string platformToString(XLinkPlatform_t p){
    
    switch (p)
    {
    case X_LINK_ANY_PLATFORM : return {"X_LINK_ANY_PLATFORM"}; break;
    case X_LINK_MYRIAD_2 : return {"X_LINK_MYRIAD_2"}; break;
    case X_LINK_MYRIAD_X : return {"X_LINK_MYRIAD_X"}; break;
    }
    return {"UNDEFINED"};
}

std::string stateToString(XLinkDeviceState_t p){
    
    switch (p)
    {
    case X_LINK_ANY_STATE : return {"X_LINK_ANY_STATE"}; break;
    case X_LINK_BOOTED : return {"X_LINK_BOOTED"}; break;
    case X_LINK_UNBOOTED : return {"X_LINK_UNBOOTED"}; break;
    }    
    return {"UNDEFINED"};
}


dai::Pipeline createSimplePipeline(){

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setCamId(0);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->input);

    return p;
}

int main(){

    bool found;
    dai::DeviceInfo devInfo;
    std::tie(found, devInfo) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(found){

        // Create a simple pipeline
        dai::Pipeline pipeline = createSimplePipeline();

        // Connect to device 'devInfo'
        dai::DeviceBootloader flasher(devInfo);

        /*
        auto dap = flasher.createDepthaiApplicationPackage(pipeline);
        std::ofstream dapStream("my_simple_app.dap", std::ios::binary);
        dapStream.write((char*)dap.data(), dap.size());
        */

        // Flash
        auto progress = [](float progress){
            std::cout << "Flashing progress: " << progress * 100.0f << std::endl;
        };

        bool success;
        std::string errorMsg;
        std::tie(success, errorMsg) = flasher.flash(progress, pipeline);

        if(success){
            std::cout << "Flashing successful\n";
        } else {
            std::cout << "Flashing error: " << errorMsg << std::endl;
        }

    } else {
        std::cout << "No devices found" << std::endl;
    }


    auto devices = dai::XLinkConnection::getAllConnectedDevices();

    for(const auto& dev : devices){
        std::cout << "name: " << std::string(dev.desc.name);
        std::cout << ", state: " << stateToString(dev.state);
        std::cout << ", protocol: " << protocolToString(dev.desc.protocol);
        std::cout << ", platform: " << platformToString(dev.desc.platform);
        std::cout << std::endl;
    }


}