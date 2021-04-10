#include "depthai/depthai.hpp"
#include <string>


int main(){
    bool res= false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    std::string path = "/home/sachin/Downloads/usbBootHeader.cmd";
    dai::DeviceBootloader bl(info, path);
    auto progress = [](float p){
        std::cout << "Flashing Progress..." << p << std::endl;
    };
    bl.flashBootloader(progress);
    return 0;
}