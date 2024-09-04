#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main() {
    dai::Device device;

    auto devices = device.getAlsaDevices();
    for(auto device : devices) {
        std::cout << "Name: " << device.name << std::endl << "Desc: " << device.desc << std::endl << "IOID: " << device.ioid << std::endl;
    }

    auto pcms = device.getAlsaPCMs();
    for(auto pcm : pcms) {
        std::cout << "Name: " << pcm.name << std::endl
                  << "ID: " << pcm.id << std::endl
                  << "Card #: " << pcm.cardNumber << std::endl
                  << "Device #: " << pcm.deviceNumber << std::endl;
    }
}
