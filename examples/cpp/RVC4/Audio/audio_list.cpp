#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/utility/AudioHelpers.hpp"

int main() {
    dai::Pipeline pipeline;
    dai::Device device;

    auto devices = device.getAlsaDevices();
    for (auto device : devices) {
	    std::cout << "Name: " << device.name << std::endl
		    << "Desc: " << device.desc << std::endl
		    << "IOID: " << device.ioid << std::endl;
    }
}
