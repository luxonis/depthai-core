#include "depthai/device/DeviceBaseNew.hpp"

#include <iostream>
#include <dlfcn.h>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/DeviceBaseImpl.hpp"

namespace dai {


// Implementation of the implementation class
std::vector<CameraBoardSocket> DeviceBaseImpl::getConnectedCameras() {
    // Return some mock data for demonstration purposes
    return {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B};
}

DeviceBaseImplMock::~DeviceBaseImplMock() {
    // Destructor
    std::cout << "DeviceBaseImplMock destructor called" << std::endl;
}

DeviceBaseImpl::~DeviceBaseImpl() {
    // Destructor
    std::cout << "DeviceBaseImpl destructor called" << std::endl;
}

// END OF IMPLEMENTATION OF THE IMPLEMENTATION CLASS


std::vector<CameraBoardSocket> DeviceBaseImplMock::getConnectedCameras() {
    // Return some mock data for demonstration purposes
    return {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_D};
}

// Constructor
dai::DeviceBaseNew::DeviceBaseNew() : pimpl{static_cast<DeviceBaseImpl*>(new DeviceBaseImplMock())}{
    shared_library_handle = dlopen("libdepthai-device-kb_shared.so", RTLD_LAZY);
    if (!shared_library_handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return;
    }

    // load the symbol
    typedef DeviceBaseImpl* (*create_t)();

    // reset errors
    dlerror();

    create_t createDeviceBaseImpl = (create_t) dlsym(shared_library_handle, "createDeviceImpl");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol 'create': " << dlsym_error << '\n';
        dlclose(shared_library_handle);
        return;
    }
    DeviceBaseImpl* impl = createDeviceBaseImpl();

    if(1){
        pimpl = std::unique_ptr<DeviceBaseImpl>(impl);
    }
}

dai::DeviceBaseNew::~DeviceBaseNew() {
    // Destructor
    std::cout << "DeviceBaseNew destructor called" << std::endl;
}

std::vector<CameraBoardSocket> dai::DeviceBaseNew::getConnectedCameras() {
    return pimpl->getConnectedCameras();
}
}  // namespace dai