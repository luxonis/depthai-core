

#include "depthai/Device.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

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

int main(){

    auto devices = dai::XLinkConnection::getAllConnectedDevices();

    for(const auto& dev : devices){
        std::cout << "name: " << std::string(dev.desc.name);
        std::cout << ", state: " << stateToString(dev.state);
        std::cout << ", protocol: " << protocolToString(dev.desc.protocol);
        std::cout << ", platform: " << platformToString(dev.desc.platform);
        std::cout << std::endl;
    }


}