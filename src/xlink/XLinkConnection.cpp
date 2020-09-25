#include "xlink/XLinkConnection.hpp"

#include <chrono>
#include <vector>
#include <thread>

#include <iostream>

#include <XLink.h>


namespace dai
{
    


// STATIC

void XLinkConnection::initXLinkGlobal() {
    if(xlinkGlobalInitialized) return;

    xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
    auto status = XLinkInitialize(&xlinkGlobalHandler);
    if (X_LINK_SUCCESS != status) {
        throw std::runtime_error("Couldn't initialize XLink");    
    }

    xlinkGlobalInitialized = true;
}

std::atomic<bool> XLinkConnection::xlinkGlobalInitialized{false};
XLinkGlobalHandler_t XLinkConnection::xlinkGlobalHandler = {};
std::mutex XLinkConnection::xlinkStreamOperationMutex;


std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(){

    initXLinkGlobal();

    std::vector<DeviceInfo> devices;

    // Get all available devices (unbooted & booted)
    for(const auto& state : {X_LINK_UNBOOTED, X_LINK_BOOTED}){
        
        const int MAX_DEVICES = 32;
        unsigned int numdev = 0;
        deviceDesc_t deviceDescAll[MAX_DEVICES] = {};
        deviceDesc_t suitableDevice = {};
        suitableDevice.protocol = X_LINK_ANY_PROTOCOL;
        suitableDevice.platform = X_LINK_ANY_PLATFORM;

        auto status = XLinkFindAllSuitableDevices(state, suitableDevice, deviceDescAll, MAX_DEVICES, &numdev);
        if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices");

        for (unsigned int i = 0; i < numdev; i++) {
            DeviceInfo info;
            info.desc = deviceDescAll[i];
            info.state = state;
            devices.push_back(info);
        }

    }


    return devices;
        
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state){
    
    auto devices = getAllConnectedDevices();
    for(const auto& d : devices){
        if(d.state == state) return {true, d};
    }
    return {false, DeviceInfo()};

}



XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary){
    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = mvcmdBinary;
    initDevice(deviceDesc);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::string pathToMvcmd){
    bootDevice = true;
    bootWithPath = true;
    this->pathToMvcmd = pathToMvcmd;
    initDevice(deviceDesc);
}

// Skip boot
XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc) {
    bootDevice = false;
    initDevice(deviceDesc);
}


XLinkConnection::~XLinkConnection() {
    if (deviceLinkId != -1) {
        if(rebootOnDestruction) XLinkResetRemote(deviceLinkId);
    }
}

void XLinkConnection::setRebootOnDestruction(bool reboot){
    rebootOnDestruction = reboot;
}

bool XLinkConnection::getRebootOnDestruction(){
    return rebootOnDestruction;
}


bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::string pathToMvcmd){
    auto status = XLinkBoot( const_cast<deviceDesc_t*>(&deviceToBoot) , pathToMvcmd.c_str());
    if (status != X_LINK_SUCCESS) return false;
    return true;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t> mvcmd){
    auto status = XLinkBootMemory( const_cast<deviceDesc_t*>(&deviceToBoot) , mvcmd.data(), mvcmd.size());
    if (status != X_LINK_SUCCESS) return false;
    return true;
}


void XLinkConnection::initDevice(const DeviceInfo& deviceToInit) {
    initXLinkGlobal();
    assert(deviceLinkId == -1);
  
    XLinkError_t rc;
    deviceDesc_t deviceDesc = {};
    
    using namespace std::chrono;

    // if device in booted state, then don't boot, just connect
    if(deviceToInit.state == X_LINK_BOOTED){
        bootDevice = false;
    } else {
        bootDevice = true;
    }

    // boot device
    if(bootDevice){
        if(bootWithPath){
            bootAvailableDevice(deviceToInit.desc, pathToMvcmd);
        } else {
            bootAvailableDevice(deviceToInit.desc, mvcmd);
        }
    } else {
        printf("Device boot is skipped\n");
    }
    
    // Search for booted device
    {
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(X_LINK_BOOTED, deviceToInit.desc, &deviceDesc);
            if (rc == X_LINK_SUCCESS) break;
        } while ( steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

        if (rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device after booting, error message: " + convertErrorCodeToString(rc));
        }
    }

    connectionHandler.devicePath = deviceDesc.name;
    connectionHandler.protocol = deviceDesc.protocol;

    // Try to connect to device
    {
        auto tstart = steady_clock::now();
        do {
            if( ( rc = XLinkConnect(&connectionHandler) ) == X_LINK_SUCCESS) break;
        } while (steady_clock::now() - tstart < WAIT_FOR_CONNECT_TIMEOUT);

        if (rc != X_LINK_SUCCESS) throw std::runtime_error("Failed to connect to device, error message: " + convertErrorCodeToString(rc));
    }

    deviceLinkId = connectionHandler.linkId;
}


void XLinkConnection::openStream(const std::string& streamName, std::size_t maxWriteSize)
{
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    
    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    streamId_t streamId = INVALID_STREAM_ID;

    assert(deviceLinkId != -1);

    for (int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; retryCount++){
    
        streamId = XLinkOpenStream( deviceLinkId, streamName.c_str(), maxWriteSize );

        if(streamId != INVALID_STREAM_ID){
            break;
        }

        // Give some time before retrying
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
    }

    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream");

    streamIdMap[streamName] = streamId;

}


void XLinkConnection::closeStream(const std::string& streamName){

    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    
    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    if(streamIdMap.count(streamName) == 0) return;
    XLinkCloseStream(streamIdMap[streamName]);
    
    // remove from map
    streamIdMap.erase(streamName);

}

void XLinkConnection::writeToStream(const std::string& streamName, std::uint8_t* data, std::size_t size){

    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    auto status = XLinkWriteData(streamIdMap[streamName], const_cast<const std::uint8_t*>(data), size);
    if (status != X_LINK_SUCCESS) throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
   
    // TODO
    // WDOG REIMPLEMENT
    //wdog_keepalive();

}

void XLinkConnection::writeToStream(const std::string& streamName, std::vector<std::uint8_t> data){
    writeToStream(streamName, data.data(), data.size());
}


void XLinkConnection::readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data ){
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't read data from stream: " + streamName);
    data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
    XLinkReleaseData(streamIdMap[streamName]);
}

std::vector<std::uint8_t> XLinkConnection::readFromStream(const std::string& streamName){
    std::vector<std::uint8_t> data;
    readFromStream(streamName, data);
    return data;
}


// USE ONLY WHEN COPYING DATA AT LATER STAGES
streamPacketDesc_t* XLinkConnection::readFromStreamRaw(const std::string& streamName){
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    streamPacketDesc_t* pPacket;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS){
        throw std::runtime_error("Error while reading data from xlink channel: " + streamName);
    }
    return pPacket;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
void XLinkConnection::readFromStreamRawRelease(const std::string& streamName){
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    XLinkReleaseData(streamIdMap[streamName]);
}








std::string XLinkConnection::convertErrorCodeToString(XLinkError_t error_code) const {
    switch(error_code)
    {
        case X_LINK_SUCCESS:                     return "X_LINK_SUCCESS";
        case X_LINK_ALREADY_OPEN:                return "X_LINK_ALREADY_OPEN";
        case X_LINK_COMMUNICATION_NOT_OPEN:      return "X_LINK_COMMUNICATION_NOT_OPEN";
        case X_LINK_COMMUNICATION_FAIL:          return "X_LINK_COMMUNICATION_FAIL";
        case X_LINK_COMMUNICATION_UNKNOWN_ERROR: return "X_LINK_COMMUNICATION_UNKNOWN_ERROR";
        case X_LINK_DEVICE_NOT_FOUND:            return "X_LINK_DEVICE_NOT_FOUND";
        case X_LINK_TIMEOUT:                     return "X_LINK_TIMEOUT";
        case X_LINK_ERROR:                       return "X_LINK_ERROR";
        case X_LINK_OUT_OF_MEMORY:               return "X_LINK_OUT_OF_MEMORY";
        default:                                 return "<UNKNOWN ERROR>";
    }
}


} // namespace dai
