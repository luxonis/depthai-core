#pragma once

// std
#include <cstdint>
#include <vector>

// libraries
#include <XLink/XLink.h>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"

namespace dai
{

    template<typename T>
    bool sendBootloaderRequest(streamId_t streamId, T request){   
        if(XLinkWriteData(streamId, (uint8_t*) &request, sizeof(T)) != X_LINK_SUCCESS) return false;
        return true;
    }

    inline bool receiveBootloaderResponseData(streamId_t streamId, std::vector<uint8_t>& data){
        data = std::vector<uint8_t>();

        streamPacketDesc_t* pPacket;
        if(XLinkReadData(streamId, &pPacket) != X_LINK_SUCCESS) return false;

        // Resize vector
        data.resize(pPacket->length);
        
        // copy data
        memcpy(data.data(), pPacket->data, pPacket->length);

        // release data
        if(XLinkReleaseData(streamId) != X_LINK_SUCCESS) return false; 

        return true;
    }

    template<typename T>
    bool parseBootloaderResponse(const std::vector<uint8_t>& data, T& response){
        // Checks that 'data' is type T
        dai::bootloader::response::Command command;
        if(data.size() < sizeof(command)) return false;
        memcpy(&command, data.data(), sizeof(command));
        if(response.cmd != command) return false;
        if(data.size() < sizeof(response)) return false;
                
        // If yes, memcpy to response
        memcpy(&response, data.data(), sizeof(response));
        return true;
    }

    template<typename T>
    bool receiveBootloaderResponse(streamId_t streamId, T& response){
        // Receive data first
        std::vector<uint8_t> data; 
        if(!receiveBootloaderResponseData(streamId, data)) return false;

        // Then try to parse
        if(!parseBootloaderResponse(data, response)) return false;

        return true;
    }

} // namespace dai


