#pragma once

#include <spdlog/async_logger.h>
#include <stdint.h>

#include <memory>
#include <string>

#include "utilities/EvaDataMemory.hpp"

#ifndef DOMAIN_ID
    #define DOMAIN_ID 3  // CDSP
#endif

// Singleton class used for offloading code on the Hexagon DSP.
class ImageAlignDspHandler {
   public:
    static ImageAlignDspHandler& getInstance(std::shared_ptr<spdlog::async_logger> plogger) {
        static ImageAlignDspHandler instance(plogger);
        return instance;
    }

    ImageAlignDspHandler(const ImageAlignDspHandler&) = delete;
    ImageAlignDspHandler& operator=(const ImageAlignDspHandler&) = delete;
    ImageAlignDspHandler(ImageAlignDspHandler&&) = delete;
    ImageAlignDspHandler& operator=(ImageAlignDspHandler&&) = delete;

    // This is an example of code offloaded on DSP
    int shiftDepthImg(std::shared_ptr<dai::EvaDataMemory> inImageData,
                                   std::shared_ptr<dai::EvaDataMemory> inConfigData,
                                   std::shared_ptr<dai::EvaDataMemory> outData);
    ~ImageAlignDspHandler();

   private:
    ImageAlignDspHandler(std::shared_ptr<spdlog::async_logger> pLogger);
    std::string getURI();

    uint64_t handle = -1;  // remote_handle64 of the FastRPC session
    int id = DOMAIN_ID;

    std::shared_ptr<spdlog::async_logger> logger;
};