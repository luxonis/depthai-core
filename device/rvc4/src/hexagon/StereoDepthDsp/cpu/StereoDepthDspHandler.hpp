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
class StereoDepthDspHandler {
   public:
    static StereoDepthDspHandler& getInstance(std::shared_ptr<spdlog::async_logger> plogger) {
        static StereoDepthDspHandler instance(plogger);
        return instance;
    }

    StereoDepthDspHandler(const StereoDepthDspHandler&) = delete;
    StereoDepthDspHandler& operator=(const StereoDepthDspHandler&) = delete;
    StereoDepthDspHandler(StereoDepthDspHandler&&) = delete;
    StereoDepthDspHandler& operator=(StereoDepthDspHandler&&) = delete;

    int disparityToDepth(
                        std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                        std::shared_ptr<dai::EvaDataMemory> inConfigData,
                        std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                        );
    int leftRightCheck(
                        std::shared_ptr<dai::EvaDataMemory> inDisparityLR,
                        std::shared_ptr<dai::EvaDataMemory> inDisparityRL,
                        std::shared_ptr<dai::EvaDataMemory> inConfigData,
                        std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                        );
    int confidenceInvalidation(
                        std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                        std::shared_ptr<dai::EvaDataMemory> inConfidenceFrame,
                        std::shared_ptr<dai::EvaDataMemory> inConfigData,
                        std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                        );
    int combineDisparityMaps(
                        std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                        std::shared_ptr<dai::EvaDataMemory> inDisparityDsFrame,
                        std::shared_ptr<dai::EvaDataMemory> inConfigData,
                        std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                        );
    ~StereoDepthDspHandler();

   private:
    StereoDepthDspHandler(std::shared_ptr<spdlog::async_logger> pLogger);
    std::string getURI();

    uint64_t handle = -1;  // remote_handle64 of the FastRPC session
    int id = DOMAIN_ID;

    std::shared_ptr<spdlog::async_logger> logger;
};