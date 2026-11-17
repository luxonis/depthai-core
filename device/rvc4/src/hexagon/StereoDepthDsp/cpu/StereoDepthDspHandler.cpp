
#include "StereoDepthDspHandler.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <memory>

#include "StereoDepthDsp.h"
#include "remote.h"
#include "dsp_capabilities_utils.h"
#include "os_defines.h"
#include "pd_status_notification.h"
#include "utilities/EvaDataMemory.hpp"

std::string StereoDepthDspHandler::getURI(void) {
    std::string url = StereoDepthDsp_URI;
    struct remote_rpc_control_unsigned_module data = {.domain = id, .enable = 1};
    domain* dspDomain = get_domain(id);

    if(dspDomain == nullptr) {
        throw std::runtime_error("Unable to get domain\n");
    }

    logger->info("Compute on domain: {}", id);
    if(AEE_SUCCESS != remote_session_control(DSPRPC_CONTROL_UNSIGNED_MODULE, (void*)&data, sizeof(data))) {
        throw std::runtime_error("Remote_session_control failed\n");
    }

    url += dspDomain->uri;

    logger->info("URL: {}", url.c_str());

    return url;
}

StereoDepthDspHandler::StereoDepthDspHandler(std::shared_ptr<spdlog::async_logger> pLogger) {
    logger = pLogger;
    int nErr = StereoDepthDsp_open(getURI().c_str(), &handle);

    if(nErr != AEE_SUCCESS) {
        logger->error("StereoDepthDsp open has failed----- {}", nErr);

        if(nErr == AEE_EUNABLETOLOAD) {
            logger->error("DSP open failed AEE_EUNABLETOLOAD");
        }

        throw std::runtime_error("DSP open failed");
    } else {
        logger->info("DSP open SUCCESS");
    }

    if(handle == -1) {
        logger->error("DSP open failed!");
    }
}

StereoDepthDspHandler::~StereoDepthDspHandler() {
    if(handle != -1) {
        logger->info("DSP Close #################");

        StereoDepthDsp_close(handle);
        handle = -1;
    }
}

int StereoDepthDspHandler::disparityToDepth(
                    std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                    std::shared_ptr<dai::EvaDataMemory> inConfigData,
                    std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                    ) {
                        return StereoDepthDsp_disparityToDepth(
                            handle,
                            inDisparityFrame->data(),
                            inDisparityFrame->getMaxSize(),
                            inConfigData->data(),
                            inConfigData->getMaxSize(),
                            outDisparityFrame->data(),
                            outDisparityFrame->getMaxSize()
                        );
                    }
int StereoDepthDspHandler::leftRightCheck(
                    std::shared_ptr<dai::EvaDataMemory> inDisparityLR,
                    std::shared_ptr<dai::EvaDataMemory> inDisparityRL,
                    std::shared_ptr<dai::EvaDataMemory> inConfigData,
                    std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                    ) {
    return StereoDepthDsp_leftRightCheck(
        handle,
        inDisparityLR->data(),
        inDisparityLR->getMaxSize(),
        inDisparityRL->data(),
        inDisparityRL->getMaxSize(),
        inConfigData->data(),
        inConfigData->getMaxSize(),
        outDisparityFrame->data(),
        outDisparityFrame->getMaxSize()
    );
}

int StereoDepthDspHandler::confidenceInvalidation(
                    std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                    std::shared_ptr<dai::EvaDataMemory> inConfidenceFrame,
                    std::shared_ptr<dai::EvaDataMemory> inConfigData,
                    std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                    ) {
    return StereoDepthDsp_confidenceInvalidation(
        handle,
        inDisparityFrame->data(),
        inDisparityFrame->getMaxSize(),
        inConfidenceFrame->data(),
        inConfidenceFrame->getMaxSize(),
        inConfigData->data(),
        inConfigData->getMaxSize(),
        outDisparityFrame->data(),
        outDisparityFrame->getMaxSize()
    );
}

int StereoDepthDspHandler::combineDisparityMaps(
                    std::shared_ptr<dai::EvaDataMemory> inDisparityFrame,
                    std::shared_ptr<dai::EvaDataMemory> inDisparityDsFrame,
                    std::shared_ptr<dai::EvaDataMemory> inConfigData,
                    std::shared_ptr<dai::EvaDataMemory> outDisparityFrame
                    ) {
    return StereoDepthDsp_combineDisparityMaps(
        handle,
        inDisparityFrame->data(),
        inDisparityFrame->getMaxSize(),
        inDisparityDsFrame->data(),
        inDisparityDsFrame->getMaxSize(),
        inConfigData->data(),
        inConfigData->getMaxSize(),
        outDisparityFrame->data(),
        outDisparityFrame->getMaxSize()
    );
}
