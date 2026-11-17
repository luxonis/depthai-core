
#include "ImageAlignDspHandler.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <memory>

#include "ImageAlignDsp.h"
#include "remote.h"
#include "dsp_capabilities_utils.h"
#include "os_defines.h"
#include "pd_status_notification.h"
#include "utilities/EvaDataMemory.hpp"

std::string ImageAlignDspHandler::getURI(void) {
    std::string url = ImageAlignDsp_URI;
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

ImageAlignDspHandler::ImageAlignDspHandler(std::shared_ptr<spdlog::async_logger> pLogger) {
    logger = pLogger;
    int nErr = ImageAlignDsp_open(getURI().c_str(), &handle);

    if(nErr != AEE_SUCCESS) {
        logger->error("SpatialLocationCalculator_open has failed----- {}", nErr);

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

ImageAlignDspHandler::~ImageAlignDspHandler() {
    if(handle) {
        logger->info("DSP Close #################");

        ImageAlignDsp_close(handle);
    }
}

int ImageAlignDspHandler::shiftDepthImg(std::shared_ptr<dai::EvaDataMemory> inImageData,
                               std::shared_ptr<dai::EvaDataMemory> inConfigData,
                               std::shared_ptr<dai::EvaDataMemory> outData) {
    return ImageAlignDsp_process(handle,
                                             inImageData->data(),
                                             inImageData->getMaxSize(),
                                             inConfigData->data(),
                                             inConfigData->getMaxSize(),
                                             outData->data(),
                                             outData->getMaxSize()
                                             );
}