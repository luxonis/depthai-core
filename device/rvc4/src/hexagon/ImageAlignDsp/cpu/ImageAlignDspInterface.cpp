#include <cstdint>
#include <stdexcept>
#include <string>

#include "AEEStdErr.h"
#include "ImageAlignDsp.h"
#include "hexagon/utilities/DspCommon.hpp"
#include "remote.h"
#include "vendor/dai_device_dsp.h"

#ifndef DOMAIN_ID
    #define DOMAIN_ID 3  // CDSP
#endif

namespace {

using hexagon::logMessage;

class Session {
   public:
    Session() {
        int nErr = ImageAlignDsp_open(getURI().c_str(), &handle);

        if(nErr != AEE_SUCCESS) {
            logMessage(DAI_LOG_LEVEL_ERROR, "ImageAlignDsp open has failed----- %d", nErr);

            if(nErr == AEE_EUNABLETOLOAD) {
                logMessage(DAI_LOG_LEVEL_ERROR, "DSP open failed AEE_EUNABLETOLOAD");
            }

            throw hexagon::DspCallError(nErr, "DSP open failed");
        } else {
            logMessage(DAI_LOG_LEVEL_INFO, "DSP open SUCCESS");
        }

        if(handle == static_cast<uint64_t>(-1)) {
            logMessage(DAI_LOG_LEVEL_ERROR, "DSP open failed!");
        }
    }

    ~Session() {
        if(handle) {
            logMessage(DAI_LOG_LEVEL_INFO, "DSP Close #################");

            ImageAlignDsp_close(handle);
        }
    }

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    uint64_t handle = static_cast<uint64_t>(-1);  // remote_handle64 of the FastRPC session

   private:
    std::string getURI() {
        std::string url = ImageAlignDsp_URI;
        struct remote_rpc_control_unsigned_module data = {.domain = id, .enable = 1};

        logMessage(DAI_LOG_LEVEL_INFO, "Compute on domain: %d", id);
        if(AEE_SUCCESS != remote_session_control(DSPRPC_CONTROL_UNSIGNED_MODULE, (void*)&data, sizeof(data))) {
            throw std::runtime_error("Remote_session_control failed");
        }

        url += CDSP_DOMAIN;

        logMessage(DAI_LOG_LEVEL_INFO, "URL: %s", url.c_str());

        return url;
    }

    int id = DOMAIN_ID;
};

Session& session() {
    static Session instance;
    return instance;
}

}  // namespace

extern "C" dai_status dai_device_dsp_open_align(void) {
    return hexagon::guard("dai_device_dsp_open_align", []() -> dai_status {
        (void)session();
        return DAI_STATUS_OK;
    });
}

extern "C" dai_status dai_device_dsp_shift_depth(const dai_device_buf* image, const dai_device_buf* config, const dai_device_buf* out) {
    return hexagon::guard("dai_device_dsp_shift_depth", [&]() -> dai_status {
        if(image == nullptr || config == nullptr || out == nullptr) {
            return DAI_STATUS_INVALID_ARGUMENT;
        }
        const uint64_t handle = session().handle;
        return hexagon::toStatus(
            ImageAlignDsp_process(
                handle, image->data, static_cast<int>(image->size), config->data, static_cast<int>(config->size), out->data, static_cast<int>(out->size)),
            "ImageAlignDsp_process");
    });
}
