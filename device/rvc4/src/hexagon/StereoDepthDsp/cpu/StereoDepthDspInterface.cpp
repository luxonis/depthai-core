// The StereoDepth part of the C interface of the DSP offload (vendor/dai_device_dsp.h): the CPU side of StereoDepthDsp.
// The node StereoDepth calls these functions directly (vendor/dai_device_dsp.h).
//
// dai_device_dsp_open_stereo, or else the first call of a function, opens the DSP session.

#include <cstdint>
#include <stdexcept>
#include <string>

#include "AEEStdErr.h"
#include "StereoDepthDsp.h"
#include "dsp_capabilities_utils.h"
#include "hexagon/utilities/DspCommon.hpp"
#include "os_defines.h"
#include "pd_status_notification.h"
#include "remote.h"
#include "vendor/dai_device_dsp.h"

#ifndef DOMAIN_ID
    #define DOMAIN_ID 3  // CDSP
#endif

namespace {

using hexagon::logMessage;

// The FastRPC session of StereoDepthDsp
class Session {
   public:
    Session() {
        int nErr = StereoDepthDsp_open(getURI().c_str(), &handle);

        if(nErr != AEE_SUCCESS) {
            logMessage(DAI_LOG_LEVEL_ERROR, "StereoDepthDsp open has failed----- %d", nErr);

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
        if(handle != static_cast<uint64_t>(-1)) {
            logMessage(DAI_LOG_LEVEL_INFO, "DSP Close #################");

            StereoDepthDsp_close(handle);
            handle = static_cast<uint64_t>(-1);
        }
    }

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;

    uint64_t handle = static_cast<uint64_t>(-1);  // remote_handle64 of the FastRPC session

   private:
    std::string getURI() {
        std::string url = StereoDepthDsp_URI;
        struct remote_rpc_control_unsigned_module data = {.domain = id, .enable = 1};
        domain* dspDomain = get_domain(id);

        if(dspDomain == nullptr) {
            throw std::runtime_error("Unable to get domain");
        }

        logMessage(DAI_LOG_LEVEL_INFO, "Compute on domain: %d", id);
        if(AEE_SUCCESS != remote_session_control(DSPRPC_CONTROL_UNSIGNED_MODULE, (void*)&data, sizeof(data))) {
            throw std::runtime_error("Remote_session_control failed");
        }

        url += dspDomain->uri;

        logMessage(DAI_LOG_LEVEL_INFO, "URL: %s", url.c_str());

        return url;
    }

    int id = DOMAIN_ID;
};

// Opens the session on the first call. Thread-safe. When the open throws, the next call tries again.
Session& session() {
    static Session instance;
    return instance;
}

}  // namespace

extern "C" dai_status dai_device_dsp_open_stereo(void) {
    return hexagon::guard("dai_device_dsp_open_stereo", []() -> dai_status {
        (void)session();
        return DAI_STATUS_OK;
    });
}

extern "C" {

dai_status dai_device_dsp_disparity_to_depth(const dai_device_buf* disparity, const dai_device_buf* config, const dai_device_buf* out) {
    return hexagon::guard("dai_device_dsp_disparity_to_depth", [&]() -> dai_status {
        if(disparity == nullptr || config == nullptr || out == nullptr) {
            return DAI_STATUS_INVALID_ARGUMENT;
        }
        const uint64_t handle = session().handle;
        return hexagon::toStatus(StereoDepthDsp_disparityToDepth(handle,
                                                                 disparity->data,
                                                                 static_cast<int>(disparity->size),
                                                                 config->data,
                                                                 static_cast<int>(config->size),
                                                                 out->data,
                                                                 static_cast<int>(out->size)),
                                 "StereoDepthDsp_disparityToDepth");
    });
}

dai_status dai_device_dsp_left_right_check(const dai_device_buf* disparity_lr,
                                           const dai_device_buf* disparity_rl,
                                           const dai_device_buf* config,
                                           const dai_device_buf* out) {
    return hexagon::guard("dai_device_dsp_left_right_check", [&]() -> dai_status {
        if(disparity_lr == nullptr || disparity_rl == nullptr || config == nullptr || out == nullptr) {
            return DAI_STATUS_INVALID_ARGUMENT;
        }
        const uint64_t handle = session().handle;
        return hexagon::toStatus(StereoDepthDsp_leftRightCheck(handle,
                                                               disparity_lr->data,
                                                               static_cast<int>(disparity_lr->size),
                                                               disparity_rl->data,
                                                               static_cast<int>(disparity_rl->size),
                                                               config->data,
                                                               static_cast<int>(config->size),
                                                               out->data,
                                                               static_cast<int>(out->size)),
                                 "StereoDepthDsp_leftRightCheck");
    });
}

dai_status dai_device_dsp_confidence_invalidation(const dai_device_buf* disparity,
                                                  const dai_device_buf* confidence,
                                                  const dai_device_buf* config,
                                                  const dai_device_buf* out) {
    return hexagon::guard("dai_device_dsp_confidence_invalidation", [&]() -> dai_status {
        if(disparity == nullptr || confidence == nullptr || config == nullptr || out == nullptr) {
            return DAI_STATUS_INVALID_ARGUMENT;
        }
        const uint64_t handle = session().handle;
        return hexagon::toStatus(StereoDepthDsp_confidenceInvalidation(handle,
                                                                       disparity->data,
                                                                       static_cast<int>(disparity->size),
                                                                       confidence->data,
                                                                       static_cast<int>(confidence->size),
                                                                       config->data,
                                                                       static_cast<int>(config->size),
                                                                       out->data,
                                                                       static_cast<int>(out->size)),
                                 "StereoDepthDsp_confidenceInvalidation");
    });
}

dai_status dai_device_dsp_combine_disparity_maps(const dai_device_buf* disparity,
                                                 const dai_device_buf* disparity_ds,
                                                 const dai_device_buf* config,
                                                 const dai_device_buf* out) {
    return hexagon::guard("dai_device_dsp_combine_disparity_maps", [&]() -> dai_status {
        if(disparity == nullptr || disparity_ds == nullptr || config == nullptr || out == nullptr) {
            return DAI_STATUS_INVALID_ARGUMENT;
        }
        const uint64_t handle = session().handle;
        return hexagon::toStatus(StereoDepthDsp_combineDisparityMaps(handle,
                                                                     disparity->data,
                                                                     static_cast<int>(disparity->size),
                                                                     disparity_ds->data,
                                                                     static_cast<int>(disparity_ds->size),
                                                                     config->data,
                                                                     static_cast<int>(config->size),
                                                                     out->data,
                                                                     static_cast<int>(out->size)),
                                 "StereoDepthDsp_combineDisparityMaps");
    });
}

}  // extern "C"
