// Stub of the ToF part of the vendor module (vendor/dai_device_tof.h), for a firmware whose vendor module has no ToF
// part (DEPTHAI_DEVICE_TOF_VD55H1=OFF). Each call fails with DAI_STATUS_UNSUPPORTED, so the ToF node reports an error
// and stops. The common part of the interface (dai_device_last_error, ...) comes from the vendor module.
#include "vendor/dai_device_tof.h"

extern "C" {

dai_status dai_device_tof_create(const dai_device_tof_params* /*params*/, dai_device_tof** out) {
    if(out != nullptr) *out = nullptr;
    return DAI_STATUS_UNSUPPORTED;
}

void dai_device_tof_destroy(dai_device_tof* /*h*/) {}

dai_status dai_device_tof_set_config(dai_device_tof* /*h*/, const dai_device_tof_config* /*config*/) {
    return DAI_STATUS_UNSUPPORTED;
}

dai_status dai_device_tof_process(dai_device_tof* /*h*/, const uint8_t* /*gray16*/, int32_t /*dma_buf_fd*/) {
    return DAI_STATUS_UNSUPPORTED;
}

int64_t dai_device_tof_next_exposure_ns(const dai_device_tof* /*h*/) {
    return 0;
}

dai_status dai_device_tof_output_get(const dai_device_tof* /*h*/, dai_device_tof_output* out) {
    if(out != nullptr) *out = dai_device_tof_output{};
    return DAI_STATUS_UNSUPPORTED;
}

}  // extern "C"
