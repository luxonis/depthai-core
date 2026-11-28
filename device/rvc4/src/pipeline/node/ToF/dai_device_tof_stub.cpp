#include "vendor/dai_device_tof.h"

extern "C" {

dai_status dai_device_tof_create(const dai_device_tof_params*, dai_device_tof** out) {
    if(out != nullptr) *out = nullptr;
    return DAI_STATUS_UNSUPPORTED;
}

void dai_device_tof_destroy(dai_device_tof*) {}

dai_status dai_device_tof_set_config(dai_device_tof*, const dai_device_tof_config*) {
    return DAI_STATUS_UNSUPPORTED;
}

dai_status dai_device_tof_process(dai_device_tof*, const uint8_t*, int32_t) {
    return DAI_STATUS_UNSUPPORTED;
}

int64_t dai_device_tof_next_exposure_ns(const dai_device_tof*) {
    return 0;
}

dai_status dai_device_tof_output_get(const dai_device_tof*, dai_device_tof_output* out) {
    if(out != nullptr) *out = dai_device_tof_output{};
    return DAI_STATUS_UNSUPPORTED;
}

}  // extern "C"
