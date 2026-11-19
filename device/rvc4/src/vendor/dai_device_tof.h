/*
 * C interface of the VD55H1 ToF depth processing of the vendor module (ToF node). See dai_device_common.h for the
 * rules.
 *
 * The vendor module owns the aligned input and output buffers of the processing. The pointers of
 * dai_device_tof_output stay valid until the next process call or destroy.
 */
#ifndef DAI_DEVICE_TOF_H
#define DAI_DEVICE_TOF_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAI_TOF_PROFILE_LOW_RANGE 0
#define DAI_TOF_PROFILE_MID_RANGE 1
#define DAI_TOF_PROFILE_HIGH_RANGE 2

typedef struct dai_device_tof dai_device_tof;

typedef struct dai_device_tof_params {
    uint32_t raw_width; /* superframe size: 1344x2420, 1344x4832, 1344x7244, 672x2420 or 672x3626 */
    uint32_t raw_height;
    uint8_t want_confidence;
    uint8_t want_intensity;
} dai_device_tof_params;

typedef struct dai_device_tof_config {
    uint32_t profile; /* DAI_TOF_PROFILE_* */
    uint8_t phase_shuffle_temporal_filter;
} dai_device_tof_config;

typedef struct dai_device_tof_output {
    uint32_t depth_width;
    uint32_t depth_height;
    const uint8_t* depth; /* uint16 for each pixel */
    size_t depth_size;
    const uint8_t* amplitude;  /* the amplitude plane of the depth map wrapper */
    const uint8_t* confidence; /* NULL when not wanted */
    const uint8_t* intensity;  /* NULL when not wanted */
    size_t float_plane_size;   /* depth_width * depth_height * 2: the size of one fp16 plane */
} dai_device_tof_output;

/* DAI_STATUS_UNSUPPORTED: the superframe size is not supported */
dai_status dai_device_tof_create(const dai_device_tof_params* params, dai_device_tof** out);
void dai_device_tof_destroy(dai_device_tof* h);
dai_status dai_device_tof_set_config(dai_device_tof* h, const dai_device_tof_config* config);
/* gray16: the raw superframe; dma_buf_fd: its DMA-BUF or -1. A gray16 that is not 128-byte aligned is copied into the
 * aligned input buffer of the vendor module first. */
dai_status dai_device_tof_process(dai_device_tof* h, const uint8_t* gray16, int32_t dma_buf_fd);
/* Exposure time for the next frame from the auto exposure of the processing */
int64_t dai_device_tof_next_exposure_ns(const dai_device_tof* h);
dai_status dai_device_tof_output_get(const dai_device_tof* h, dai_device_tof_output* out);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_TOF_H */
