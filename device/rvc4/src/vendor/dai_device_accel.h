/*
 * C interface of the hardware accelerator units of the vendor module: downscale, image pyramid, stereo,
 * warp and the feature tracker stages. See dai_device_common.h for the rules.
 *
 * Sequence of a unit: _create, _open, _init, _config, then for each frame _run_async and _wait (or _run), then
 * _close and _destroy. _destroy also closes the unit. The images of a _run_async call must stay valid until _wait.
 * The status of the work that _run_async started comes from _wait (> 0: the status that the accelerator driver gives
 * for the work).
 */
#ifndef DAI_DEVICE_ACCEL_H
#define DAI_DEVICE_ACCEL_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Downscale (ImageManip) ---------------------------------------------------------------------------------- */

typedef struct dai_device_dsimg dai_device_dsimg;

typedef struct dai_device_dsimg_init_params {
    uint32_t actual_fps;
    uint32_t operational_fps;
    uint32_t max_src_width;
    uint32_t max_src_height;
    uint32_t min_dst_width;
    uint32_t min_dst_height;
    uint32_t format; /* DAI_FRAME_FORMAT_* */
} dai_device_dsimg_init_params;

typedef struct dai_device_dsimg_config_params {
    uint32_t interpolation; /* DAI_INTERPOLATION_* */
} dai_device_dsimg_config_params;

dai_status dai_device_dsimg_create(dai_device_dsimg** out);
void dai_device_dsimg_destroy(dai_device_dsimg* h);
dai_status dai_device_dsimg_open(dai_device_dsimg* h);
dai_status dai_device_dsimg_init(dai_device_dsimg* h, const dai_device_dsimg_init_params* init);
dai_status dai_device_dsimg_config(dai_device_dsimg* h, const dai_device_dsimg_config_params* config);
dai_status dai_device_dsimg_close(dai_device_dsimg* h);
/* All downscale units share one accelerator session: each node registers, and the session starts when the first
 * frame of each registered node is done (first_frame_over waits for the others, at most 25 s). */
dai_status dai_device_dsimg_start_session(dai_device_dsimg* h);
void dai_device_dsimg_register_node(dai_device_dsimg* h);
dai_status dai_device_dsimg_first_frame_over(dai_device_dsimg* h);
/* use_metadata 1: the unit reads the stride and the plane offsets of in and out.
 * use_metadata 0: the unit computes them, and writes format, stride, p1_offset and p2_offset into *out. */
dai_status dai_device_dsimg_run_async(dai_device_dsimg* h, const dai_device_image* in, dai_device_image* out, uint8_t use_metadata);
dai_status dai_device_dsimg_wait(dai_device_dsimg* h);
/* Minimum buffer sizes for an image of this size, in the format of the init call. 0 when the query fails. */
uint32_t dai_device_dsimg_in_min_bytes(dai_device_dsimg* h, int32_t width, int32_t height);
uint32_t dai_device_dsimg_out_min_bytes(dai_device_dsimg* h, int32_t width, int32_t height);
dai_status dai_device_dsimg_image_info(dai_device_dsimg* h, int32_t width, int32_t height, dai_device_image_info* out);

/* ---- Image pyramid (StereoDepth) ----------------------------------------------------------------------------- */

typedef struct dai_device_pyrimg dai_device_pyrimg;

typedef struct dai_device_pyrimg_init_params {
    uint8_t out_base_image;
    uint32_t actual_fps;
    uint32_t operational_fps;
    uint32_t width;
    uint32_t height;
    uint32_t format; /* DAI_FRAME_FORMAT_* */
    uint32_t octaves;
    uint32_t scales_per_octave;
} dai_device_pyrimg_init_params;

typedef struct dai_device_pyrimg_config_params {
    uint32_t interpolation; /* DAI_INTERPOLATION_* */
} dai_device_pyrimg_config_params;

dai_status dai_device_pyrimg_create(dai_device_pyrimg** out);
void dai_device_pyrimg_destroy(dai_device_pyrimg* h);
dai_status dai_device_pyrimg_open(dai_device_pyrimg* h);
dai_status dai_device_pyrimg_init(dai_device_pyrimg* h, const dai_device_pyrimg_init_params* init);
dai_status dai_device_pyrimg_config(dai_device_pyrimg* h, const dai_device_pyrimg_config_params* config);
dai_status dai_device_pyrimg_close(dai_device_pyrimg* h);
/* outs: out_count output images, one for each pyramid level */
dai_status dai_device_pyrimg_run_async(dai_device_pyrimg* h, const dai_device_image* in, const dai_device_image* outs, size_t out_count);
dai_status dai_device_pyrimg_wait(dai_device_pyrimg* h);
/* Minimum size of the output buffer of a pyramid level; valid after init */
uint32_t dai_device_pyrimg_min_bytes(const dai_device_pyrimg* h, int32_t level);

/* ---- Stereo (StereoDepth) ------------------------------------------------------------------------------------ */

#define DAI_STEREO_SEARCH_L2R 0
#define DAI_STEREO_SEARCH_R2L 1

typedef struct dai_device_stereo dai_device_stereo;

typedef struct dai_device_stereo_init_params {
    uint8_t conf_output_enable;
    uint8_t slic_output_enable;
    uint32_t actual_fps;
    uint32_t operational_fps;
    uint32_t width;
    uint32_t height;
    uint32_t format; /* DAI_FRAME_FORMAT_* */
} dai_device_stereo_init_params;

/* The parts of dai::StereoDepthConfig that the hardware matcher uses, plus the search direction */
typedef struct dai_device_stereo_p1 {
    uint8_t enable_adaptive;
    uint8_t default_value;
    uint8_t edge_value;
    uint8_t smooth_value;
    uint8_t edge_threshold;
    uint8_t smooth_threshold;
} dai_device_stereo_p1;

typedef struct dai_device_stereo_p2 {
    uint8_t enable_adaptive;
    uint8_t default_value;
    uint8_t edge_value;
    uint8_t smooth_value;
} dai_device_stereo_p2;

typedef struct dai_device_stereo_config_params {
    int8_t census_noise_threshold_offset;
    int8_t census_noise_threshold_scale;
    uint8_t occlusion_confidence_weight;
    uint8_t motion_vector_confidence_weight;
    uint8_t motion_vector_confidence_threshold;
    uint8_t flatness_confidence_weight;
    uint8_t flatness_confidence_threshold;
    uint8_t flatness_override;
    dai_device_stereo_p1 p1;
    dai_device_stereo_p2 p2;
    uint8_t hole_filling_enable;
    uint8_t hole_filling_high_confidence_threshold;
    uint8_t hole_filling_fill_confidence_threshold;
    uint8_t hole_filling_min_valid_disparity;
    uint8_t hole_filling_invalidate_disparities;
    uint8_t adaptive_median_enable;
    uint8_t adaptive_median_confidence_threshold;
    uint32_t search_dir; /* DAI_STEREO_SEARCH_* */
} dai_device_stereo_config_params;

dai_status dai_device_stereo_create(dai_device_stereo** out);
void dai_device_stereo_destroy(dai_device_stereo* h);
dai_status dai_device_stereo_open(dai_device_stereo* h);
dai_status dai_device_stereo_init(dai_device_stereo* h, const dai_device_stereo_init_params* init);
dai_status dai_device_stereo_config(dai_device_stereo* h, const dai_device_stereo_config_params* config);
dai_status dai_device_stereo_close(dai_device_stereo* h);
/* confidence and slic can be NULL */
dai_status dai_device_stereo_run_async(dai_device_stereo* h,
                                       const dai_device_image* left,
                                       const dai_device_image* right,
                                       const dai_device_image* disparity,
                                       const dai_device_image* confidence,
                                       const dai_device_image* slic);
dai_status dai_device_stereo_wait(dai_device_stereo* h);
/* Minimum sizes of the output buffers; valid after init */
uint32_t dai_device_stereo_min_disparity_bytes(const dai_device_stereo* h);
uint32_t dai_device_stereo_min_confidence_bytes(const dai_device_stereo* h);
uint32_t dai_device_stereo_min_slic_bytes(const dai_device_stereo* h);

/* ---- Warp ------------------------------------------------------------------------------------------------------ */

typedef struct dai_device_warp dai_device_warp;

typedef struct dai_device_warp_init_params {
    uint32_t output_width;
    uint32_t output_height;
    uint32_t mesh_width;
    uint32_t mesh_height;
    uint32_t interpolation; /* DAI_INTERPOLATION_* */
    uint32_t in_width;
    uint32_t in_height;
    uint32_t in_format;  /* DAI_FRAME_FORMAT_* */
    uint32_t out_format; /* DAI_FRAME_FORMAT_* */
} dai_device_warp_init_params;

dai_status dai_device_warp_create(dai_device_warp** out);
void dai_device_warp_destroy(dai_device_warp* h);
dai_status dai_device_warp_open(dai_device_warp* h);
dai_status dai_device_warp_init(dai_device_warp* h, const dai_device_warp_init_params* init);
dai_status dai_device_warp_close(dai_device_warp* h);
/* count = mesh_width * mesh_height points of the mesh of input input_index */
dai_status dai_device_warp_set_mesh(dai_device_warp* h, const double* grid_x, const double* grid_y, size_t count, size_t input_index);
dai_status dai_device_warp_run_async(dai_device_warp* h, const dai_device_image* in, const dai_device_image* out, size_t input_index);
dai_status dai_device_warp_wait(dai_device_warp* h);
/* Minimum size of an output buffer; valid after init */
uint32_t dai_device_warp_min_out_bytes(const dai_device_warp* h);
/* Minimum size of an input buffer (the size of the copy of an input that is not DMA-BUF); valid after init */
uint32_t dai_device_warp_min_in_bytes(const dai_device_warp* h);

/* ---- Feature tracker ------------------------------------------------------------------------------------------ */

/* One corner as the corner detector writes it: 8 bytes, no padding. The low 28 bits of score are the Harris score. */
typedef struct dai_device_feature_point {
    uint16_t x;
    uint16_t y;
    uint32_t score;
} dai_device_feature_point;
#define DAI_FEATURE_HARRIS_SCORE_MASK 0x0FFFFFFFu

typedef struct dai_device_point16 {
    uint16_t x;
    uint16_t y;
} dai_device_point16;

/* Corner detector (Harris corners, 8x8 peak mode). The template frame of init gives the size and the format. */
typedef struct dai_device_corner dai_device_corner;
dai_status dai_device_corner_create(dai_device_corner** out);
void dai_device_corner_destroy(dai_device_corner* h);
dai_status dai_device_corner_open(dai_device_corner* h);
dai_status dai_device_corner_init(dai_device_corner* h, const dai_device_image* frame, uint8_t run_async);
dai_status dai_device_corner_config(dai_device_corner* h);
dai_status dai_device_corner_run(dai_device_corner* h, const dai_device_image* frame);
dai_status dai_device_corner_close(dai_device_corner* h);
void dai_device_corner_set_threshold(dai_device_corner* h, uint32_t threshold);         /* default 2000 */
void dai_device_corner_set_robustness(dai_device_corner* h, uint8_t robustness);        /* default 10 */
void dai_device_corner_set_gradient_filter_shift(dai_device_corner* h, uint32_t shift); /* default 1 */
/* The corners of the last run; valid until the next run */
dai_status dai_device_corner_points(const dai_device_corner* h, const dai_device_feature_point** points, size_t* count);

/* Dense optical flow between a reference frame and the current frame. The asynchronous mode is not supported. */
typedef struct dai_device_oflow dai_device_oflow;
typedef struct dai_device_oflow_result {
    const int16_t* forward_mv;
    const int16_t* backward_mv;
    const uint8_t* confidence;
} dai_device_oflow_result;
dai_status dai_device_oflow_create(dai_device_oflow** out);
void dai_device_oflow_destroy(dai_device_oflow* h);
dai_status dai_device_oflow_open(dai_device_oflow* h);
dai_status dai_device_oflow_init(dai_device_oflow* h, const dai_device_image* frame, uint8_t run_async);
dai_status dai_device_oflow_config(dai_device_oflow* h);
dai_status dai_device_oflow_run(dai_device_oflow* h, const dai_device_image* ref, const dai_device_image* cur);
dai_status dai_device_oflow_close(dai_device_oflow* h);
/* The outputs of the last run; valid until the next run */
dai_status dai_device_oflow_result_get(const dai_device_oflow* h, dai_device_oflow_result* out);

/* Descriptor calculator: one binary descriptor of DAI_DESCRIPTOR_SIZE bytes for each search position.
 * The asynchronous mode is not supported. */
#define DAI_DESCRIPTOR_SIZE 32
#define DAI_DESCRIPTOR_MAX_POINTS 1024
typedef struct dai_device_descriptor dai_device_descriptor;
dai_status dai_device_descriptor_create(dai_device_descriptor** out);
void dai_device_descriptor_destroy(dai_device_descriptor* h);
dai_status dai_device_descriptor_open(dai_device_descriptor* h);
dai_status dai_device_descriptor_init(dai_device_descriptor* h, const dai_device_image* frame, uint8_t run_async);
dai_status dai_device_descriptor_config(dai_device_descriptor* h);
dai_status dai_device_descriptor_run(dai_device_descriptor* h, const dai_device_image* frame, const dai_device_point16* positions, size_t count);
dai_status dai_device_descriptor_close(dai_device_descriptor* h);
void dai_device_descriptor_set_num_points(dai_device_descriptor* h, uint32_t num_points);
uint8_t dai_device_descriptor_low_pass_filter(const dai_device_descriptor* h);
/* num_points * DAI_DESCRIPTOR_SIZE bytes of the last run; valid until the next run */
const uint8_t* dai_device_descriptor_data(const dai_device_descriptor* h);

/* Size of the input buffer that the unit expects (the total size of the driver image info of the init frame); valid
 * after init, 0 before */
uint32_t dai_device_corner_in_min_bytes(const dai_device_corner* h);
uint32_t dai_device_oflow_in_min_bytes(const dai_device_oflow* h);
uint32_t dai_device_descriptor_in_min_bytes(const dai_device_descriptor* h);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_ACCEL_H */
