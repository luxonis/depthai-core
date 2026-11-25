/*
 * C interface of the DSP offload (src/hexagon): disparity post-processing of StereoDepth and the depth shift of
 * ImageAlign and Align. See dai_device_common.h for the rules.
 *
 * The DSP offload is a separate library: built from src/hexagon when the Hexagon SDK is present, else taken from
 * the dsp/ folder of the prebuilt vendor module. It has its own log setting. StereoDepth and ImageAlign have separate
 * DSP sessions: dai_device_dsp_open_stereo and dai_device_dsp_open_align, or else the first call, open them. Each
 * buffer is a plain view (fd, CPU address, size) of memory that the DSP can map.
 * A status > 0 is the AEE status of the FastRPC call (dai_device_dsp_status_name gives its name).
 */
#ifndef DAI_DEVICE_DSP_H
#define DAI_DEVICE_DSP_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void dai_device_dsp_set_log(dai_device_log_fn fn, void* user, int32_t min_level);

/* Name of an AEE status of the DSP offload, never NULL. For a status without a name the text has the number and is
 * valid until the next call of this function in the same thread. */
const char* dai_device_dsp_status_name(int32_t status);

/* StereoDepth */
/* Opens the DSP session of StereoDepth now. The StereoDepth functions also open it on their first call. */
dai_status dai_device_dsp_open_stereo(void);
dai_status dai_device_dsp_disparity_to_depth(const dai_device_buf* disparity, const dai_device_buf* config, const dai_device_buf* out);
dai_status dai_device_dsp_left_right_check(const dai_device_buf* disparity_lr,
                                           const dai_device_buf* disparity_rl,
                                           const dai_device_buf* config,
                                           const dai_device_buf* out);
dai_status dai_device_dsp_confidence_invalidation(const dai_device_buf* disparity,
                                                  const dai_device_buf* confidence,
                                                  const dai_device_buf* config,
                                                  const dai_device_buf* out);
dai_status dai_device_dsp_combine_disparity_maps(const dai_device_buf* disparity,
                                                 const dai_device_buf* disparity_ds,
                                                 const dai_device_buf* config,
                                                 const dai_device_buf* out);

/* ImageAlign and Align */
/* Opens the DSP session of ImageAlign now. dai_device_dsp_shift_depth also opens it on its first call. */
dai_status dai_device_dsp_open_align(void);
dai_status dai_device_dsp_shift_depth(const dai_device_buf* image, const dai_device_buf* config, const dai_device_buf* out);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_DSP_H */
