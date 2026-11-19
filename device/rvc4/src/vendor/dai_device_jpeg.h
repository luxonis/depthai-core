/*
 * C interface of the hardware offline JPEG encoder of the vendor module (VideoEncoder). See dai_device_common.h for
 * the rules.
 *
 * Encoding is asynchronous: an encode call queues one frame and returns. Later the completion function runs on the
 * thread of the encoder, with the job_tag of the call and the size of the JPEG in the output DMA-BUF.
 */
#ifndef DAI_DEVICE_JPEG_H
#define DAI_DEVICE_JPEG_H

#include <cutils/native_handle.h>

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dai_device_jpeg dai_device_jpeg;

/* Runs on the thread of the encoder. It must not throw. */
typedef void (*dai_device_jpeg_done_fn)(void* user, void* job_tag, uint32_t encoded_size);

dai_status dai_device_jpeg_create(dai_device_jpeg_done_fn on_done, void* user, dai_device_jpeg** out);
void dai_device_jpeg_destroy(dai_device_jpeg* h);
dai_status dai_device_jpeg_initialize(dai_device_jpeg* h, int32_t width, int32_t height);
uint8_t dai_device_jpeg_is_initialized(const dai_device_jpeg* h);
dai_status dai_device_jpeg_encode_handle(
    dai_device_jpeg* h, const native_handle_t* input, int32_t output_fd, int64_t frame_num, uint8_t quality, void* job_tag);
dai_status dai_device_jpeg_encode_fd(dai_device_jpeg* h, int32_t input_fd, int32_t output_fd, int64_t frame_num, uint8_t quality, void* job_tag);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_JPEG_H */
