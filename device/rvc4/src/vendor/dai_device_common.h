/*
 * C interface between the public RVC4 firmware and the prebuilt vendor module (depthai-device-rvc4-vendor):
 * the common part. The vendor module includes only the dai_device_*.h headers of this folder, the C and C++
 * standard libraries and the sysroot. The interface has only C types, so a change of a depthai-core header can not
 * change it. A change of these headers needs a new vendor module: increase DAI_DEVICE_ABI_VERSION.
 *
 * Rules:
 *  - Each object is an opaque handle with a _create and a _destroy function.
 *  - Structs have fixed-width fields only. Booleans are uint8_t (0 or 1), named values are uint32_t.
 *  - No exception crosses the interface. A function that can fail returns a dai_status.
 *  - The public code converts its own types (dai::ImgFrame::Type, dai::Interpolation, ...) with a switch.
 */
#ifndef DAI_DEVICE_COMMON_H
#define DAI_DEVICE_COMMON_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Version of this interface. The vendor module defines the symbol dai_device_abi_<version>; the public code refers
 * to it, so a vendor module of another version gives a link error that names the version. */
#define DAI_DEVICE_ABI_VERSION 1
#define DAI_DEVICE_ABI_SYMBOL_CAT_(version) dai_device_abi_##version
#define DAI_DEVICE_ABI_SYMBOL_(version) DAI_DEVICE_ABI_SYMBOL_CAT_(version)
#define DAI_DEVICE_ABI_SYMBOL DAI_DEVICE_ABI_SYMBOL_(DAI_DEVICE_ABI_VERSION)
extern const uint32_t DAI_DEVICE_ABI_SYMBOL;
uint32_t dai_device_abi_version(void);

/* ---- Status of one call -------------------------------------------------------------------------------------- */

/* The status of one call. It is not a state of the device or of an object.
 *    0   DAI_STATUS_OK
 *  > 0   the status of the hardware driver in this call (for example the status of the accelerator driver)
 *  < 0   DAI_STATUS_*: the call failed for another reason
 * When a call fails, dai_device_last_error() gives a text for it. */
typedef int32_t dai_status;
#define DAI_STATUS_OK 0
#define DAI_STATUS_INVALID_ARGUMENT (-1)
#define DAI_STATUS_INVALID_STATE (-2) /* wrong call order, for example run before init */
#define DAI_STATUS_NO_MEMORY (-3)
#define DAI_STATUS_UNSUPPORTED (-4)
#define DAI_STATUS_INTERNAL (-5) /* an exception or an unexpected error in the vendor module */

/* Name of a status. Inline, so both sides have the same text and it is not part of the binary interface. */
static inline const char* dai_status_name(dai_status status) {
    switch(status) {
        case DAI_STATUS_OK:
            return "ok";
        case DAI_STATUS_INVALID_ARGUMENT:
            return "invalid argument";
        case DAI_STATUS_INVALID_STATE:
            return "invalid state";
        case DAI_STATUS_NO_MEMORY:
            return "no memory";
        case DAI_STATUS_UNSUPPORTED:
            return "unsupported";
        case DAI_STATUS_INTERNAL:
            return "internal error";
        default:
            return status > 0 ? "driver status" : "unknown status";
    }
}

/* Text of the last call of the calling thread that failed: one sentence with the name of the driver call and the
 * name of its status. Valid until the next call of this thread that fails. Never NULL. */
const char* dai_device_last_error(void);

/* ---- Log ----------------------------------------------------------------------------------------------------- */

#define DAI_LOG_LEVEL_TRACE 0
#define DAI_LOG_LEVEL_DEBUG 1
#define DAI_LOG_LEVEL_INFO 2
#define DAI_LOG_LEVEL_WARN 3
#define DAI_LOG_LEVEL_ERROR 4

/* Receives the log messages of the vendor module. It can be called from any thread, also from the threads of the
 * hardware drivers. It must not throw. */
typedef void (*dai_device_log_fn)(void* user, int32_t level, const char* message);
/* Messages below min_level are not formatted. fn NULL: no log (the default). */
void dai_device_set_log(dai_device_log_fn fn, void* user, int32_t min_level);

/* ---- Frames --------------------------------------------------------------------------------------------------- */

#define DAI_FRAME_FORMAT_UNKNOWN 0
#define DAI_FRAME_FORMAT_GRAY8 1
#define DAI_FRAME_FORMAT_RAW8 2
#define DAI_FRAME_FORMAT_NV12 3
#define DAI_FRAME_FORMAT_YUV420P 4

/* Interpolation of the hardware scaler and of the hardware warp */
#define DAI_INTERPOLATION_BILINEAR 0
#define DAI_INTERPOLATION_BICUBIC 1

/* ---- Memory --------------------------------------------------------------------------------------------------- */

/* A plain view of memory */
typedef struct dai_device_buf {
    int32_t fd;    /* DMA-BUF file descriptor, -1 if none */
    uint8_t* data; /* CPU address of the first byte */
    size_t size;   /* number of bytes from data */
} dai_device_buf;

/* An accelerator buffer: a buffer that the vendor module allocates, or a DMA-BUF that it registers with the
 * accelerator (no copy). The accelerator uses the bytes from base + offset. */
typedef struct dai_device_mem dai_device_mem;

/* Allocates a new accelerator buffer of size bytes */
dai_status dai_device_mem_alloc(size_t size, dai_device_mem** out);
/* Registers the DMA-BUF fd with CPU address data and size bytes. The caller keeps the memory valid until
 * dai_device_mem_release. */
dai_status dai_device_mem_import(int32_t fd, void* data, size_t size, dai_device_mem** out);
/* Registers the buffer of mem again, with the same offset. The caller keeps mem valid until the clone is released. */
dai_status dai_device_mem_clone(const dai_device_mem* mem, dai_device_mem** out);
/* Unregisters or frees the buffer. NULL: no operation. */
void dai_device_mem_release(dai_device_mem* mem);
/* buf: fd, base address and size of the whole buffer; offset: the current offset */
dai_status dai_device_mem_get(const dai_device_mem* mem, dai_device_buf* buf, size_t* offset);
/* Adds offset to the current offset (at most the size of the buffer) */
dai_status dai_device_mem_add_offset(dai_device_mem* mem, size_t offset);

/* An image in an accelerator buffer */
typedef struct dai_device_image {
    dai_device_mem* mem;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t format; /* DAI_FRAME_FORMAT_* */
    uint32_t p1_offset;
    uint32_t p2_offset;
    uint32_t p3_offset;
} dai_device_image;

/* Layout of an image as the accelerator expects it */
typedef struct dai_device_image_info {
    int32_t driver_format; /* the format code of the driver, for the log */
    uint32_t width;
    uint32_t height;
    uint32_t total_size;
    uint32_t planes;
    uint32_t stride;
    uint32_t aligned_size;
} dai_device_image_info;

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_COMMON_H */
