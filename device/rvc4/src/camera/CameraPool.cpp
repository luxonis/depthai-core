#include "CameraPool.hpp"

#include <camera/CameraMetadata.h>
#include <camera/VendorTagDescriptor.h>
#include <color_metadata.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <gbm_priv.h>
#include <hardware/camera.h>
#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "utilities/ErrorMacros.hpp"

using namespace android;
using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
using VendorTagDescriptor = android::hardware::camera::common::V1_0::helper::VendorTagDescriptor;

#ifdef ENALBE_DEBUG
    #define DEBUG(...) fprintf(stderr, ##__VA_ARGS__)
#else
    #define DEBUG(...)
#endif

#define MEMORY_PATH "/dev/dma_heap/qcom,system"
int gbm_fd_ = -1;
gbm_device* gbm_device = nullptr;

uint32_t getSize(struct gbm_bo* bo) {
    size_t bo_size = 0;
    int ret = gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &bo_size);
    DAI_CHECK_IN(ret == GBM_ERROR_NONE);
    return (uint32_t)bo_size;
}

buffer_handle_t* repackToGralloc(struct gbm_bo* bo, int format) {
    DAI_CHECK_IN(gbm_fd_ >= 0);
    const CameraPool::private_handle_t* priv_hnd = new CameraPool::private_handle_t(
        gbm_bo_get_fd(bo), getSize(bo), (int)(GRALLOC_USAGE_HW_FB | GRALLOC_USAGE_HW_VIDEO_ENCODER), 0, format, gbm_bo_get_width(bo), gbm_bo_get_height(bo));
    DAI_CHECK_IN(priv_hnd != NULL);
    buffer_handle_t* grallocHandle = new buffer_handle_t();
    *grallocHandle = static_cast<buffer_handle_t>(priv_hnd);
    return grallocHandle;
}

struct gbm_bo* allocBuffer(int32_t width, int32_t height, int format) {
    static int bufCount{0};
    DAI_CHECK_IN(gbm_device != nullptr);
    DEBUG("calling gbm_bo_create() start\n");
    const std::unordered_map<int, int> formatMap = {
        {HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, GBM_FORMAT_IMPLEMENTATION_DEFINED},
        {HAL_PIXEL_FORMAT_RAW10, GBM_FORMAT_RAW10},
        {HAL_PIXEL_FORMAT_RAW16, GBM_FORMAT_RAW16},
        {HAL_PIXEL_FORMAT_BLOB, GBM_FORMAT_BLOB},
    };
    const auto realFormat = formatMap.find(format);
    DAI_CHECK_IN(realFormat != formatMap.end());
    uint32_t gbmUsage = (format == HAL_PIXEL_FORMAT_RAW16 || format == HAL_PIXEL_FORMAT_RAW10)  // Avoids RAW metadata plane DRM_IOCTL_PRIME_FD_TO_HANDLE errors
                            ? (GBM_BO_USAGE_CAMERA_WRITE_QTI | GBM_BO_USAGE_CPU_READ_QTI | GBM_BO_USAGE_CPU_WRITE_QTI)
                            : (0 | GBM_BO_USAGE_VIDEO_ENCODER_QTI);
    struct gbm_bo* bo = gbm_bo_create(gbm_device, (uint32_t)width, (uint32_t)height, realFormat->second, gbmUsage);
    if(!bo) {
        DAI_ERROR("Couldn't allocate memory on " MEMORY_PATH " for new image buffer.");
        return nullptr;
    }

    if(format != HAL_PIXEL_FORMAT_RAW16 && format != HAL_PIXEL_FORMAT_RAW10) {  // RAW has no metadata plane: gbm_perform fails (DRM_IOCTL_PRIME_FD_TO_HANDLE)
        char prop[PROPERTY_VALUE_MAX];
        property_get("persist.qmmf.mem.color.space", prop, "ITU_R_601");
        std::string colorspace(prop);

        DEBUG("%s: Using color space: %s\n", __func__, colorspace.c_str());

        int32_t value = 0;
        ColorMetaData colormeta = {};

        auto ret = gbm_perform(GBM_PERFORM_GET_METADATA, bo, GBM_METADATA_GET_COLOR_METADATA, &colormeta);
        if(ret != GBM_ERROR_NONE) {
            DAI_ERROR("Get metadata color space failed.");
            return nullptr;
        }

        if(colorspace == "ITU_R_601") {
            value = GBM_METADATA_COLOR_SPACE_ITU_R_601;
            colormeta.colorPrimaries = ColorPrimaries_BT601_6_625;
            colormeta.range = Range_Full;
            colormeta.transfer = Transfer_SMPTE_170M;
            colormeta.matrixCoefficients = MatrixCoEff_BT601_6_625;
        } else if(colorspace == "ITU_R_601_FR") {
            value = GBM_METADATA_COLOR_SPACE_ITU_R_601_FR;
            colormeta.colorPrimaries = ColorPrimaries_BT601_6_525;
            colormeta.range = Range_Full;
            colormeta.transfer = Transfer_SMPTE_170M;
            colormeta.matrixCoefficients = MatrixCoEff_BT601_6_525;
        } else if(colorspace == "ITU_R_709") {
            value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
            colormeta.colorPrimaries = ColorPrimaries_BT709_5;
            colormeta.range = Range_Full;
            colormeta.transfer = Transfer_sRGB;
            colormeta.matrixCoefficients = MatrixCoEff_BT709_5;
        } else {
            DAI_ERROR("Unsupported color space, using ITU_R_709");
            value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
            colormeta.colorPrimaries = ColorPrimaries_BT709_5;
            colormeta.range = Range_Full;
            colormeta.transfer = Transfer_sRGB;
            colormeta.matrixCoefficients = MatrixCoEff_BT709_5;
        }

        ret = gbm_perform(GBM_PERFORM_SET_METADATA, bo, GBM_METADATA_SET_COLOR_SPACE, (void*)&value);
        if(ret != GBM_ERROR_NONE) {
            DAI_ERROR("Set color space failed.");
            return nullptr;
        }

        ret = gbm_perform(GBM_PERFORM_SET_METADATA, bo, GBM_METADATA_SET_COLOR_METADATA, &colormeta);
        if(ret != GBM_ERROR_NONE) {
            DAI_ERROR("Set metadata color space failed.");
            return nullptr;
        }

        ret = gbm_perform(GBM_PERFORM_GET_METADATA, bo, GBM_METADATA_GET_COLOR_METADATA, &colormeta);
        if(ret != GBM_ERROR_NONE) {
            DAI_ERROR("Get metadata color space failed.");
            return nullptr;
        }

        DEBUG(
            "%s: Color Primaries %d, Color Range %d, Gamma Transfer %d, "
            "Matrix Coefficients %d\n",
            __func__,
            colormeta.colorPrimaries,
            colormeta.range,
            colormeta.transfer,
            colormeta.matrixCoefficients);
    }

    auto stride = gbm_bo_get_stride(bo);

    size_t size = 0;
    auto ret2 = gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &size);
    if(ret2 != GBM_ERROR_NONE) {
        DAI_ERROR("Failed to get size");
    }

    uint32_t align_width = 0;
    ret2 = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH, bo, &align_width);
    if(ret2 != GBM_ERROR_NONE) {
        DAI_ERROR("Failed to get align width");
    }

    uint32_t align_height = 0;
    ret2 = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT, bo, &align_height);
    if(ret2 != GBM_ERROR_NONE) {
        DAI_ERROR("Failed to get align height");
    }

    DEBUG("%s: GBM aligned dim: %dx%d stride: %d size: %ld\n", __func__, align_width, align_height, stride, size);

    return bo;
}

buffer_handle_t* CameraPool::allocateBuffer(int32_t buf_width, int32_t buf_height, int format, BufferWrap& buf) {
    auto handle = allocBuffer(buf_width, buf_height, format);
    if(!handle) {
        buf.bo = nullptr;
        return nullptr;
    }
    buf.bo = handle;
    return repackToGralloc(handle, format);
}

CameraPool::CameraPool(std::optional<int> numFramesPoolOutputs, std::optional<int> maxSizePoolOutputs) {
    if(numFramesPoolOutputs != std::nullopt) {
        maxFrames = *numFramesPoolOutputs;
    }
    if(maxSizePoolOutputs != std::nullopt) {
        maxSize = *maxSizePoolOutputs;
    }
    gbm_fd_ = open(MEMORY_PATH, O_RDONLY | O_CLOEXEC);
    DEBUG("opening " MEMORY_PATH " %d\n", gbm_fd_);
    if(gbm_fd_ < 0) {
        DAI_ERROR("Couldn't open " MEMORY_PATH);
        return;
    }
    gbm_device = gbm_create_device(gbm_fd_);
    if(gbm_device == nullptr) {
        DAI_ERROR("Couldn't create gbm device");
        return;
    }
}

void CameraPool::getUsedResources(camera3_stream_t* stream, int& curFrames, size_t& curSize, size_t& oneSize) {
    curFrames = 0;
    curSize = 0;
    oneSize = 0;
    for(auto& buf : bufferPool) {
        if(buf.buffer.stream == stream && buf.inHal) {
            if(oneSize == 0) {
                oneSize = getSize(buf.bo);
            }
            curSize += oneSize;
            ++curFrames;
        }
    }
}

camera3_stream_buffer_t CameraPool::getFreeBuffer(camera3_stream_t* stream) {
    while(true) {
        std::unique_lock lock(bufferPoolLock);
        for(auto& buf : bufferPool) {
            if(buf.buffer.stream == stream && !buf.inHal) {
                buf.inHal = true;
                buf.lastUsed = std::chrono::steady_clock::now();
                DEBUG("BUF: %p -> true\n", buf.buffer.buffer);
                return buf.buffer;
            }
        }
        int usedBufs{0};
        size_t usedSize{0};
        size_t oneSize{0};
        getUsedResources(stream, usedBufs, usedSize, oneSize);
        if(usedBufs < maxFrames && (usedSize + oneSize) <= maxSize) {
            BufferWrap buf{};
            buf.inHal = true;
            buf.lastUsed = std::chrono::steady_clock::now();
            auto buffer = camera3_stream_buffer_t();
            buffer.stream = stream;
            buffer.buffer = allocateBuffer(stream->width, stream->height, stream->format, buf);
            if(!buffer.buffer) {
                DAI_CHECK_V(false,
                            "Failed to allocate GBM buffer for {}x{} format {} - stream resolution may be unsupported by the DRM driver",
                            stream->width,
                            stream->height,
                            stream->format);
            }
            getSize(buf.bo);
            buffer.status = CAMERA3_BUFFER_STATUS_OK;
            buffer.acquire_fence = -1;
            buffer.release_fence = -1;
            buffer.priv = NULL;
            buf.buffer = buffer;
            bufferPool.push_back(buf);
            DEBUG("New buffer allocated: oneSize was: %lu, num_buffers: %lu\n", oneSize, bufferPool.size());
            DEBUG("BUF: NEW %p -> true\n", buf.buffer.buffer);
            return buffer;
        } else {
            freeBufferWait.wait(lock);
        }
    }
}

void CameraPool::returnBuffer(const camera3_stream_buffer_t* buffer) {
    std::lock_guard lock(bufferPoolLock);
    int idx{0};
    for(auto& buf : bufferPool) {
        DEBUG("BUF[%d]: %p = %d\n", idx, buf.buffer.buffer, buf.inHal);
        ++idx;
    }
    for(auto& buf : bufferPool) {
        if(buf.inHal && buffer->stream == buf.buffer.stream && buf.buffer.buffer == buffer->buffer) {
            buf.inHal = false;
            DEBUG("BUF: %p -> false\n", buf.buffer.buffer);
            freeBufferWait.notify_one();
            return;
        }
    }
    for(auto& buf : bufferPool) {
        if(buffer->stream == buf.buffer.stream && buf.buffer.buffer == buffer->buffer) {
            DAI_CHECK_V(false, "The buffer was already release from HAL: {} = {}", (void*)buf.buffer.buffer, buf.inHal);
        }
    }
    DAI_CHECK_IN(false);
}

struct gbm_bo* CameraPool::getBo(const camera3_stream_buffer_t* buffer) {
    std::lock_guard lock(bufferPoolLock);
    for(auto& buf : bufferPool) {
        if(buffer->stream == buf.buffer.stream && buf.buffer.buffer == buffer->buffer) {
            return buf.bo;
        }
    }
    DAI_CHECK_IN(false);
}

size_t CameraPool::getCurrentReservedBuffers() {
    std::lock_guard lock(bufferPoolLock);
    return bufferPool.size();
}

void CameraPool::printBufferStats() {
    std::lock_guard lock(bufferPoolLock);
    for(auto& buf : bufferPool) {
        const std::chrono::duration<double> diff = std::chrono::steady_clock::now() - buf.lastUsed;
        fprintf(stderr,
                "BUF: %dx%d, %p - %p: last_used: %.6fs, inHAL: %d\n",
                buf.buffer.stream->width,
                buf.buffer.stream->height,
                buf.buffer.stream,
                buf.buffer.buffer,
                diff.count(),
                buf.inHal);
        if(buf.inHal && diff.count() > 10.0) {
            fprintf(stderr,
                    "STUCK BUFFER!!!: %dx%d, %p - %p: last_used: %.6fs, inHAL: %d\n",
                    buf.buffer.stream->width,
                    buf.buffer.stream->height,
                    buf.buffer.stream,
                    buf.buffer.buffer,
                    diff.count(),
                    buf.inHal);
        }
    }
}
