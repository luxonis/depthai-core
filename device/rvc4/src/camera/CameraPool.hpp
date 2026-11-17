#pragma once

#include <hardware/camera3.h>

#include <condition_variable>
#include <mutex>
#include <optional>
#include <vector>

// Taken from qmmf-sdk:
// common/memory/qmmf_memory_interface.h
#define GRALLOC_USAGE_SW_READ_OFTEN 0x00000003
#define GRALLOC_USAGE_SW_WRITE_OFTEN 0x00000030
#define GRALLOC_USAGE_HW_FB 0x00001000
#define GRALLOC_USAGE_HW_VIDEO_ENCODER 0x00010000

class CameraPool {
   public:
    // Attention. Keep this class in sync with the qualcomm HAL implementation (CamX/chi-cdk)
    struct private_handle_t : public native_handle {
        enum { PRIV_FLAGS_FRAMEBUFFER = 0x00000001, PRIV_FLAGS_VIDEO_ENCODER = 0x00010000 };

        int fd;
        int flags;
        unsigned int size;
        unsigned int offset;
        int bufferType;
        int format;
        int width;             // holds aligned width of the actual buffer allocated
        int height;            // holds aligned height of the  actual buffer allocated
        int unaligned_width;   // holds width client asked to allocate
        int unaligned_height;  // holds height client asked to allocate

        static const int sNumFds = 2;
        static inline int sNumInts() {
            return (((sizeof(private_handle_t) - sizeof(native_handle_t)) / sizeof(int)) - sNumFds);
        }

        private_handle_t(int fd, unsigned int size, int flags, int bufferType, int format, int width, int height)
            : fd(fd),
              flags(flags),
              size(size),
              offset(0),
              bufferType(bufferType),
              format(format),
              width(width),
              height(height),
              unaligned_width(width),
              unaligned_height(height){};

        ~private_handle_t(){};
    };

    CameraPool(std::optional<int> numFramesPoolOutputs, std::optional<int> maxSizePoolOutputs);

    camera3_stream_buffer_t getFreeBuffer(camera3_stream_t* stream);
    void returnBuffer(const camera3_stream_buffer_t* buffer);
    struct gbm_bo* getBo(const camera3_stream_buffer_t* buffer);
    size_t getCurrentReservedBuffers();
    void printBufferStats();

   private:
    struct BufferWrap {
        camera3_stream_buffer_t buffer;
        struct gbm_bo* bo;
        std::chrono::time_point<std::chrono::steady_clock> lastUsed;
        bool inHal{false};  // probably better to do this without the wrap with the fences in camera3_stream_buffer_t
                            // check if they get correctly changed when sent to HAL and then back or do we need to wait
                            // on the file descriptor we get as documented ...
                            // cameraadaptor doesn't wait on no fd's
    };

    std::mutex bufferPoolLock;
    std::vector<BufferWrap> bufferPool;
    // Below the min of the 2 is taken to be the actual limit of the pool per stream
    // int maxSize{100 * 1024 * 1024};  // Default to 100MB per stream max
    size_t maxSize{10ul * 1024 * 1024 * 1024};  // Default to 10GB per stream max
    int maxFrames{7};                           // Default to lucky 7 frames max
    std::condition_variable freeBufferWait;

    static buffer_handle_t* allocateBuffer(int32_t buf_width, int32_t buf_height, int format, BufferWrap& buf);
    // This has to be called inside a scope that has bufferPoolLock held!!!
    void getUsedResources(camera3_stream_t* stream, int& curFrames, size_t& curSize, size_t& oneSize);
};
