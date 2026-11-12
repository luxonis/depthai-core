#pragma once

// std
#include <cstdint>
#include <functional>
#include <memory>

// AOSP
#include <cutils/native_handle.h>

namespace dai {
namespace gate {

/**
 * Hardware offline JPEG encoder of the camera subsystem.
 *
 * Encoding is asynchronous: encode() queues one frame and returns; the completion callback runs
 * later, on the encoder's thread, with the caller's `jobTag` and the size of the JPEG written into
 * the output dma-buf. The implementation lives in the private module. The class is non-copyable
 * and usable as a by-value member.
 */
class OfflineJpegEncoder {
   public:
    /// Called once for every accepted encode() call when the JPEG is in the output buffer. The call runs
    /// on the encoder's thread.
    using CompletionCallback = std::function<void(void* jobTag, uint32_t encodedSize)>;

    explicit OfflineJpegEncoder(CompletionCallback onComplete);
    ~OfflineJpegEncoder();
    OfflineJpegEncoder(const OfflineJpegEncoder&) = delete;
    OfflineJpegEncoder& operator=(const OfflineJpegEncoder&) = delete;

    /// Loads the encoder library and creates the encoder instance for frames of `width` x `height`
    /// (input: implementation-defined YUV buffers, output: JPEG blobs). Returns false on failure.
    bool initialize(int width, int height);

    /// True after a successful initialize().
    bool isInitialized() const;

    /// Queues the frame behind `inputHandle` (a camera HAL / gralloc buffer handle that the caller keeps
    /// valid until completion) for encoding into the dma-buf `outputFd`. Returns false when the encoder
    /// rejected the frame; the callback does not run for it, and because the encoder may still reference
    /// the job, neither `jobTag` nor the buffers of a rejected frame are reclaimed (the node treats a
    /// rejection as a fatal error).
    bool encode(const native_handle_t* inputHandle, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag);

    /// Queues the frame in the dma-buf `inputFd` for encoding into the dma-buf `outputFd`.
    bool encode(int inputFd, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace gate
}  // namespace dai
