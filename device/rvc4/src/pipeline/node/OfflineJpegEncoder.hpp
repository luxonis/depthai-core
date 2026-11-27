#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <cutils/native_handle.h>

#include "vendor/dai_device_jpeg.h"

namespace dai {
namespace gate {

class OfflineJpegEncoder {
   public:
    using CompletionCallback = std::function<void(void* jobTag, uint32_t encodedSize)>;  // Called once per accepted encode(), on the encoder's thread

    explicit OfflineJpegEncoder(CompletionCallback onComplete);
    ~OfflineJpegEncoder();
    OfflineJpegEncoder(const OfflineJpegEncoder&) = delete;
    OfflineJpegEncoder& operator=(const OfflineJpegEncoder&) = delete;

    bool initialize(int width, int height);

    bool isInitialized() const;

    bool encode(const native_handle_t* inputHandle, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag);  // Keep inputHandle valid until completion

    bool encode(int inputFd, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag);

   private:
    CompletionCallback completionCallback;  // The vendor module holds a pointer to this member: the object must not move
    std::unique_ptr<dai_device_jpeg, void (*)(dai_device_jpeg*)> handle;
};

}  // namespace gate
}  // namespace dai
