#include "OfflineJpegEncoder.hpp"

#include <exception>
#include <utility>

#include "pipeline/PipelineBuilder.hpp"
#include "utilities/ErrorMacros.hpp"
#include "vendor/DeviceError.hpp"

namespace dai {
namespace gate {

namespace {

constexpr const char* UNIT = "Hardware JPEG encoder";

void logCallbackError(const char* what) noexcept {
    try {
        auto logger = PipelineBuilder::getInstance().getSystemLogger();
        if(logger) {
            logger->error("{}: exception in the completion callback: {}", UNIT, what);
        }
    } catch(...) {
        // Nothing to do: this runs on the thread of the encoder
    }
}

/// The completion function of the vendor module (dai_device_jpeg_done_fn): user is the CompletionCallback of the
/// encoder. It runs on the thread of the encoder and must not throw.
void onJpegDone(void* user, void* jobTag, uint32_t encodedSize) noexcept {
    auto* callback = static_cast<OfflineJpegEncoder::CompletionCallback*>(user);
    if(callback == nullptr || !*callback) {
        return;
    }
    try {
        (*callback)(jobTag, encodedSize);
    } catch(const std::exception& e) {
        logCallbackError(e.what());
    } catch(...) {
        logCallbackError("unknown exception");
    }
}

}  // namespace

OfflineJpegEncoder::OfflineJpegEncoder(CompletionCallback onComplete) : completionCallback(std::move(onComplete)), handle(nullptr, &dai_device_jpeg_destroy) {
    device::installDeviceLog();
    dai_device_jpeg* encoder = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_jpeg_create(&onJpegDone, &completionCallback, &encoder));
    handle.reset(encoder);
}

OfflineJpegEncoder::~OfflineJpegEncoder() = default;

// The methods return false when the vendor module gives a status that is not DAI_STATUS_OK, as before the C interface
// (dai_device_last_error() has the reason)

bool OfflineJpegEncoder::initialize(int width, int height) {
    return dai_device_jpeg_initialize(handle.get(), width, height) == DAI_STATUS_OK;
}

bool OfflineJpegEncoder::isInitialized() const {
    return dai_device_jpeg_is_initialized(handle.get()) != 0;
}

bool OfflineJpegEncoder::encode(const native_handle_t* inputHandle, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag) {
    return dai_device_jpeg_encode_handle(handle.get(), inputHandle, outputFd, frameNum, quality, jobTag) == DAI_STATUS_OK;
}

bool OfflineJpegEncoder::encode(int inputFd, int outputFd, int64_t frameNum, uint8_t quality, void* jobTag) {
    return dai_device_jpeg_encode_fd(handle.get(), inputFd, outputFd, frameNum, quality, jobTag) == DAI_STATUS_OK;
}

}  // namespace gate
}  // namespace dai
