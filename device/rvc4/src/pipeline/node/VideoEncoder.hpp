#pragma once
#include <stdint.h>

#include <mutex>

#include "OfflineJpegEncoder.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "rvc4/GstVaapiEncRvc4.hpp"
#include "utilities/ErrorMacros.hpp"
// shared
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class VideoEncoder : public node::VideoEncoder {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

   private:
    std::shared_ptr<GstVaapiEnc> encodePipeline;
    std::mutex encodePipelineMutex;

   public:
    constexpr static const char* NAME = node::VideoEncoder::NAME;

   public:
    class PassthroughMessage {
       public:
        GstBufferPool* fdMemoryPool{nullptr};
        std::shared_ptr<Memory> inputData;
        GstBuffer* inputGst{nullptr};
        ImgFrame frame;
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tsBeforeVideoEncoder;

        ~PassthroughMessage() {
            if(inputGst != nullptr) {
                if(fdMemoryPool == nullptr) {
                    DAI_ERROR("fdMemoryPool was nullptr.");
                    return;
                }
                gst_buffer_pool_release_buffer(fdMemoryPool, inputGst);
            }
        }
    };

    /// One frame queued at the JPEG encoder; handed to OfflineJpegEncoder as the job tag.
    struct JpegJob {
        GstBuffer* outputBuffer{nullptr};
        PassthroughMessage msg;
    };

    VideoEncoder() : VideoEncoder(std::make_unique<VideoEncoder::Properties>()) {}
    VideoEncoder(std::unique_ptr<VideoEncoder::Properties> props) : node::VideoEncoder(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;
    void stop() override;
    // Safe to call from the record node while this encoder is running.
    void requestKeyframe();
    void send_buffer(const uint8_t* buffer, const int buffer_size, const int width, const int height);

    LockingQueue<PassthroughMessage> passthroughQueue;
    void notifyJpegData(JpegJob* job, uint32_t encodedSize);
    void sendBuffer(const PassthroughMessage& passthroughMessage, const uint8_t* buffer, const int buffer_size, const int width, const int height);

   private:
    OfflineJpegEncoder jpegEncoder{[this](void* jobTag, uint32_t encodedSize) { notifyJpegData(static_cast<JpegJob*>(jobTag), encodedSize); }};
    GstBufferPool* outpool{nullptr};
    GstBufferPool* fdMemoryPool{nullptr};
    GstClockTime vencFrameBaseNs{GST_CLOCK_TIME_NONE};
    void initialize(const int width, const int height, const int fps, dai::ImgFrame::Type type);
    std::tuple<bool, std::string> copyFrame(std::shared_ptr<dai::ImgFrame> frame, GstBuffer** bufferPtr);
};

}  // namespace gate
}  // namespace dai
