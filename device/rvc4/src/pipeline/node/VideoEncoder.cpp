#include "VideoEncoder.hpp"

// std
#include <chrono>
#include <memory>

// shared
#include <gst/gstpoll.h>
#include <gst/memory/gstmempool.h>
#include <gst/video/gstimagepool.h>
#include <gst/video/video-info.h>

// libraries
#include <hardware/gr_priv_handle.h>

#include "camera/CameraPool.hpp"
#include "camera/CameraSingleton.hpp"
#include "camera/HalBufferMemory.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/NodeTiming.hpp"
#include "utilities/ErrorMacros.hpp"

// #define DEBUG_ON
#undef DEBUG
#ifdef DEBUG_ON
    #define DEBUG(A) fprintf(stderr, "VideoEncoder: " A "\n")
    #define DEBUG_V(A, ...) fprintf(stderr, "VideoEncoder: " A "\n", __VA_ARGS__)
#else
    #define DEBUG(...)
    #define DEBUG_V(...)
#endif

// #define TIME_DEBUG_ON
#undef DAI_PRINTF_TIME
#ifdef TIME_DEBUG_ON
    #define DAI_PRINTF_TIME(...) printf(__VA_ARGS__)
#else
    #define DAI_PRINTF_TIME(...)
#endif

namespace dai {
namespace gate {

#define DEFAULT_PROP_MIN_BUFFERS 2
#define DEFAULT_PROP_MAX_BUFFERS 10

static GstBufferPool* createPool(GstCaps* caps, int width, int height) {
    GstBufferPool* pool = NULL;
    GstStructure* config = NULL;
    GstAllocator* allocator = NULL;

    // image/jpeg is not video/x-raw. gst_video_info_from_caps rejects it, and a
    // failed pool here used to take down MJPEG recording on the first frame.
    DAI_CHECK_IN(width > 0 && height > 0);

    pool = gst_image_buffer_pool_new(GST_IMAGE_BUFFER_POOL_TYPE_ION);

    // Align size to 64 lines
    const gint alignedw = (width + 64 - 1) & ~(64 - 1);
    const gint alignedh = (height + 64 - 1) & ~(64 - 1);
    const gsize aligned_size = static_cast<gsize>(alignedw) * static_cast<gsize>(alignedh) * 4;

    config = gst_buffer_pool_get_config(pool);
    auto res = true;
    gst_buffer_pool_config_set_params(config, caps, aligned_size, DEFAULT_PROP_MIN_BUFFERS, DEFAULT_PROP_MAX_BUFFERS);

    allocator = gst_fd_allocator_new();
    gst_buffer_pool_config_set_allocator(config, allocator, NULL);

    res = gst_buffer_pool_set_config(pool, config);
    if(!res) {
        g_object_unref(pool);
        pool = NULL;
        DAI_CHECK_IN(false);
    }

    g_object_unref(allocator);

    return pool;
}

std::tuple<bool, std::string> VideoEncoder::copyFrame(std::shared_ptr<dai::ImgFrame> frame, GstBuffer** bufferPtr) {
    auto data = frame->data;
    void* _data = data->getData().data();
    size_t _dataSize = data->getData().size();
    int width = frame->getWidth();
    int height = frame->getHeight();
    int stride = frame->getStride();
    int planeStride = frame->getPlaneStride();

    if(fdMemoryPool == nullptr) {
        GstStructure* config = NULL;
        GstAllocator* fdAllocator = NULL;

        fdMemoryPool = gst_mem_buffer_pool_new(GST_MEMORY_BUFFER_POOL_TYPE_ION);
        if(!fdMemoryPool) {
            return {false, "Failed to create memory pool!"};
        }
        config = gst_buffer_pool_get_config(fdMemoryPool);
        if(!config) {
            g_object_unref(fdMemoryPool);
            return {false, "Failed to get memory pool configuration!"};
        }
        gst_buffer_pool_config_set_params(config, NULL, _dataSize, DEFAULT_PROP_MIN_BUFFERS, DEFAULT_PROP_MAX_BUFFERS);
        fdAllocator = gst_fd_allocator_new();
        if(!fdAllocator) {
            g_object_unref(fdMemoryPool);
            return {false, "Failed to create memory pool allocator!"};
        }
        gst_buffer_pool_config_set_allocator(config, fdAllocator, NULL);

        if(!gst_buffer_pool_set_config(fdMemoryPool, config)) {
            g_object_unref(fdMemoryPool);
            return {false, "Failed to set memory pool configuration!"};
        }

        g_object_unref(fdAllocator);

        if(!gst_buffer_pool_is_active(fdMemoryPool) && !gst_buffer_pool_set_active(fdMemoryPool, TRUE)) {
            return {false, "Failed to activate memory pool!"};
        }
    }

    gst_buffer_pool_acquire_buffer(fdMemoryPool, bufferPtr, NULL);
    if(!gst_is_fd_memory(gst_buffer_peek_memory(*bufferPtr, 0))) {
        return {false, "Failed to acquire memory from memory pool!"};
    }

    GstMapInfo map;
    if(gst_buffer_map(*bufferPtr, &map, GST_MAP_READWRITE)) {
        memcpy(map.data, static_cast<void*>(_data), _dataSize);
        gst_buffer_unmap(*bufferPtr, &map);
    } else {
        return {false, "Failed to write to memory!"};
    }

    GstClockTime frame_ts_ns = frame->getTimestampDevice().time_since_epoch().count();

    if(vencFrameBaseNs == GST_CLOCK_TIME_NONE) {
        vencFrameBaseNs = frame_ts_ns;
    }

    frame_ts_ns -= vencFrameBaseNs;

    auto buffer = *bufferPtr;
    buffer->pts = frame_ts_ns;
    buffer->dts = GST_CLOCK_TIME_NONE;
    buffer->offset_end = GST_CLOCK_TIME_NONE;
    buffer->offset = GST_CLOCK_TIME_NONE;
    buffer->duration = GST_CLOCK_TIME_NONE;

    const gint numPlanes = 2;
    gsize offsets[numPlanes];
    gint strides[numPlanes];

    offsets[0] = 0;
    offsets[1] = planeStride;

    strides[0] = stride;
    strides[1] = stride;

    gst_buffer_add_video_meta_full(buffer, GST_VIDEO_FRAME_FLAG_NONE, GST_VIDEO_FORMAT_NV12, width, height, numPlanes, offsets, strides);
    return {true, ""};
}

// void VideoEncoder::send_buffer(std::shared_ptr<dai::Memory> data, const int buffer_size) {
//     auto bitstream_frame = std::make_shared<dai::ImgFrame>();
//     bitstream_frame->setType(dai::ImgFrame::Type::BITSTREAM);
//     // TODO Pass user data with gstreamer
//     int64_t sequenceNumber = 0;
//     bool retValue = sequenceNumbersQueue.waitAndPop(sequenceNumber);
//     if(!retValue){
//         logger->warn("Couldn't get a sequence number for the image");
//     }
//     bitstream_frame->setSequenceNum(sequenceNumber);

//     // memcpy(bitstream_frame->getData().data(), buffer, buffer_size);
//     // zerocopy
//     bitstream_frame->data = data;

//     bitstream.send(bitstream_frame);
//     printf("sent bitstream_frame...\n");
// }

// TODO Move to zero copy version (commented out above)
void VideoEncoder::send_buffer(const uint8_t* buffer, const int buffer_size, const int width, const int height) {
    // TODO Pass user data with gstreamer
    PassthroughMessage passthroughMessage;
    bool retValue = passthroughQueue.waitAndPop(passthroughMessage);
    logger->debug("VideoEncoder latency is {}ms",
                  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - passthroughMessage.tsBeforeVideoEncoder).count());
    if(!retValue) {
        logger->warn("Couldn't get metadata for the image");
    }
    sendBuffer(passthroughMessage, buffer, buffer_size, width, height);
}

void VideoEncoder::sendBuffer(const PassthroughMessage& passthroughMessage, const uint8_t* buffer, const int buffer_size, const int width, const int height) {
    DAI_LOG_TIME_START(sendBuffer);
    if(!bitstream.getConnections().empty()) {
        auto bitstream_frame = std::make_shared<dai::ImgFrame>();
        bitstream_frame->setMetadata(passthroughMessage.frame);
        bitstream_frame->setSize(passthroughMessage.frame.getWidth(), passthroughMessage.frame.getHeight());
        bitstream_frame->data->setSize(buffer_size);
        bitstream_frame->setType(dai::ImgFrame::Type::BITSTREAM);
        memcpy(bitstream_frame->data->getData().data(), buffer, buffer_size);

        {
            auto blockEvent = this->outputBlockEvent();
            bitstream.send(bitstream_frame);
        }
        // printf("sent bitstream_frame...\n");
    }
    if(!out.getConnections().empty()) {
        auto bitstream_frame = std::make_shared<dai::EncodedFrame>();
        bitstream_frame->setBufferMetadataFrom(&passthroughMessage.frame);

        bitstream_frame->cam = passthroughMessage.frame.cam;
        bitstream_frame->instanceNum = passthroughMessage.frame.instanceNum;
        bitstream_frame->transformation = passthroughMessage.frame.transformation;

        bitstream_frame->setSize(passthroughMessage.frame.getWidth(), passthroughMessage.frame.getHeight());

        bitstream_frame->quality = properties.quality;
        bitstream_frame->bitrate = properties.bitrate;
        EncodedFrame::Profile profile = EncodedFrame::Profile::JPEG;
        switch(properties.profile) {
            case VideoEncoderProperties::Profile::H264_BASELINE:
                profile = EncodedFrame::Profile::AVC;
                break;
            case VideoEncoderProperties::Profile::H264_HIGH:
                profile = EncodedFrame::Profile::AVC;
                break;
            case VideoEncoderProperties::Profile::H264_MAIN:
                profile = EncodedFrame::Profile::AVC;
                break;
            case VideoEncoderProperties::Profile::H265_MAIN:
                profile = EncodedFrame::Profile::HEVC;
                break;
            case VideoEncoderProperties::Profile::MJPEG:
                profile = EncodedFrame::Profile::JPEG;
                break;
        }
        bitstream_frame->profile = profile;
        bitstream_frame->lossless = properties.lossless;
        bitstream_frame->type = EncodedFrame::FrameType::Unknown;
        bitstream_frame->frameSize = buffer_size;
        bitstream_frame->data->setSize(buffer_size);
        DAI_LOG_TIME_START(sendBufferMEMCPY);
        memcpy(bitstream_frame->data->getData().data(), buffer, buffer_size);
        DAI_LOG_TIME_END(sendBufferMEMCPY);

        /*
        DAI_LOG_TIME_START(sendBufferSEND);
        out.send(bitstream_frame);
        DAI_LOG_TIME_END(sendBufferSEND);
        */
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(bitstream_frame);
        }
        // printf("sent out...\n");
    }
    this->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", passthroughMessage.frame.getSequenceNum());
    DAI_LOG_TIME_END(sendBuffer);
}

void VideoEncoder::notifyJpegData(JpegJob* job, uint32_t encodedSize) {
    DAI_LOG_TIME_START(notifyJpegData);
    GstMapInfo map;
    auto* buffer = job->outputBuffer;
    auto res = gst_buffer_map(buffer, &map, GST_MAP_READ);
    if(!res) {
        logger->error("Couldn't map output buffer.");
        gst_buffer_pool_release_buffer(outpool, job->outputBuffer);
        delete job;
        return;
    }
    const auto& frame = job->msg.frame;
    // Resize the buffer to the encoded size
    GstMemory* memory = gst_buffer_peek_memory(buffer, 0);
    gsize maxsize = 0;
    gst_memory_get_sizes(memory, 0, &maxsize);
    if(encodedSize < maxsize) {
        gst_memory_resize(memory, 0, encodedSize);
    }
    DEBUG("End compressing, encodedSize: %d", encodedSize);
    try {
        sendBuffer(job->msg, map.data, gst_buffer_get_size(buffer) * sizeof(uint8_t), frame.getWidth(), frame.getHeight());
    } catch(const std::exception& exception) {
        logger->error("Exception in VideoEncoder::sendBuffer(): {}", exception.what());
    } catch(const std::string& exception) {
        logger->error("Exception in VideoEncoder::sendBuffer(): {}", exception);
    } catch(...) {
        logger->error("Exception in VideoEncoder::sendBuffer(): Unknown type of exception");
    }
    gst_buffer_unmap(buffer, &map);
    gst_buffer_pool_release_buffer(outpool, job->outputBuffer);
    delete job;
    DAI_LOG_TIME_END(notifyJpegData);
}

void VideoEncoder::initialize(const int width, const int height, const int fps, dai::ImgFrame::Type type) {
    GstCaps* outcaps = NULL;

    // Set output caps
    outcaps = gst_caps_new_simple("image/jpeg", "width", G_TYPE_INT, width, "height", G_TYPE_INT, height, NULL);
    outpool = createPool(outcaps, width, height);
    DAI_CHECK_IN(outpool);

    // Activate the pool
    if(!gst_buffer_pool_is_active(outpool)) {
        auto res = gst_buffer_pool_set_active(outpool, TRUE);
        DAI_CHECK_IN(res);
    }

    /*
    camera_module_t* camera_module_;
    printf("opening cam module START\n");
    // This is required for proper working of the jpeg lib
    auto ret = hw_get_module(CAMERA_HARDWARE_MODULE_ID, (const hw_module_t**)&camera_module_);
    printf("opening cam module END\n");
    DAI_CHECK_IN(ret == 0 && camera_module_);
    (void)CameraSingleton::getInstance();
    does the above
    */
    (void)CameraSingleton::getInstance();
    DAI_CHECK_IN(jpegEncoder.initialize(width, height));
}

void VideoEncoder::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    // Some sanity checking
    if(properties.bitrate && properties.maxBitrate && properties.rateCtrlMode == dai::VideoEncoderProperties::RateControlMode::VBR) {
        logger->warn("Both bitrate and maxBitrate are set, bitrate will be ignored.");
        properties.bitrate = properties.maxBitrate;
    }
    // jpegenc encodes MJPEG from the mapped camera buffer. qtijpegenc does not
    // return while the camera holds QMMF, and the offline JPEG CHI postproc faults
    // inside DoEncode.
    const auto useGstreamer = true;
    if(useGstreamer) {
        auto pipeline = std::make_shared<GstVaapiEnc>(properties, logger);
        if(pipeline->encoder == nullptr || pipeline->src == nullptr || pipeline->appsink == nullptr) {
            throw std::runtime_error("VideoEncoder failed to create the GStreamer encoder");
        }
        pipeline->register_handle_buffer(&VideoEncoder::send_buffer, this);
        {
            std::lock_guard<std::mutex> lock(encodePipelineMutex);
            encodePipeline = pipeline;
        }
        pipeline->run(true);
    }

    bool first_frame = false;
    while(isRunning()) {
        auto startTs = std::chrono::steady_clock::now();
        std::shared_ptr<dai::ImgFrame> inputFrame = input.get<dai::ImgFrame>();
        auto tGotInput = std::chrono::steady_clock::now();
        this->pipelineEventDispatcher->startTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", inputFrame->getSequenceNum(), startTs);
        DAI_LOG_TIME_START(getFrame);
        DEBUG("VideoEncoder Got frame from queue\n");
        auto passthroughMessage = PassthroughMessage();
        auto tProcessed = std::chrono::steady_clock::now();
        if(useGstreamer) {
            // Need first frame to set the video caps
            if(!first_frame) {
                first_frame = true;
                encodePipeline->set_appsrc_caps(inputFrame->getWidth(), inputFrame->getHeight(), properties.frameRate, inputFrame->getType());
                logger->info("VideoEncoder submitting the first {}x{} frame", inputFrame->getWidth(), inputFrame->getHeight());
            }
            bool error;
            std::string errorMsg;
            std::tie(error, errorMsg) = encodePipeline->push_to_encoder(inputFrame);
            if(!error) {
                throw std::runtime_error(fmt::format("Could not encode the frame, error occured: {}", errorMsg));
            }
            passthroughMessage.tsBeforeVideoEncoder = std::chrono::steady_clock::now();
            passthroughMessage.frame = ImgFrame().setMetadata(*inputFrame);
            passthroughQueue.push(passthroughMessage);
        } else {
            if(!first_frame) {
                first_frame = true;
                initialize(inputFrame->getWidth(), inputFrame->getHeight(), properties.frameRate, inputFrame->getType());
            }
            DAI_CHECK_IN(jpegEncoder.isInitialized());
            auto job = std::make_unique<JpegJob>();
            uint8_t quality = properties.quality;
            DAI_CHECK_IN(outpool);
            GstBuffer* outputBuffer;
            auto rc = gst_buffer_pool_acquire_buffer(outpool, &outputBuffer, NULL);
            DAI_CHECK_IN(rc == GST_FLOW_OK);
            // Copy the flags and timestamps from the input buffer.
            // gst_buffer_copy_into(frame->output_buffer, frame->input_buffer, GST_BUFFER_COPY_FLAGS | GST_BUFFER_COPY_TIMESTAMPS, 0, -1);

            const native_handle_t* inputHandle = nullptr;
            int inputFd = -1;
            auto inData = std::dynamic_pointer_cast<HalBufferMemory>(inputFrame->data);
            if(inData) {
                // keep inData alive until passthroughMessage is destructed after postproc finishes
                passthroughMessage.inputData = inData;
                inputHandle = inData->privHnd;
            } else {
                // Copy frame to new gst buffer as we don't know what format it is in
                GstBuffer* inputBuffer{nullptr};
                bool error;
                std::string errorMsg;
                std::tie(error, errorMsg) = copyFrame(inputFrame, &inputBuffer);
                if(!error) {
                    throw std::runtime_error(fmt::format("Could not copy the frame, error occured: {}", errorMsg));
                }
                GstMemory* inmemory = gst_buffer_peek_memory(inputBuffer, 0);
                inputFd = gst_fd_memory_get_fd(inmemory);
                // keep inputBuffer alive until passthroughMessage is destructed after postproc finishes
                passthroughMessage.fdMemoryPool = fdMemoryPool;
                passthroughMessage.inputGst = inputBuffer;
            }
            GstMemory* outmemory = gst_buffer_peek_memory(outputBuffer, 0);
            const int outputFd = gst_fd_memory_get_fd(outmemory);
            passthroughMessage.tsBeforeVideoEncoder = std::chrono::steady_clock::now();
            passthroughMessage.frame = ImgFrame().setMetadata(*inputFrame);
            DEBUG("Before encode()");
            job->outputBuffer = outputBuffer;
            job->msg = passthroughMessage;
            // From here on the encoder owns the job and hands it back through notifyJpegData().
            JpegJob* jobTag = job.release();
            DAI_LOG_TIME_START(getFrameJpegEncode);
            const bool encoded = (inputHandle != nullptr) ? jpegEncoder.encode(inputHandle, outputFd, inputFrame->getSequenceNum(), quality, jobTag)
                                                          : jpegEncoder.encode(inputFd, outputFd, inputFrame->getSequenceNum(), quality, jobTag);
            DAI_LOG_TIME_END(getFrameJpegEncode);
            DEBUG("After encode()");
            DAI_CHECK_IN(encoded);
        }
        DAI_LOG_TIME_END(getFrame);
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), startTs, tGotInput, tProcessed, tAbsoluteEnd);
    }
}
void VideoEncoder::requestKeyframe() {
    if(properties.profile == VideoEncoderProperties::Profile::MJPEG) return;
    std::lock_guard<std::mutex> lock(encodePipelineMutex);
    if(!encodePipeline || encodePipeline->encoder == nullptr) {
        logger->debug("VideoEncoder keyframe request arrived before the encoder was ready");
        return;
    }
    encodePipeline->requestKeyframe();
}

void VideoEncoder::stop() {
    ThreadedNode::stop();
    std::shared_ptr<GstVaapiEnc> pipeline;
    {
        std::lock_guard<std::mutex> lock(encodePipelineMutex);
        pipeline = encodePipeline;
    }
    if(pipeline != nullptr) {
        pipeline->stop();
    }
}

}  // namespace gate
}  // namespace dai
