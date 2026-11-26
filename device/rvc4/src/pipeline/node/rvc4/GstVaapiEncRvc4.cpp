#include "GstVaapiEncRvc4.hpp"
// Temporary path, TODO install properly
#include <gst/allocators/gstfdmemory.h>
#include <gst/memory/gstmempool.h>
#include <qmmf-sdk/qmmf_buffer.h>

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <limits>

#include "depthai/pipeline/MessageQueue.hpp"
#include "camera/HalBufferMemory.hpp"
#include "glib-object.h"
#include "gst/video/gstvideometa.h"
#include "gst/video/video-frame.h"
#include "utilities/GstBufferMemory.hpp"

// Error callback function
static void on_error(GstBus* bus, GstMessage* message, gpointer user_data) {
    GError* err;
    gchar* debug_info;
    gst_message_parse_error(message, &err, &debug_info);
    g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(message->src), err->message);
    g_printerr("Debugging information: %s\n", debug_info);

    GstVaapiEnc* that = static_cast<GstVaapiEnc*>(user_data);
    that->error = true;
    that->errorMsg = std::string(err->message);
    if(that->logger) that->logger->error("VideoEncoder GStreamer error from {}: {}", GST_OBJECT_NAME(message->src), err->message);

    g_error_free(err);
    g_free(debug_info);
}

static void on_destroy_buffer(gpointer user_data) {
    auto shared_ptr = static_cast<std::shared_ptr<dai::ImgFrame>*>(user_data);
    delete shared_ptr;
}

static guint get_auto_bitrate(const int width, const int height, const int fps) {
    struct BitrateReference {
        double pixels;
        double bitrate;
    };

    constexpr std::array<BitrateReference, 4> references = {
        {{1280.0 * 720.0, 4.0e6}, {1920.0 * 1080.0, 8.5e6}, {2560.0 * 1440.0, 14.0e6}, {3840.0 * 2160.0, 20.0e6}}};

    const double pixels = static_cast<double>(width) * height;
    double bitrateAt30Fps = 0.0;

    if(pixels <= references.front().pixels) {
        bitrateAt30Fps = references.front().bitrate * pixels / references.front().pixels;
    } else if(pixels >= references.back().pixels) {
        bitrateAt30Fps = references.back().bitrate * pixels / references.back().pixels;
    } else {
        const auto upper = std::upper_bound(
            references.begin(), references.end(), pixels, [](double value, const BitrateReference& reference) { return value < reference.pixels; });
        const auto lower = upper - 1;
        const double ratio = (pixels - lower->pixels) / (upper->pixels - lower->pixels);
        bitrateAt30Fps = lower->bitrate + ratio * (upper->bitrate - lower->bitrate);
    }

    const double bitrate = bitrateAt30Fps * fps / 30.0;
    return static_cast<guint>(std::clamp(bitrate, 0.0, static_cast<double>(std::numeric_limits<guint>::max())));
}

static GstFlowReturn on_new_sample(GstElement* elem, GstVaapiEnc* vaapiEncClass) {
    GstSample* sample = NULL;
    GstBuffer* buffer = NULL;
    GstMapInfo map;
    int sample_width = vaapiEncClass->width;
    int sample_height = vaapiEncClass->height;
    g_signal_emit_by_name(elem, "pull-sample", &sample);
    if(sample) {
        // auto& logger = vaapiEncClass->logger;
        buffer = gst_sample_get_buffer(sample);
        // This doesn' work for MJPEG for some reason
        // GstCaps* caps = gst_sample_get_caps(sample);
        // const GstStructure* info = gst_caps_get_structure(caps, 0);
        // gst_structure_get_int(info, "width", &sample_width);
        // gst_structure_get_int(info, "height", &sample_height);

        gst_buffer_map(buffer, &map, GST_MAP_READ);

        // Codec2 delivers this sample on its own thread. An exception leaving this
        // callback crosses the GStreamer C boundary and aborts the process.
        try {
            vaapiEncClass->handle_buffer(map.data, gst_buffer_get_size(buffer) * sizeof(uint8_t), sample_width, sample_height);
            vaapiEncClass->noteEncodedFrame();
        } catch(const dai::MessageQueue::QueueException& exception) {
            if(vaapiEncClass->logger) {
                vaapiEncClass->logger->debug("VideoEncoder output queue is closed, dropping encoded frame: {}", exception.what());
            }
        } catch(const std::exception& exception) {
            if(vaapiEncClass->logger) {
                vaapiEncClass->logger->error("Exception in VideoEncoder encoded-frame callback: {}", exception.what());
            }
        } catch(...) {
            if(vaapiEncClass->logger) {
                vaapiEncClass->logger->error("Unknown exception in VideoEncoder encoded-frame callback");
            }
        }
        // size_t size = map.size;
        // int fd = gst_dmabuf_memory_get_fd(map.memory);
        gst_buffer_unmap(buffer, &map);

        // // Creates VpuDataMemory and imports fd
        // auto data = std::make_shared<dai::gate::VpuDataMemory>(map.size, fd);

        // vaapiEncClass->handle_buffer(data, gst_buffer_get_size(buffer) * sizeof(uint8_t));

        // Creating GstBufferMemory adds a reference to buffer, but sample still has to be unrefed
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    return GST_FLOW_ERROR;
}
const char* GstVaapiEnc::get_encoder_name_by_video_prop(dai::VideoEncoderProperties::Profile profile) {
    switch(profile) {
        case dai::VideoEncoderProperties::Profile::MJPEG:
            // qtijpegenc opens a QMMF session and does not return while the camera
            // already holds that service. jpegenc encodes the mapped NV12 on the CPU.
            return "jpegenc";
        case dai::VideoEncoderProperties::Profile::H265_MAIN:
        case dai::VideoEncoderProperties::Profile::H264_BASELINE:
        case dai::VideoEncoderProperties::Profile::H264_HIGH:
        case dai::VideoEncoderProperties::Profile::H264_MAIN:
            return "qtic2venc";

        default:
            // TODO(themarpe) - error out instead
            return "qtic2venc";
            break;
    }
}

GstVaapiEnc::GstVaapiEnc(dai::VideoEncoderProperties props, std::shared_ptr<spdlog::async_logger> logger) {
    this->logger = logger;
    this->props = props;

    gst_init(0, NULL);
    const char* encoder_name = get_encoder_name_by_video_prop(props.profile);

    pipeline = gst_pipeline_new(NULL);
    src = gst_element_factory_make("appsrc", NULL);
    encoder = gst_element_factory_make(encoder_name, NULL);
    capsfilter = gst_element_factory_make("capsfilter", NULL);
    appsink = gst_element_factory_make("appsink", NULL);
    videoconvert = nullptr;
    if(props.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
        videoconvert = gst_element_factory_make("videoconvert", NULL);
    }

    if(!src) {
        logger->error("Failed to create appsrc");
        return;
    }
    if(!encoder) {
        logger->error("Failed to create encoder {}", encoder_name);
        return;
    }
    if(!capsfilter) {
        logger->error("Failed to create capsfilter");
        return;
    }
    if(!appsink) {
        logger->error("Failed to create appsink");
        return;
    }
    if(props.profile == dai::VideoEncoderProperties::Profile::MJPEG && videoconvert == nullptr) {
        logger->error("Failed to create videoconvert");
        encoder = nullptr;
        return;
    }

    g_object_set(src, "block", TRUE, "max-buffers", static_cast<guint64>(1), "max-bytes", static_cast<guint64>(0), NULL);
    g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, "max-buffers", static_cast<guint>(1), "enable-last-sample", FALSE, NULL);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), this);
    set_encoder_props(props);
    gboolean linked = FALSE;
    if(videoconvert != nullptr) {
        gst_bin_add_many(GST_BIN(pipeline), src, videoconvert, encoder, capsfilter, appsink, NULL);
        linked = gst_element_link_many(src, videoconvert, encoder, capsfilter, appsink, NULL);
    } else {
        gst_bin_add_many(GST_BIN(pipeline), src, encoder, capsfilter, appsink, NULL);
        linked = gst_element_link_many(src, encoder, capsfilter, appsink, NULL);
    }
    if(!linked) {
        logger->error("VideoEncoder failed to link the {} pipeline", encoder_name);
        encoder = nullptr;
        return;
    }
    bus = gst_element_get_bus(pipeline);
    main_loop = g_main_loop_new(NULL, FALSE);
    // TODO(themarpe) - doesn't work at the moment
    gst_bus_add_signal_watch(bus);
    g_signal_connect(bus, "message::error", G_CALLBACK(on_error), this);
}

void GstVaapiEnc::set_encoder_props(dai::VideoEncoderProperties props) {
    this->props = props;
    uint32_t rateCtrlMode = 1;  // constant
    uint32_t bitrate = props.bitrate;
    std::int32_t qualityClamped = std::clamp(props.quality, 0, 100);
    switch(props.profile) {
        case dai::VideoEncoderProperties::Profile::H264_BASELINE:
        case dai::VideoEncoderProperties::Profile::H264_HIGH:
        case dai::VideoEncoderProperties::Profile::H264_MAIN:
        case dai::VideoEncoderProperties::Profile::H265_MAIN:
            // from gst-inspect-1.0 qtic2venc
            //   control-rate        : Bitrate control method
            //             flags: readable, writable, changeable only in NULL or READY state
            //             Enum "GstC2RateControl" Default: 0, "disable"
            //                (0): disable          - Disable bitrate control
            //                (1): constant         - Constant bitrate
            //                (2): CBR-VFR          - Constant bitrate, variable framerate
            //                (3): VBR-CFR          - Variable bitrate, constant framerate
            //                (4): VBR-VFR          - Variable bitrate, variable framerate
            //                (5): CQ               - Constant quality
            if(props.rateCtrlMode == dai::VideoEncoderProperties::RateControlMode::CBR) {
                rateCtrlMode = 1;  // constant

            } else if(props.rateCtrlMode == dai::VideoEncoderProperties::RateControlMode::VBR) {
                rateCtrlMode = 3;  // VBR-CFR
                if(props.maxBitrate) {
                    bitrate = props.maxBitrate;
                }
            } else {
                rateCtrlMode = 3;  // VBR-CFR
            }

            g_object_set(encoder, "control-rate", rateCtrlMode, NULL);
            // qtic2venc uses UINT_MAX as "component default", while setting any explicit value (including
            // zero) enables its adaptive B-frame path. Preserve the component default when B-frames are disabled.
            if(props.numBFrames > 0) {
                g_object_set(encoder, "b-frames", static_cast<guint>(props.numBFrames), NULL);
            }
            if(props.keyframeFrequency > 0) {
                // Periodicity of IDR frames, in frames
                g_object_set(encoder, "idr-interval", static_cast<guint>(props.keyframeFrequency), NULL);
            }
            // Cap the I-frame QP: the component's rate control over-quantizes keyframes (measured ~1-2 dB
            // PSNR-Y below a QP-30 I-frame at equal size), which also degrades the P-frames referencing them.
            // The cap only sets a quality floor - RC still picks a lower QP when the budget allows. Value 30
            // matches the previously shipped behavior (the old code accidentally clamped I-frame QP to
            // keyframeFrequency, which defaults to 30).
            g_object_set(encoder, "max-quant-i-frames", static_cast<guint>(30), NULL);
            if(props.bitrate) {
                g_object_set(encoder, "target-bitrate", bitrate, NULL);
            }

            // g_object_set(encoder, "resend-pps", 1, NULL);
            // g_object_set(encoder, "resend-sps", 1, NULL);
            // g_object_set(encoder, "send-aud", 1, NULL);
            // g_object_set(encoder, "send-sei", 1, NULL);
            // g_object_set(encoder, "aud", 1, NULL);
            break;
        case dai::VideoEncoderProperties::Profile::MJPEG:
            g_object_set(encoder, "quality", qualityClamped, NULL);
            // TODO Lossless - it seems there is no option for it in GStreamer
            if(props.lossless) {
                logger->error("Lossless encoding is not supported for MJPEG encoder on RVC4");
            }
            break;
        default:
            break;
    }

    // Set downsteam capabilites (caps)
    GstCaps* caps = nullptr;
    switch(props.profile) {
        case dai::VideoEncoderProperties::Profile::H264_BASELINE:
            caps = gst_caps_new_simple("video/x-h264", "profile", G_TYPE_STRING, "baseline", NULL);
            break;
        case dai::VideoEncoderProperties::Profile::H264_HIGH:
            caps = gst_caps_new_simple("video/x-h264", "profile", G_TYPE_STRING, "high", NULL);
            break;
        case dai::VideoEncoderProperties::Profile::H264_MAIN:
            caps = gst_caps_new_simple("video/x-h264", "profile", G_TYPE_STRING, "main", NULL);
            break;
        case dai::VideoEncoderProperties::Profile::H265_MAIN:
            caps = gst_caps_new_simple("video/x-h265", "profile", G_TYPE_STRING, "main", NULL);
            break;
        case dai::VideoEncoderProperties::Profile::MJPEG:
            caps = gst_caps_new_simple("image/jpeg", NULL);
            break;
        default:
            break;
    }

    if(caps != nullptr) {
        g_object_set(capsfilter, "caps", caps, NULL);
    }
}

void GstVaapiEnc::requestKeyframe() {
    if(encoder == nullptr) {
        logger->warn("VideoEncoder cannot force a keyframe before the encoder exists");
        return;
    }
    if(props.profile == dai::VideoEncoderProperties::Profile::MJPEG) return;
    // qtic2venc does not act on GstForceKeyUnit. trigger-iframe sets the Codec2 sync-frame
    // parameter, but the handler dereferences the engine with no null check. The engine is
    // created while the first picture is encoded, so the signal is emitted on a later push.
    keyframeRequested.store(true, std::memory_order_release);
    logger->debug("VideoEncoder queued an IDR request");
}

void GstVaapiEnc::noteEncodedFrame() {
    encoderStarted.store(true, std::memory_order_release);
}

void GstVaapiEnc::emitQueuedKeyframe() {
    if(!encoderStarted.load(std::memory_order_acquire)) return;
    if(!keyframeRequested.load(std::memory_order_acquire)) return;
    // Look the signal up before clearing the request. A missing signal used to drop the IDR for good.
    if(encoder == nullptr || g_signal_lookup("trigger-iframe", G_OBJECT_TYPE(encoder)) == 0) {
        logger->warn("VideoEncoder has no trigger-iframe signal");
        keyframeRequested.store(false, std::memory_order_release);
        return;
    }
    if(!keyframeRequested.exchange(false, std::memory_order_acq_rel)) return;
    // Flush before the signal. A null engine pointer kills the process with no dump.
    // The signal returns gboolean. The plugin reports false when the engine call returns 0,
    // but the next picture is still an IDR, so the return is not a rejection.
    logger->debug("VideoEncoder emitting trigger-iframe");
    logger->flush();
    gboolean triggered = FALSE;
    g_signal_emit_by_name(encoder, "trigger-iframe", &triggered);
    logger->debug("VideoEncoder requested an IDR frame");
}
void GstVaapiEnc::set_appsrc_caps(const int width, const int height, const int fps, dai::ImgFrame::Type type) {
    this->width = width;
    this->height = height;

    // Interpolate the documented 30 FPS bitrate reference points by pixel count, then scale by FPS.
    if(props.bitrate == 0 && props.maxBitrate == 0 && props.profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        const auto autoBitrate = get_auto_bitrate(width, height, fps);
        logger->info("VideoEncoder: automatic bitrate set to {} bit/s for {}x{}@{}", autoBitrate, width, height, fps);
        g_object_set(encoder, "target-bitrate", autoBitrate, NULL);
    }
    std::string formatName;
    switch(type) {
        case dai::ImgFrame::Type::NV12:
            formatName = "NV12";
            break;
        default:
            logger->error("Unsupported input datatype - currently Video Encoder only supports NV12 color format.");
            break;
    }
    printf("Setting the datatype: %s\n", formatName.c_str());

    // jpegenc reads ordinary system memory. qtic2venc requires the GBM feature.
    GstCaps* caps = gst_caps_from_string(props.profile == dai::VideoEncoderProperties::Profile::MJPEG ? "video/x-raw" : "video/x-raw(memory:GBM)");

    gst_caps_set_simple(caps, "format", G_TYPE_STRING, formatName.c_str(), NULL);
    gst_caps_set_simple(caps, "width", G_TYPE_INT, width, NULL);
    gst_caps_set_simple(caps, "height", G_TYPE_INT, height, NULL);
    gst_caps_set_simple(caps, "framerate", GST_TYPE_FRACTION, (int)fps, 1, NULL);  // TODO(themarpe) - fix for fractional FPS
    g_object_set(src, "caps", caps, NULL);
}

std::tuple<bool, std::string> GstVaapiEnc::push_to_encoder(std::shared_ptr<dai::ImgFrame> frame) {
    auto data = frame->data;
    // Check first if pipeline is okay
    if(error) {
        return {false, errorMsg};
    }

    GstFlowReturn ret = GST_FLOW_OK;
    GstAppSrc* appsrc = reinterpret_cast<GstAppSrc*>(src);
    GstBuffer* buffer = nullptr;

    if(auto ptr = std::dynamic_pointer_cast<dai::gate::GstBufferMemory>(data); ptr != nullptr) {
        printf("retrieved a GstBufferMemory, convert & push forward\n");
        buffer = gst_buffer_ref(ptr->buf);
    } else {
        void* _data = data->getData().data();
        size_t _dataSize = data->getData().size();
        int width = frame->getWidth();
        int height = frame->getHeight();
        int stride = frame->getStride();
        int planeStride = frame->getPlaneStride();

        // qtic2venc imports the camera dmabuf. jpegenc reads the mapping HalBufferMemory already made.
        auto* halMem = dynamic_cast<dai::gate::HalBufferMemory*>(data.get());
        if(props.profile != dai::VideoEncoderProperties::Profile::MJPEG && halMem != nullptr && halMem->privHnd != nullptr && halMem->privHnd->fd >= 0) {
            if(inputFdAllocator == nullptr) {
                inputFdAllocator = gst_fd_allocator_new();
                if(inputFdAllocator == nullptr) {
                    return {false, "Failed to create input FD allocator!"};
                }
            }
            GstMemory* mem = gst_fd_allocator_alloc(
                inputFdAllocator, halMem->privHnd->fd, halMem->privHnd->size, static_cast<GstFdMemoryFlags>(GST_FD_MEMORY_FLAG_DONT_CLOSE));
            if(mem == nullptr) {
                return {false, "Failed to import input dmabuf!"};
            }
            buffer = gst_buffer_new();
            if(buffer == nullptr) {
                gst_memory_unref(mem);
                return {false, "Failed to create input buffer!"};
            }
            gst_buffer_append_memory(buffer, mem);
            // Keep the camera buffer alive until GStreamer releases its non-owning dmabuf import.
            gst_mini_object_set_qdata(
                GST_MINI_OBJECT(buffer), g_quark_from_static_string("dai-imgframe"), new std::shared_ptr<dai::ImgFrame>(frame), on_destroy_buffer);
        } else {
            buffer = gst_buffer_new_wrapped_full((GstMemoryFlags)(GST_MEMORY_FLAG_READONLY | GST_MEMORY_FLAG_PHYSICALLY_CONTIGUOUS),
                                                 _data,
                                                 _dataSize,
                                                 0,
                                                 _dataSize,
                                                 new std::shared_ptr<dai::ImgFrame>(frame),
                                                 on_destroy_buffer);
        }

        GstClockTime frame_ts_ns = frame->getTimestampDevice().time_since_epoch().count();

        if(vencFrameBaseNs == GST_CLOCK_TIME_NONE) {
            vencFrameBaseNs = frame_ts_ns;
        }

        frame_ts_ns -= vencFrameBaseNs;

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
    }

    // g_signal_emit_by_name(src, "push-buffer", buffer, &ret);
    emitQueuedKeyframe();
    ret = gst_app_src_push_buffer(appsrc, buffer);  // buffer released automatically
    if(ret != GST_FLOW_OK) {
        printf("Couldn't push to appsrc, ret: %d\n", ret);
        return {false, "Could not push to appsrc"};
    }

    return {true, ""};
}

void GstVaapiEnc::run(bool run_in_thread) {
    // Debug: dump graph
    gst_debug_bin_to_dot_file(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "encoder");

    logger->info("VideoEncoder setting the pipeline to PLAYING");
    const auto change = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    logger->info("VideoEncoder pipeline state change returned {}", static_cast<int>(change));

    // run in thread
    if(run_in_thread) {
        g_thread_new("GstreamerLoop", (GThreadFunc)g_main_loop_run, main_loop);
    } else {
        g_main_loop_run(main_loop);
    }
}
void GstVaapiEnc::stop() {
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    if(inputFdAllocator != nullptr) {
        gst_object_unref(inputFdAllocator);
        inputFdAllocator = nullptr;
    }
    gst_object_unref(pipeline);
    g_main_loop_quit(main_loop);
}
