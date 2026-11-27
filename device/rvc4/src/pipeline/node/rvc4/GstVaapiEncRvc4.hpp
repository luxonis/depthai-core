#pragma once

// std
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <tuple>

// project
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/Memory.hpp"
#include "utilities/GstBufferMemory.hpp"

// libraries
#include <gstreamer-1.0/gst/app/gstappsink.h>
#include <gstreamer-1.0/gst/app/gstappsrc.h>
#include <gstreamer-1.0/gst/gst.h>
#include <spdlog/async_logger.h>

class GstVaapiEnc {
   public:
    GMainLoop* main_loop;
    GstElement* pipeline;
    GstElement* src;
    GstElement* videoconvert;
    GstElement* encoder;
    GstElement* capsfilter;
    GstElement* appsink;
    GstBus* bus;
    GstAllocator* inputFdAllocator = nullptr;
    std::shared_ptr<spdlog::async_logger> logger;

    GstVaapiEnc(dai::VideoEncoderProperties props, std::shared_ptr<spdlog::async_logger> logger);

    const char* get_encoder_name_by_video_prop(dai::VideoEncoderProperties::Profile profile);

    // std::function<void(std::shared_ptr<dai::Memory> data, const int size)> handle_buffer;
    std::function<void(const uint8_t* buffer, const int size, const int width, const int height)> handle_buffer;

    std::atomic<bool> error{false};
    std::string errorMsg{""};

    template <typename A, typename B>
    void register_handle_buffer(A func_ptr, B obj_ptr) {
        handle_buffer = std::bind(func_ptr, obj_ptr, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    }
    void set_appsrc_caps(const int width, const int height, const int fps, dai::ImgFrame::Type type);
    void set_encoder_props(dai::VideoEncoderProperties props);
    // Ask the encoder to emit an IDR, with SPS and PPS, on a later frame.
    void requestKeyframe();
    // The encoder has produced a buffer, so its Codec2 engine exists.
    void noteEncodedFrame();
    std::tuple<bool, std::string> push_to_encoder(std::shared_ptr<dai::ImgFrame> frame);
    void run(bool run_in_thread);
    void stop();

    dai::VideoEncoderProperties props;
    int width, height;

   private:
    GstClockTime vencFrameBaseNs = GST_CLOCK_TIME_NONE;
    // trigger-iframe faults if it runs before the first encoded frame.
    std::atomic<bool> keyframeRequested{false};
    std::atomic<bool> encoderStarted{false};

    void emitQueuedKeyframe();
};
