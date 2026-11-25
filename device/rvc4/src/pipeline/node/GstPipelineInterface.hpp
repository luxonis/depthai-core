#pragma once
#include <stdint.h>
#include <functional>
#include <gstreamer-1.0/gst/gst.h>
#include <gstreamer-1.0/gst/app/gstappsink.h>
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/utility/Memory.hpp"

class GstPipelineInterface{
    public:
        GMainLoop *main_loop;
        GstElement *pipeline;
        GstElement *src;
        GstBus *bus;

        GstPipelineInterface() {
        }

        std::function<void(std::shared_ptr<dai::Memory>, const int width, const int height,
                        const int size, const gchar* format, dai::CameraBoardSocket socket, dai::Timestamp ts )> handle_buffer;

        template<typename A, typename B>
        void register_handle_buffer(A func_ptr, B obj_ptr)
        {
            handle_buffer = std::bind(func_ptr, obj_ptr,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4,
                        std::placeholders::_5,
                        std::placeholders::_6,
                        std::placeholders::_7);
        }

        virtual void setup_video_chain(const int fps, const uint32_t width, const uint32_t height) {}
        virtual void setup_preview_chain(const int fps, const uint32_t width, const uint32_t height,
                                    const char* colorFormat) {}
        virtual void setup_still_chain() {}
        virtual void setup_isp_chain(const int fps, const uint32_t width, const uint32_t height) {}
        virtual void setup_raw_chain(const int fps) {}

        void run(bool run_in_thread=false);
        void stop();
};