#pragma once
#include <gstreamer-1.0/gst/app/gstappsink.h>
#include <gstreamer-1.0/gst/gst.h>

#include <cstdint>
#include <functional>
#include <memory>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"

#include "depthai/utility/Memory.hpp"

class GstMonoCamera {
   public:
    GMainLoop* main_loop;
    GstElement* pipeline;
    std::vector<GstElement*> src;
    GstBus* bus;
    std::unordered_map<dai::CameraBoardSocket, int> socketToGstSrcIdx;
    GstMonoCamera(std::vector<dai::MonoCameraProperties> props);
    struct GstCallbackContext {
        GstMonoCamera* webcamClass;
        dai::CameraBoardSocket socket;
    };
    using Callback = std::function<void(std::shared_ptr<dai::Memory> data,
                                        const int width,
                                        const int height,
                                        const int size,
                                        const gchar* format,
                                        dai::CameraBoardSocket socket,
                                        dai::Timestamp ts,
                                        int stride,
                                        int chromaPlaneOffset)>;
    std::unordered_map<dai::CameraBoardSocket, Callback> callback;
    std::unordered_map<dai::CameraBoardSocket, std::shared_ptr<GstCallbackContext>> context;

    struct V4lDesc {
        int cameraFd = -1;
        int sensorFd = -1;
        int lensFd = -1;
    };
    std::unordered_map<dai::CameraBoardSocket, V4lDesc> v4ldesc;

    void run(std::vector<dai::MonoCameraProperties> props);
    void stop();

    void applyCameraControl(dai::CameraBoardSocket socket, dai::CameraControl control);
};
