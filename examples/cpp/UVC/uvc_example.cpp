#include <atomic>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
// #include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "uvc_example.hpp"

extern "C" {
#include "uvc-gadget/lib/video-buffers.h"
#include "uvcgadget/configfs.h"
#include "uvcgadget/events.h"
#include "uvcgadget/stream.h"
#include "uvcgadget/libcamera-source.h"
#include "uvcgadget/v4l2-source.h"
#include "uvcgadget/test-source.h"
#include "uvcgadget/jpg-source.h"
#include "uvcgadget/slideshow-source.h"
#include "uvcgadget/depthai-source.h"
}

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

std::shared_ptr<dai::MessageQueue> outputQueue;

/* Necessary for and only used by signal handler. */
static struct events *sigint_events;

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;

	/* Stop the main loop when the user presses CTRL-C */
	events_stop(sigint_events);
}

// Custom host node for saving video data
class VideoSaver : public dai::node::CustomNode<VideoSaver> {
   public:
    VideoSaver() : fileHandle("video.encoded", std::ios::binary) {
        if(!fileHandle.is_open()) {
            throw std::runtime_error("Could not open video.encoded for writing");
        }
    }

    ~VideoSaver() {
        if(fileHandle.is_open()) {
            fileHandle.close();
        }
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> message) override {
        if(!fileHandle.is_open()) return nullptr;

        // Get raw data and write to file
        auto frame = message->get<dai::EncodedFrame>("data");
        unsigned char* frameData = frame->getData().data();
        size_t frameSize = frame->getData().size();
        std::cout << "Storing frame of size: " << frameSize << std::endl;
        fileHandle.write(reinterpret_cast<const char*>(frameData), frameSize);

        // Don't send anything back
        return nullptr;
    }

   private:
    std::ofstream fileHandle;
};

extern "C" void depthai_uvc_get_buffer(struct video_source *s, struct video_buffer *buf) {
	unsigned int frame_size, size;
    uint8_t *f;

    if(quitEvent) {
        std::cout << "depthai_uvc_get_buffer(): Stopping capture due to quit event." << std::endl;
        return;
    }      

    auto frame = outputQueue->get<dai::ImgFrame>();
    if(frame == nullptr) {
        std::cerr << "depthai_uvc_get_buffer(): No frame available." << std::endl;
        return;
    }

    f = frame->getData().data();
    frame_size = frame->getData().size();

	size = std::min(frame_size, buf->size);
	memcpy(buf->mem, f, size);
	buf->bytesused = size;
}

int main() {
    struct events events;
    struct uvc_function_config *fc;
    struct video_source* src;
    struct uvc_stream* stream;

    depthai_uvc_register_get_buffer(depthai_uvc_get_buffer);

    fc = configfs_parse_uvc_function("uvc.0");
    if (!fc) {
        std::cerr << "Failed to parse UVC function configuration." << std::endl;
        return 1;
    }

    events_init(&events);
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    src = depthai_video_source_create();
    if (!src) {
        std::cerr << "Failed to create video source." << std::endl;
        return 1;
    }

    stream = uvc_stream_new(fc->video);
    if (!stream) {
        std::cerr << "Failed to create UVC stream." << std::endl;
        video_source_destroy(src);
        return 1;
    }

	uvc_stream_set_event_handler(stream, &events);
	uvc_stream_set_video_source(stream, src);
	uvc_stream_init_uvc(stream, fc);

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto output = camRgb->requestOutput(std::make_pair(1920, 1080), dai::ImgFrame::Type::NV12);

    // Create video encoder node
    auto encoded = pipeline.create<dai::node::VideoEncoder>();
    encoded->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    encoded->input.setMaxSize(1);
    encoded->input.setBlocking(false);
    output->link(encoded->input);
    outputQueue = encoded->bitstream.createOutputQueue();

    // Start pipeline
    pipeline.start();
    std::cout << "Started the pipeline" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    auto timeStart = std::chrono::steady_clock::now();

	/* Main capture loop */
	events_loop(&events);

    // Cleanup
    pipeline.stop();
    pipeline.wait();

	uvc_stream_delete(stream);
	video_source_destroy(src);
	events_cleanup(&events);
	configfs_free_uvc_function(fc);
    
    std::cout << "Video capture stopped." << std::endl;
    return 0;
}