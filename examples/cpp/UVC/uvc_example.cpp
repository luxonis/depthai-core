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
#include "depthai/pipeline/datatype/MessageGroup.hpp"

extern "C" {
#include "config.h"
#include "configfs.h"
#include "events.h"
#include "stream.h"
#include "libcamera-source.h"
#include "v4l2-source.h"
#include "test-source.h"
#include "jpg-source.h"
#include "slideshow-source.h"
}

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
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

int depthai_uvc_get_buffer(struct video_source *s, struct video_buffer *buf) {
    struct depthai_source *src = to_depthai_source(s);
	unsigned int size;

    if(!pipeline.isRunning() || quitEvent) {
        return -1;
    }      

    auto frame = outputQueue->get<dai::ImgFrame>();
    if(frame == nullptr) {
        return -1;
    }

	size = min(src->imgsize, buf->size);
	memcpy(buf->mem, src->imgdata, size);
	buf->bytesused = size;

    std::cout << "Filled a buffer" << std::endl;
}

int main() {
    struct events events;
    struct uvc_function_config *fc;
    struct video_source* src;
    struct uvc_stream* stream;

    fc = configfs_parse_uvc_function("uvc.0");
    if (!fc) {
        std::cerr << "Failed to parse UVC function configuration." << std::endl;
        return 1;
    }

    events_init(&events);
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    src = depthai_video_source_create(fc->video->img_path);
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
    auto output = camRgb->requestOutput(std::make_pair(1920, 1440), dai::ImgFrame::Type::NV12);
    auto outputQueue = output->createOutputQueue();

    // Create video encoder node
    auto encoded = pipeline.create<dai::node::VideoEncoder>();
    encoded->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    output->link(encoded->input);

    // Create video saver node
    auto saver = pipeline.create<VideoSaver>();
    encoded->out.link(saver->inputs["data"]);

    // Start pipeline
    pipeline.start();
    std::cout << "Started to save video to video.encoded" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    auto timeStart = std::chrono::steady_clock::now();

    while(pipeline.isRunning() && !quitEvent) {
        auto frame = outputQueue->get<dai::ImgFrame>();
        if(frame == nullptr) continue;
        
        depthai_source_fill_buffer(src, frame->getData().data(), frame->getData().size());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Simulate processing delay
        std::cout << "I'm running yo!" << std::endl;
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    std::cout << "To view the encoded data, convert the stream file (.encoded) into a video file (.mp4) using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate 30 -i video.encoded -c copy video.mp4" << std::endl;
}