#include <chrono>
#include <opencv2/highgui.hpp>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    // Encode camera frames as MJPEG on the device before transferring them to the host.
    auto encoder = pipeline.create<dai::node::VideoEncoder>();
    // Use H264_BASELINE, H264_MAIN or H264_HIGH with DEPTHAI_ENABLE_OPENH264.
    encoder->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    camera->requestOutput({640, 400}, dai::ImgFrame::Type::NV12)->link(encoder->input);

    // The encoded stream is decoded on the host using the SDK's OpenCV support.
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    decoder->setOutFrameType(dai::ImgFrame::Type::BGR888i);
    encoder->out.link(decoder->input);
    auto frames = decoder->out.createOutputQueue();

    // Display decoded frames. Press q to exit.
    pipeline.start();
    while(pipeline.isRunning()) {
        bool timedOut = false;
        auto frame = frames->get<dai::ImgFrame>(std::chrono::seconds(1), timedOut);
        if(frame) cv::imshow("Decoded MJPEG", frame->getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }
    pipeline.stop();
    pipeline.wait();
    cv::destroyAllWindows();
}
