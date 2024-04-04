#include <mp4v2/mp4v2.h>

#include <chrono>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/span.hpp"

constexpr unsigned int TIMESCALE = 90000;

int main() {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setFps(30);

    xout->setStreamName("img");

    cam->video.link(xout->input);

    dai::Device device(pipeline);

    // MP4 stuff
    MP4FileHandle mp4 = MP4Create("output.mp4", 0);
    MP4TrackId track = MP4_INVALID_TRACK_ID;
    float fps = 0.f;
    assert(mp4 != MP4_INVALID_FILE_HANDLE);
    MP4SetTimeScale(mp4, TIMESCALE);

    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    for(int i = 0; i < 310; i++) {
        auto mjpeg = device.getOutputQueue("img")->get<dai::ImgFrame>();
        if(i == 0)
            start = mjpeg->getTimestampDevice();
        else if(i == 9)
            end = mjpeg->getTimestampDevice();
        else if(i == 10) {
            fps = 10e6f / (float)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::cout << "FPS: " << fps << std::endl;
            track = MP4AddVideoTrack(mp4, TIMESCALE, TIMESCALE / fps, 1920, 1080, MP4_JPEG_VIDEO_TYPE);
            assert(track != MP4_INVALID_TRACK_ID);
            MP4SetVideoProfileLevel(mp4, 0x7F);
        }
        if(i > 9) {
            std::cout << "Write frame\n";
            if (track == MP4_INVALID_TRACK_ID) continue;
            assert(MP4WriteSample(mp4, track, mjpeg->getData().data(), mjpeg->getData().size()));
        }
    }
    MP4Close(mp4);
}
