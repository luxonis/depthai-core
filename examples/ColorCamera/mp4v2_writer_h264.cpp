#include <mp4v2/mp4v2.h>

#include <chrono>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/span.hpp"

enum class NALU { P = 1, I = 5, SPS = 7, PPS = 8, INVALID = 0x00 };

constexpr unsigned int TIMESCALE = 90000;

struct H26xNals {
    dai::span<const uint8_t> data;
    size_t index = 0;

    H26xNals(dai::span<const uint8_t> data) : data(data) {}

    dai::span<const uint8_t> next() {
        if(index >= data.size()) return {};
        while(index < data.size() - 4) {
            if(data[index] == 0 && data[index + 1] == 0 && data[index + 2] == 0 && data[index + 3] == 1) {
                auto nal = data.subspan(index, data.size() - index);
                index += 4;
                return nal;
            }
            ++index;
        }
        return {};
    }
};

int main() {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setFps(30);

    videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

    xout->setStreamName("h264");

    cam->video.link(videoEncoder->input);
    videoEncoder->out.link(xout->input);

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
        auto h264 = device.getOutputQueue("h264")->get<dai::EncodedFrame>();
        if(i == 0) {
            start = h264->getTimestampDevice();
            std::cout << "Got frame with size:" << h264->getWidth() << "x" << h264->getHeight() << std::endl;
        } else if(i == 9) {
            end = h264->getTimestampDevice();
            fps = 10e6f / (float)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::cout << "FPS: " << fps << std::endl;
        }
        if(i >= 9) {
            H26xNals nals(h264->getData());
            auto nal = nals.next();
            while(!nal.empty()) {
                NALU type = (NALU)(nal[4] & 0x1F);
                switch(type) {
                    case NALU::P:
                    case NALU::I: {
                        std::cout << "Write frame\n";
                        if (track == MP4_INVALID_TRACK_ID) break;
                        std::vector<uint8_t> data(nal.data(), nal.data() + nal.size());
                        data[0] = (nal.size() - 4) >> 24;
                        data[1] = (nal.size() - 4) >> 16;
                        data[2] = (nal.size() - 4) >> 8;
                        data[3] = (nal.size() - 4) & 0xFF;
                        assert(MP4WriteSample(mp4, track, data.data(), data.size()));
                        break;
                    }
                    case NALU::SPS:
                        std::cout << "Write SPS\n";
                        if(track == MP4_INVALID_TRACK_ID) {
                            track = MP4AddH264VideoTrack(mp4, TIMESCALE, TIMESCALE / fps, 1920, 1080, nal[5], nal[6], nal[7], 3);
                            assert(track != MP4_INVALID_TRACK_ID);
                            MP4SetVideoProfileLevel(mp4, 0x7F);
                            MP4AddH264SequenceParameterSet(mp4, track, nal.data(), nal.size());
                        }
                        break;
                    case NALU::PPS:
                        std::cout << "Write PPS\n";
                        MP4AddH264PictureParameterSet(mp4, track, nal.data(), nal.size());
                        break;
                    case NALU::INVALID:
                        break;
                }
                nal = nals.next();
            }
        }
    }
    MP4Close(mp4);
}
