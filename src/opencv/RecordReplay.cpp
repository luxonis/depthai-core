#include "../utility/RecordReplayImpl.hpp"

#include <spdlog/spdlog.h>

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <stdexcept>

namespace dai {
namespace utility {

constexpr unsigned int MP4V2_TIMESCALE = 90000;

enum class NALU { P = 1, I = 5, SPS = 7, PPS = 8, INVALID = 0x00 };

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

VideoRecorder::~VideoRecorder() {
    close();
}

void VideoRecorder::init(const std::string& filePath, unsigned int width, unsigned int height, unsigned int fps, VideoCodec codec) {
    if(initialized) {
        throw std::runtime_error("VideoRecorder already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("VideoRecorder file path is empty");
    }
    if(width <= 0 || height <= 0) {
        throw std::runtime_error("VideoRecorder width or height is invalid");
    }
    if(fps <= 0) {
        throw std::runtime_error("VideoRecorder fps is invalid");
    }
    this->codec = codec;
    this->fps = fps;
    this->width = width;
    this->height = height;
    switch(codec) {
        case VideoCodec::H264:
        case VideoCodec::MJPEG:
            mp4Writer = MP4Create(filePath.c_str(), 0);
            if(mp4Writer == MP4_INVALID_FILE_HANDLE) {
                throw std::runtime_error("Failed to create MP4 file");
            }
            MP4SetTimeScale(mp4Writer, MP4V2_TIMESCALE);
            break;
        case VideoCodec::RAW:
            cvWriter = std::make_unique<cv::VideoWriter>();
            cvWriter->open(filePath, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(width, height));
            assert(cvWriter->isOpened());
            break;
    }
    initialized = true;
}

void VideoRecorder::write(span<uint8_t>& data) {
    if(!initialized) {
        throw std::runtime_error("VideoRecorder not initialized");
    }
    switch(this->codec) {
        case VideoCodec::H264: {
            H26xNals nals(data);
            auto nal = nals.next();
            while(!nal.empty()) {
                NALU type = (NALU)(nal[4] & 0x1F);
                switch(type) {
                    case NALU::P:
                    case NALU::I: {
                        if(mp4Track == MP4_INVALID_TRACK_ID) {
                            // spdlog::info("VideoRecorder track is invalid"); // TODO(asahtik) - check if this is OK or should be a warning
                            break;
                        };
                        std::vector<uint8_t> nalData(nal.data(), nal.data() + nal.size());
                        nalData[0] = (nal.size() - 4) >> 24;
                        nalData[1] = (nal.size() - 4) >> 16;
                        nalData[2] = (nal.size() - 4) >> 8;
                        nalData[3] = (nal.size() - 4) & 0xFF;
                        if(!MP4WriteSample(mp4Writer, mp4Track, nalData.data(), nalData.size())) {
                            spdlog::warn("Failed to write sample to MP4 file");
                        }
                        break;
                    }
                    case NALU::SPS:
                        if(mp4Track == MP4_INVALID_TRACK_ID) {
                            mp4Track = MP4AddH264VideoTrack(mp4Writer, MP4V2_TIMESCALE, MP4V2_TIMESCALE / fps, width, height, nal[5], nal[6], nal[7], 3);
                            assert(mp4Track != MP4_INVALID_TRACK_ID);
                            MP4SetVideoProfileLevel(mp4Writer, 0x7F);
                            MP4AddH264SequenceParameterSet(mp4Writer, mp4Track, nal.data(), nal.size());
                        }
                        break;
                    case NALU::PPS:
                        MP4AddH264PictureParameterSet(mp4Writer, mp4Track, nal.data(), nal.size());
                        break;
                    case NALU::INVALID:
                        break;
                }
                nal = nals.next();
            }
            break;
        }
        case VideoCodec::MJPEG:
            if(mp4Track == MP4_INVALID_TRACK_ID) {
                mp4Track = MP4AddVideoTrack(mp4Writer, MP4V2_TIMESCALE, MP4V2_TIMESCALE / fps, width, height, MP4_JPEG_VIDEO_TYPE);
                assert(mp4Track != MP4_INVALID_TRACK_ID);
                MP4SetVideoProfileLevel(mp4Writer, 0x7F);
            } else {
                if(!MP4WriteSample(mp4Writer, mp4Track, data.data(), data.size())) {
                    spdlog::warn("Failed to write sample to MP4 file");
                }
            }
            break;
        case VideoCodec::RAW: {
            if(!cvWriter->isOpened()) {
                throw std::runtime_error("VideoRecorder OpenCV writer is not initialized");
            }
            cv::Mat img(height, width, CV_8UC3, (void*)data.data());
            cvWriter->write(img);
            break;
        }
    }
}

void VideoRecorder::close() {
    if(mp4Writer != MP4_INVALID_FILE_HANDLE) {
        MP4Close(mp4Writer);
    }
    if(cvWriter && cvWriter->isOpened()) {
        cvWriter->release();
    }
}

VideoPlayer::~VideoPlayer() {
    close();
}

void VideoPlayer::init(const std::string& filePath) {
    if(initialized) {
        throw std::runtime_error("VideoPlayer already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("VideoPlayer file path is empty");
    }
    cvReader = std::make_unique<cv::VideoCapture>();
    cvReader->open(filePath);
    assert(cvReader->isOpened());
    initialized = true;
}

void VideoPlayer::setSize(uint32_t width, uint32_t height) {
    this->width = width;
    this->height = height;
}

std::optional<std::vector<uint8_t>> VideoPlayer::next() {
    if(!initialized) {
        throw std::runtime_error("VideoPlayer not initialized");
    }
    cv::Mat frame;
    if(!cvReader->read(frame)) {
        return std::nullopt;
    }
    if(width > 0 && height > 0) {
        cv::resize(frame, frame, cv::Size(width, height));
    } else {
        width = frame.cols;
        height = frame.rows;
    }
    assert(frame.isContinuous());
    std::vector<uint8_t> data;
    data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    return data;
}

std::tuple<uint32_t, uint32_t> VideoPlayer::size() {
    if(!initialized) {
        throw std::runtime_error("VideoPlayer not initialized");
    }
    return {width, height};
}

void VideoPlayer::restart() {
    if(!initialized) {
        throw std::runtime_error("VideoPlayer not initialized");
    }
    cvReader->set(cv::CAP_PROP_POS_FRAMES, 0);
}

void VideoPlayer::close() {
    if(cvReader && cvReader->isOpened()) {
        cvReader->release();
    }
}

}  // namespace utility
}  // namespace dai
