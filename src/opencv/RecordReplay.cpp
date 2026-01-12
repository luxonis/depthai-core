#include <spdlog/spdlog.h>

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <stdexcept>

#include "../utility/RecordReplayImpl.hpp"

namespace dai {
namespace utility {

constexpr unsigned int MP4V2_TIMESCALE = 90000;

enum class NALU { P = 1, I = 5, SPS = 7, PPS = 8, INVALID = 0x00 };

struct NaluView {
    dai::span<const uint8_t> data;
    NALU type = NALU::INVALID;
};

struct H26xNals {
    dai::span<const uint8_t> data;
    size_t index = 0;

    H26xNals(dai::span<const uint8_t> data) : data(data) {}

    static bool findStartCode(dai::span<const uint8_t> data, size_t offset, size_t& start, size_t& startCodePos) {
        for(size_t i = offset; i + 3 < data.size(); ++i) {
            if(data[i] == 0 && data[i + 1] == 0) {
                if(data[i + 2] == 1) {
                    startCodePos = i;
                    start = i + 3;
                    return true;
                }
                if(data[i + 2] == 0 && data[i + 3] == 1) {
                    startCodePos = i;
                    start = i + 4;
                    return true;
                }
            }
        }
        return false;
    }

    std::optional<NaluView> next() {
        if(index >= data.size()) return std::nullopt;
        size_t start = 0;
        size_t startCodePos = 0;
        if(!findStartCode(data, index, start, startCodePos)) {
            if(index == 0 && !data.empty()) {
                index = data.size();
                return NaluView{data, static_cast<NALU>(data[0] & 0x1F)};
            }
            return std::nullopt;
        }
        size_t nextStart = 0;
        size_t nextStartCodePos = 0;
        size_t end = data.size();
        if(findStartCode(data, start, nextStart, nextStartCodePos)) {
            end = nextStartCodePos;
        }
        if(end <= start) {
            index = end;
            return std::nullopt;
        }
        index = end;
        auto nal = data.subspan(start, end - start);
        return NaluView{nal, static_cast<NALU>(nal[0] & 0x1F)};
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
#ifdef DEPTHAI_ENABLE_MP4V2
            mp4Writer = MP4Create(filePath.c_str());
            if(mp4Writer == MP4_INVALID_FILE_HANDLE) {
                throw std::runtime_error("Failed to create MP4 file");
            }
            MP4SetTimeScale(mp4Writer, MP4V2_TIMESCALE);
#else
            throw std::runtime_error("Encoded video not support. Please recompile with DEPTHAI_ENABLE_MP4V2=ON");
#endif
            break;
        case VideoCodec::RAW:
            cvWriter = std::make_unique<cv::VideoWriter>();
            cvWriter->open(filePath, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps, cv::Size(width, height));
            // TODO - It would be better to use a lossless codec here
            // cvWriter->open(filePath, cv::VideoWriter::fourcc('F','F','V','1'), fps, cv::Size(width, height));
            assert(cvWriter->isOpened());
            break;
    }
    initialized = true;
}

void VideoRecorder::write(span<uint8_t>& data, const uint32_t stride) {
    if(!initialized) {
        throw std::runtime_error("VideoRecorder not initialized");
    }
    switch(this->codec) {
#ifdef DEPTHAI_ENABLE_MP4V2
        case VideoCodec::H264: {
            H26xNals nals(data);
            std::vector<dai::span<const uint8_t>> sampleNals;
            bool isSyncSample = false;
            auto nal = nals.next();
            while(nal.has_value()) {
                auto nalData = nal->data;
                switch(nal->type) {
                    case NALU::SPS:
                        if(mp4Track == MP4_INVALID_TRACK_ID && nalData.size() >= 4) {
                            mp4Track =
                                MP4AddH264VideoTrack(mp4Writer, MP4V2_TIMESCALE, MP4V2_TIMESCALE / fps, width, height, nalData[1], nalData[2], nalData[3], 3);
                            assert(mp4Track != MP4_INVALID_TRACK_ID);
                            MP4SetVideoProfileLevel(mp4Writer, 0x7F);
                        }
                        if(mp4Track != MP4_INVALID_TRACK_ID) {
                            MP4AddH264SequenceParameterSet(mp4Writer, mp4Track, nalData.data(), nalData.size());
                        }
                        break;
                    case NALU::PPS:
                        if(mp4Track != MP4_INVALID_TRACK_ID) {
                            MP4AddH264PictureParameterSet(mp4Writer, mp4Track, nalData.data(), nalData.size());
                        }
                        break;
                    case NALU::I:
                        isSyncSample = true;
                        sampleNals.push_back(nalData);
                        break;
                    case NALU::P:
                        sampleNals.push_back(nalData);
                        break;
                    case NALU::INVALID:
                    default:
                        break;
                }
                nal = nals.next();
            }

            if(mp4Track == MP4_INVALID_TRACK_ID) {
                break;
            }
            if(sampleNals.empty()) {
                break;
            }

            size_t totalSize = 0;
            for(const auto& n : sampleNals) {
                totalSize += 4 + n.size();
            }
            std::vector<uint8_t> sampleData;
            sampleData.reserve(totalSize);
            for(const auto& n : sampleNals) {
                uint32_t naluSize = static_cast<uint32_t>(n.size());
                sampleData.push_back(static_cast<uint8_t>((naluSize >> 24) & 0xFF));
                sampleData.push_back(static_cast<uint8_t>((naluSize >> 16) & 0xFF));
                sampleData.push_back(static_cast<uint8_t>((naluSize >> 8) & 0xFF));
                sampleData.push_back(static_cast<uint8_t>(naluSize & 0xFF));
                sampleData.insert(sampleData.end(), n.begin(), n.end());
            }

            if(!MP4WriteSample(mp4Writer, mp4Track, sampleData.data(), sampleData.size(), MP4V2_TIMESCALE / fps, 0, isSyncSample)) {
                spdlog::warn("Failed to write sample to MP4 file");
            }
            break;
        }
        case VideoCodec::MJPEG:
            if(mp4Track == MP4_INVALID_TRACK_ID) {
                mp4Track = MP4AddVideoTrack(mp4Writer, MP4V2_TIMESCALE, MP4V2_TIMESCALE / fps, width, height, MP4_JPEG_VIDEO_TYPE);
                assert(mp4Track != MP4_INVALID_TRACK_ID);
                MP4SetVideoProfileLevel(mp4Writer, 0x7F);
            }
            if(!MP4WriteSample(mp4Writer, mp4Track, data.data(), data.size(), MP4V2_TIMESCALE / fps)) {
                spdlog::warn("Failed to write sample to MP4 file");
            }
            break;
#else
        case VideoCodec::H264:
        case VideoCodec::MJPEG: {
            throw std::runtime_error("Encoded video not support. Please recompile with DEPTHAI_ENABLE_MP4V2=ON");
            break;
        }
#endif
        case VideoCodec::RAW: {
            if(!cvWriter->isOpened()) {
                throw std::runtime_error("VideoRecorder OpenCV writer is not initialized");
            }
            if(stride > 0) {
                cv::Mat img(height, width, CV_8UC3, (void*)data.data(), stride);
                cvWriter->write(img);
            } else {
                cv::Mat img(height, width, CV_8UC3, (void*)data.data());
                cvWriter->write(img);
            }
            break;
        }
    }
}

void VideoRecorder::close() {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(mp4Writer != MP4_INVALID_FILE_HANDLE) {
        // MP4Dump(mp4Writer);
        MP4Close(mp4Writer);
    }
#endif
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
    cvReader->open(filePath, cv::CAP_FFMPEG);
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

std::tuple<size_t, size_t> getVideoSize(const std::string& filePath) {
    cv::VideoCapture cvReader;
    cvReader.open(filePath);
    if(!cvReader.isOpened()) {
        throw std::runtime_error("Failed to open video file");
    }
    auto width = cvReader.get(cv::CAP_PROP_FRAME_WIDTH);
    auto height = cvReader.get(cv::CAP_PROP_FRAME_HEIGHT);
    cvReader.release();
    return {width, height};
}

}  // namespace utility
}  // namespace dai
