#include <spdlog/spdlog.h>

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <stdexcept>

#include "../utility/RecordReplayImpl.hpp"

namespace dai {
namespace utility {

VideoRecorder::~VideoRecorder() {
    try {
        close();
    } catch(const std::exception&) {
        // Explicit close reports write errors. Destructors must still release the writer.
    }
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
    std::filesystem::path pathObj(filePath);
    this->codec = codec;
    this->fps = fps;
    this->width = width;
    this->height = height;
    switch(codec) {
        case VideoCodec::H264:
        case VideoCodec::MJPEG:
            encodedWriter = std::make_unique<EncodedVideoRecorder>();
            encodedWriter->init(filePath, width, height, fps, codec == VideoCodec::MJPEG);
            break;
        case VideoCodec::RAW:
            if(pathObj.extension() != ".avi" && pathObj.extension() != ".mkv") {
                throw std::runtime_error("Raw video can only be recorded in .avi or .mkv format");
            }
            cvWriter = std::make_unique<cv::VideoWriter>();
            cvWriter->open(filePath, cv::VideoWriter::fourcc('F', 'F', 'V', '1'), fps, cv::Size(width, height));
            assert(cvWriter->isOpened());
            break;
    }
    initialized = true;
}

void VideoRecorder::setH264ParameterSets(span<const std::uint8_t> sps, span<const std::uint8_t> pps) {
    if(!encodedWriter) throw std::runtime_error("VideoRecorder has no H264 writer");
    encodedWriter->setH264ParameterSets(sps, pps);
}

void VideoRecorder::write(span<uint8_t>& data, const uint32_t stride) {
    if(!initialized) {
        throw std::runtime_error("VideoRecorder not initialized");
    }
    switch(this->codec) {
        case VideoCodec::H264:
        case VideoCodec::MJPEG:
            encodedWriter->write(data);
            break;
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
    if(encodedWriter) {
        encodedWriter->close();
        encodedWriter.reset();
    }
    if(cvWriter && cvWriter->isOpened()) {
        cvWriter->release();
    }
    initialized = false;
}

VideoPlayer::~VideoPlayer() {
    close();
}

void VideoPlayer::init(const std::string& filePath) {
    if(initialized) {
        throw std::runtime_error("VideoPlayer already initialized");
    }
    // Check if file exists
    if(filePath.empty()) {
        throw std::runtime_error("VideoPlayer file path is empty");
    }
    if(!std::filesystem::is_regular_file(filePath)) {
        throw std::runtime_error("VideoPlayer file does not exist: " + filePath);
    }
    cvReader = std::make_unique<cv::VideoCapture>();
    if(!cvReader->open(filePath, cv::CAP_FFMPEG) || !cvReader->isOpened()) {
        throw std::runtime_error("Failed to open video with FFMPEG: " + filePath);
    }
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

std::tuple<size_t, size_t, double> getVideoSize(const std::string& filePath) {
    cv::VideoCapture cvReader;
    cvReader.open(filePath, cv::CAP_FFMPEG);
    if(!cvReader.isOpened()) {
        throw std::runtime_error("Failed to open video file");
    }
    auto width = cvReader.get(cv::CAP_PROP_FRAME_WIDTH);
    auto height = cvReader.get(cv::CAP_PROP_FRAME_HEIGHT);
    auto fps = cvReader.get(cv::CAP_PROP_FPS);
    cvReader.release();
    return {width, height, fps};
}

}  // namespace utility
}  // namespace dai
