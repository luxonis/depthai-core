#include "RecordReplayImpl.hpp"

#include <stdexcept>

// Encoded recording also works in builds without OpenCV. The raw-video backend
// remains in opencv/RecordReplay.cpp, which supplies these methods in host builds.
namespace dai {
namespace utility {

VideoRecorder::~VideoRecorder() = default;

void VideoRecorder::init(const std::string& filePath, unsigned int width, unsigned int height, unsigned int fps, VideoCodec codec) {
    if(initialized) throw std::runtime_error("VideoRecorder already initialized");
    if(codec == VideoCodec::RAW) throw std::runtime_error("Record raw video requires OpenCV support");
    encodedWriter = std::make_unique<EncodedVideoRecorder>();
    encodedWriter->init(filePath, width, height, fps, codec == VideoCodec::MJPEG);
    initialized = true;
}

void VideoRecorder::setH264ParameterSets(span<const std::uint8_t> sps, span<const std::uint8_t> pps) {
    if(!encodedWriter) throw std::runtime_error("VideoRecorder has no H264 writer");
    encodedWriter->setH264ParameterSets(sps, pps);
}

void VideoRecorder::write(span<uint8_t>& data, const uint32_t stride) {
    (void)stride;
    if(!initialized) throw std::runtime_error("VideoRecorder not initialized");
    encodedWriter->write(data);
}

void VideoRecorder::close() {
    if(encodedWriter) encodedWriter->close();
    encodedWriter.reset();
    initialized = false;
}

}  // namespace utility
}  // namespace dai
