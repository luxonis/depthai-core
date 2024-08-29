#include "depthai/pipeline/datatype/AudioFrame.hpp"

#include "depthai/utility/SharedMemory.hpp"

namespace dai {

AudioFrame::AudioFrame() {
    setTimestamp(std::chrono::steady_clock::now());
}

AudioFrame::AudioFrame(size_t size) : AudioFrame() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}

AudioFrame::AudioFrame(long fd) : AudioFrame() {
    auto mem = std::make_shared<SharedMemory>(fd);
    data = mem;
}

AudioFrame::AudioFrame(long fd, size_t size) : AudioFrame() {
    auto mem = std::make_shared<SharedMemory>(fd, size);
    data = mem;
}

AudioFrame::AudioFrame(sf_count_t frames, unsigned int bitrate, unsigned int channels, int format) : AudioFrame() {
    auto mem = std::make_shared<VectorMemory>();
    data = mem;

    this->frames = frames;
    this->bitrate = bitrate;
    this->channels = channels;
    this->format = format;
}

void AudioFrame::setFrames(sf_count_t frames) {
    this->frames = frames;
}

void AudioFrame::setBitrate(unsigned int bitrate) {
    this->bitrate = bitrate;
}

void AudioFrame::setChannels(unsigned int channels) {
    this->channels = channels;
}

void AudioFrame::setFormat(int format) {
    this->format = format;
}

sf_count_t AudioFrame::getFrames() const {
    return frames;
}

unsigned int AudioFrame::getBitrate() const {
    return bitrate;
}

unsigned int AudioFrame::getChannels() const {
    return channels;
}

int AudioFrame::getFormat() const {
    return format;
}

}  // namespace dai
