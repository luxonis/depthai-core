#include "depthai/pipeline/datatype/AudioFrame.hpp"

namespace dai {

AudioFrame::AudioFrame(sf_count_t frames, unsigned int bitrate, unsigned int channels, int format) :
	frames(frames), bitrate(bitrate), channels(channels), format(format) {}

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
