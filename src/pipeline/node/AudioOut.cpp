#include "depthai/pipeline/node/AudioOut.hpp"

namespace dai {
namespace node {

AudioOut::AudioOut(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioOut, AudioOutProperties>(std::move(props)) { }

AudioOut::~AudioOut() {}

std::shared_ptr<AudioOut> AudioOut::build() {
	if (isBuild) {
	        throw std::runtime_error("AudioOut node is already built");
	}

	isBuild = true;

	return std::static_pointer_cast<AudioOut>(shared_from_this());;
}

void AudioOut::setDeviceName(std::string audioOutName) {
	properties.audioOutName = audioOutName;
}

void AudioOut::setDevicePath(std::string audioOutPath) {
	properties.audioOutPath = audioOutPath;
}

void AudioOut::setBitrate(unsigned int bitrate) {
	properties.bitrate = bitrate;
}

void AudioOut::setFps(unsigned int fps) {
	properties.framesPerSecond = fps;
}

void AudioOut::setChannels(unsigned int channels) {
	properties.channels = channels;
}

std::string AudioOut::getDeviceName() {
	return properties.audioOutName;
}

std::string AudioOut::getDevicePath() {
	return properties.audioOutPath;
}

unsigned int AudioOut::getBitrate() {
	return properties.bitrate;
}

unsigned int AudioOut::getFps() {
	return properties.framesPerSecond;
}

unsigned int AudioOut::getChannels() {
	return properties.channels;
}



}
}
