#include "depthai/pipeline/node/AudioIn.hpp"

namespace dai {
namespace node {

AudioIn::AudioIn(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>(std::move(props)) { }

AudioIn::~AudioIn() {}

std::shared_ptr<AudioIn> AudioIn::build() {
	if (isBuild) {
	        throw std::runtime_error("AudioIn node is already built");
	}

	isBuild = true;

	return std::static_pointer_cast<AudioIn>(shared_from_this());;
}

void AudioIn::setDeviceName(std::string audioInName) {
	properties.audioInName = audioInName;
}

void AudioIn::setDevicePath(std::string audioInPath) {
	properties.audioInPath = audioInPath;
}

void AudioIn::setBitrate(unsigned int bitrate) {
	properties.bitrate = bitrate;
}

void AudioIn::setFps(unsigned int fps) {
	properties.framesPerSecond = fps;
}

void AudioIn::setChannels(unsigned int channels) {
	properties.channels = channels;
}

std::string AudioIn::getDeviceName() {
	return properties.audioInName;
}

std::string AudioIn::getDevicePath() {
	return properties.audioInPath;
}

unsigned int AudioIn::getBitrate() {
	return properties.bitrate;
}

unsigned int AudioIn::getFps() {
	return properties.framesPerSecond;
}

unsigned int AudioIn::getChannels() {
	return properties.channels;
}

}
}
