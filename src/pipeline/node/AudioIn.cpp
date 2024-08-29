#include "depthai/pipeline/node/AudioIn.hpp"
#include "depthai/utility/AudioHelpers.hpp"

#include <spdlog/async_logger.h>
#include <alsa/asoundlib.h>

#include <memory>

namespace dai {
namespace node {

AudioIn::AudioIn(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>(std::move(props)) { }

void AudioIn::run() {
	snd_pcm_hw_params_t *hwParams;

	int err;

	err = snd_pcm_open(&captureHandle, properties.audioInPath.c_str(), SND_PCM_STREAM_CAPTURE, 0);
	if(err < 0) {
		logger->warn("AudioInHost {}: Unable to open device {}", __func__, properties.audioInPath.c_str());
	}

	snd_pcm_hw_params_malloc(&hwParams);
	snd_pcm_hw_params_any(captureHandle, hwParams);

	snd_pcm_hw_params_set_access(captureHandle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(captureHandle, hwParams, properties.format);
	snd_pcm_hw_params_set_rate_near(captureHandle, hwParams, &properties.bitrate, 0);
	snd_pcm_hw_params_set_channels(captureHandle, hwParams, properties.channels);

	// Set period time based on desired packets per second
	unsigned int periodTime = 1000000 / properties.framesPerSecond; // period time in microseconds
	snd_pcm_hw_params_set_period_time_near(captureHandle, hwParams, &periodTime, 0);

	long unsigned int bufferFrames;
	snd_pcm_hw_params_get_period_size(hwParams, &bufferFrames, 0);
	size_t bufferSize = bufferFrames * snd_pcm_format_width(properties.format) / 8 * properties.channels;

	snd_pcm_hw_params(captureHandle, hwParams);
	snd_pcm_hw_params_free(hwParams);
	snd_pcm_prepare(captureHandle);
		
	std::vector<uint8_t> data;
	data.resize(bufferSize);
	while(isRunning()) {
		std::fill(data.begin(), data.end(), 0);

		err = snd_pcm_readi(captureHandle, data.data(), bufferFrames);
		if (err == -EPIPE) {
			// EPIPE means overrun
			logger->warn("AudioInHost {}: Overrun occurred", __func__);
			snd_pcm_prepare(captureHandle);
			continue;
		} else if (err < 0) {
			logger->warn("AudioInHost {}: Error from read: {}", __func__, snd_strerror(err));
			continue;
		} else if (err != (int)bufferFrames) {
			logger->warn("AudioInHost {}: Short read, read {} frames", __func__, err);
		}
		
		std::shared_ptr<AudioFrame> buf = std::make_shared<AudioFrame>(err, properties.bitrate, properties.channels, getFormat());

//			logger->warn("AudioInHost {}: read from audio interface failed ({}, {})", __func__, err, snd_strerror(err));

		buf->setData(data);
		out.send(buf);
	}

	snd_pcm_drain(captureHandle);
	snd_pcm_close(captureHandle);

	logger->info("AudioInHost {}: Audio interface closed", __func__);
}

int AudioIn::getFormat() const {
	// This cast is to avoid 'unhandled value in switch' errors
	switch((long)properties.format) {
		case SND_PCM_FORMAT_S8: return SF_FORMAT_PCM_S8;
		case SND_PCM_FORMAT_S16_LE: return SF_FORMAT_PCM_16;
		case SND_PCM_FORMAT_S24_LE: return SF_FORMAT_PCM_24;
		case SND_PCM_FORMAT_S32_LE: return SF_FORMAT_PCM_32;
		case SND_PCM_FORMAT_FLOAT_LE: return SF_FORMAT_FLOAT;
		case SND_PCM_FORMAT_FLOAT64_LE: return SF_FORMAT_DOUBLE;
		default: break;
	}

	return 0;
}

void AudioIn::setFormat(int format) {
	switch(format) {
		case SF_FORMAT_PCM_S8: properties.format = SND_PCM_FORMAT_S8; break;
		case SF_FORMAT_PCM_16: properties.format = SND_PCM_FORMAT_S16_LE; break;
		case SF_FORMAT_PCM_24: properties.format = SND_PCM_FORMAT_S24_LE; break;
		case SF_FORMAT_PCM_32: properties.format = SND_PCM_FORMAT_S32_LE; break;
		case SF_FORMAT_FLOAT: properties.format = SND_PCM_FORMAT_FLOAT_LE; break;
		case SF_FORMAT_DOUBLE: properties.format = SND_PCM_FORMAT_FLOAT64_LE; break;
		default: break;
	}
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

std::string AudioIn::getDeviceName() const {
	return properties.audioInName;
}

std::string AudioIn::getDevicePath() const {
	return properties.audioInPath;
}

unsigned int AudioIn::getBitrate() const {
	return properties.bitrate;
}

unsigned int AudioIn::getFps() const {
	return properties.framesPerSecond;
}

unsigned int AudioIn::getChannels() const {
	return properties.channels;
}

void AudioIn::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool AudioIn::runOnHost() const {
    return runOnHostVar;
}
}
}
