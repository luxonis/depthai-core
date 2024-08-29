#include "depthai/pipeline/node/AudioOut.hpp"

#include <alsa/asoundlib.h>
#include <spdlog/async_logger.h>

#include <memory>

#include "depthai/utility/AudioHelpers.hpp"

namespace dai {
namespace node {

AudioOut::AudioOut(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioOut, AudioOutProperties>(std::move(props)) {}

void AudioOut::run() {
    snd_pcm_hw_params_t* hwParams;

    int err;

    err = snd_pcm_open(&captureHandle, properties.audioOutPath.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if(err < 0) {
        logger->warn("AudioOutHost {}: Unable to open device {}", __func__, properties.audioOutPath.c_str());
    }

    snd_pcm_hw_params_malloc(&hwParams);
    snd_pcm_hw_params_any(captureHandle, hwParams);

    snd_pcm_hw_params_set_access(captureHandle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(captureHandle, hwParams, properties.format);
    snd_pcm_hw_params_set_rate_near(captureHandle, hwParams, &properties.bitrate, 0);
    snd_pcm_hw_params_set_channels(captureHandle, hwParams, properties.channels);

    // Set period time based on desired packets per second
    unsigned int periodTime = 1000000 / properties.framesPerSecond;  // period time in microseconds
    snd_pcm_hw_params_set_period_time_near(captureHandle, hwParams, &periodTime, 0);

    long unsigned int bufferFrames;
    snd_pcm_hw_params_get_period_size(hwParams, &bufferFrames, 0);
    size_t bufferSize = bufferFrames * snd_pcm_format_width(properties.format) / 8 * properties.channels;
    // std::cout << "AudioFrame frames " << bufferFrames << std::endl;
    // std::cout << "AudioFrame size " << bufferSize << std::endl;

    snd_pcm_hw_params(captureHandle, hwParams);
    snd_pcm_hw_params_free(hwParams);
    snd_pcm_prepare(captureHandle);

    std::vector<uint8_t> data;
    data.resize(bufferSize);
    while(isRunning()) {
        std::shared_ptr<AudioFrame> recvBuf = input.get<AudioFrame>();
        // Check if recv has same audio metadata as output

        const auto& recvData = recvBuf->getData();
        size_t recvSize = recvData.size();
        size_t writeSize = recvSize;

        if(recvSize == 0) {
            // std::cout << "Empty packet received, waiting for next one" << std::endl;
            continue;
        }

        if(recvSize > bufferSize) {
            /* TODO: Manage multiple cycles */
            writeSize = bufferSize;
            logger->warn("AudioOutHost {}: Received data larger than buffer, truncating to buffer size", __func__);

        } else if(recvSize < bufferSize) {
            std::fill(data.begin() + recvSize, data.end(), 0);
        }

        // std::cout << "AudioFrame size: " << bufferSize << " Write size: " << writeSize << " Recv size: " << recvSize << std::endl;

        std::memcpy(data.data(), recvData.data(), writeSize);
        // std::cout << "Memcpy" << std::endl;

        err = snd_pcm_writei(captureHandle, data.data(), bufferFrames);
        // std::cout << "Write" << std::endl;
        if(err == -EPIPE) {
            // EPIPE means overrun
            logger->warn("AudioOutHost {}: Underrun occurred", __func__);
            snd_pcm_prepare(captureHandle);
        } else if(err < 0) {
            logger->warn("AudioOutHost {}: Error from write: {}", __func__, snd_strerror(err));
        } else if(err != (int)bufferFrames) {
            logger->warn("AudioOutHost {}: Short write, wrote {} frames", __func__, err);
        }

        //			logger->warn("AudioOutHost {}: read from audio interface failed ({}, {})", __func__, err, snd_strerror(err));
    }

    snd_pcm_drain(captureHandle);
    snd_pcm_close(captureHandle);

    logger->info("AudioOutHost {}: Audio interface closed", __func__);
}

int AudioOut::getFormat() const {
    // This cast is to avoid 'unhandled value in switch' errors
    switch((long)properties.format) {
        case SND_PCM_FORMAT_S8:
            return SF_FORMAT_PCM_S8;
        case SND_PCM_FORMAT_S16_LE:
            return SF_FORMAT_PCM_16;
        case SND_PCM_FORMAT_S24_LE:
            return SF_FORMAT_PCM_24;
        case SND_PCM_FORMAT_S32_LE:
            return SF_FORMAT_PCM_32;
        case SND_PCM_FORMAT_FLOAT_LE:
            return SF_FORMAT_FLOAT;
        case SND_PCM_FORMAT_FLOAT64_LE:
            return SF_FORMAT_DOUBLE;
        default:
            break;
    }

    return 0;
}

void AudioOut::setFormat(int format) {
    switch(format) {
        case SF_FORMAT_PCM_S8:
            properties.format = SND_PCM_FORMAT_S8;
            break;
        case SF_FORMAT_PCM_16:
            properties.format = SND_PCM_FORMAT_S16_LE;
            break;
        case SF_FORMAT_PCM_24:
            properties.format = SND_PCM_FORMAT_S24_LE;
            break;
        case SF_FORMAT_PCM_32:
            properties.format = SND_PCM_FORMAT_S32_LE;
            break;
        case SF_FORMAT_FLOAT:
            properties.format = SND_PCM_FORMAT_FLOAT_LE;
            break;
        case SF_FORMAT_DOUBLE:
            properties.format = SND_PCM_FORMAT_FLOAT64_LE;
            break;
        default:
            break;
    }
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

std::string AudioOut::getDeviceName() const {
    return properties.audioOutName;
}

std::string AudioOut::getDevicePath() const {
    return properties.audioOutPath;
}

unsigned int AudioOut::getBitrate() const {
    return properties.bitrate;
}

unsigned int AudioOut::getFps() const {
    return properties.framesPerSecond;
}

unsigned int AudioOut::getChannels() const {
    return properties.channels;
}

void AudioOut::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool AudioOut::runOnHost() const {
    return runOnHostVar;
}

}  // namespace node
}  // namespace dai
