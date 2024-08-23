#include "depthai/pipeline/node/AudioEncoder.hpp"
#include "depthai/utility/AudioHelpers.hpp"

namespace dai {
namespace node {

AudioEncoder::AudioEncoder(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioEncoder, AudioEncoderProperties>(std::move(props)) { }
    
AudioEncoder::~AudioEncoder() {}

std::shared_ptr<AudioEncoder> AudioEncoder::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioEncoder>(shared_from_this());;
}

void AudioEncoder::run() {
	while(isRunning()) {
		std::shared_ptr<AudioFrame> recvBuf = input.get<AudioFrame>();
			
		std::shared_ptr<AudioFrame> sendBuf = std::make_shared<AudioFrame>(0, properties.bitrate, properties.channels, properties.format);

		std::vector<float> floatData = convertToFloat(recvBuf);

		if(recvBuf->getChannels() != sendBuf->getChannels()) {
	            throw std::runtime_error("Different channels counts not supported");
			// TODO: deal with channels
		}

		if(recvBuf->getBitrate() != sendBuf->getBitrate()) {
			floatData = resampleAudio(floatData, recvBuf->getBitrate(), properties.bitrate, properties.channels);
		}

		sendBuf = convertFromFloat(floatData, sendBuf);

		out.send(sendBuf);
	}
}

std::vector<float> AudioEncoder::convertToFloat(std::shared_ptr<AudioFrame> inputFrame) {
    uint8_t *inputData = inputFrame->getData().data();
    std::vector<float> floatData;

    // Determine how to convert based on the input format
    size_t frameCount = inputFrame->getFrames();

    floatData.resize(frameCount * inputFrame->getChannels());

    switch (inputFrame->getFormat()) {
        case SF_FORMAT_PCM_S8:
            for (size_t i = 0; i < frameCount * inputFrame->getChannels(); ++i) {
                floatData[i] = inputData[i] / 128.0f; // Convert S8 to float [-1.0, 1.0]
            }
            break;
        case SF_FORMAT_PCM_16:
            for (size_t i = 0; i < frameCount * inputFrame->getChannels(); ++i) {
                int16_t sample = (inputData[i * 2 + 1] << 8) | inputData[i * 2];
                floatData[i] = sample / 32768.0f; // Convert S16 to float [-1.0, 1.0]
            }
            break;
        case SF_FORMAT_PCM_24:
            for (size_t i = 0; i < frameCount * inputFrame->getChannels(); ++i) {
                int32_t sample = (inputData[i * 3 + 2] << 16) | (inputData[i * 3 + 1] << 8) | inputData[i * 3];
                floatData[i] = sample / 8388608.0f; // Convert S24 to float [-1.0, 1.0]
            }
            break;
        case SF_FORMAT_PCM_32:
            for (size_t i = 0; i < frameCount * inputFrame->getChannels(); ++i) {
                int32_t sample = (inputData[i * 4 + 3] << 24) | (inputData[i * 4 + 2] << 16) | (inputData[i * 4 + 1] << 8) | inputData[i * 4];
                floatData[i] = sample / 2147483648.0f; // Convert S32 to float [-1.0, 1.0]
            }
            break;
        default:
            throw std::runtime_error("Unsupported format");
    }

    return floatData;
}

std::vector<float> AudioEncoder::resampleAudio(std::vector<float> inputData, int inputSampleRate, int outputSampleRate, int channels) {
    SRC_DATA srcData;
    srcData.data_in = inputData.data();
    srcData.input_frames = inputData.size() / channels;
    srcData.data_out = nullptr;
    srcData.output_frames = static_cast<long>(inputData.size() / channels * outputSampleRate / inputSampleRate);
    srcData.src_ratio = static_cast<double>(outputSampleRate) / inputSampleRate;
    srcData.end_of_input = 0;

    std::vector<float> outputData(srcData.output_frames * channels);
    srcData.data_out = outputData.data();

    int error = src_simple(&srcData, SRC_SINC_BEST_QUALITY, channels);
    if (error) {
        throw std::runtime_error(src_strerror(error));
    }

    return outputData;
}

std::shared_ptr<AudioFrame> AudioEncoder::convertFromFloat(std::vector<float> inputData, std::shared_ptr<AudioFrame> outputFrame) {
    std::vector<uint8_t> outputData;

    size_t frameCount = inputData.size() / outputFrame->getChannels();
    size_t outputSize = frameCount * outputFrame->getChannels();

    outputFrame->setFrames(frameCount);

	switch(outputFrame->getFormat()) {
		case SF_FORMAT_PCM_S8:
		case SF_FORMAT_PCM_16:
			outputSize *= (16 / 8);
			break;
		case SF_FORMAT_PCM_24:
		case SF_FORMAT_PCM_32:
		case SF_FORMAT_FLOAT:
			outputSize *= (32 / 8);
			break;
		case SF_FORMAT_DOUBLE:
			outputSize *= (64/ 8);
			break;

	} 

    outputData.resize(outputSize);

    switch (outputFrame->getFormat()) {
        case SF_FORMAT_PCM_S8:
            for (size_t i = 0; i < frameCount * outputFrame->getChannels(); ++i) {
                outputData[i] = static_cast<uint8_t>(std::clamp(inputData[i] * 128.0f, -128.0f, 127.0f));
            }
            break;
        case SF_FORMAT_PCM_16:
            for (size_t i = 0; i < frameCount * outputFrame->getChannels(); ++i) {
                int16_t sample = static_cast<int16_t>(std::clamp(inputData[i] * 32768.0f, -32768.0f, 32767.0f));
                outputData[i * 2] = sample & 0xFF;
                outputData[i * 2 + 1] = (sample >> 8) & 0xFF;
            }
            break;
        case SF_FORMAT_PCM_24:
            for (size_t i = 0; i < frameCount * outputFrame->getChannels(); ++i) {
                int32_t sample = static_cast<int32_t>(std::clamp(inputData[i] * 8388608.0f, -8388608.0f, 8388607.0f));
                outputData[i * 3] = sample & 0xFF;
                outputData[i * 3 + 1] = (sample >> 8) & 0xFF;
                outputData[i * 3 + 2] = (sample >> 16) & 0xFF;
            }
            break;
        case SF_FORMAT_PCM_32:
            for (size_t i = 0; i < frameCount * outputFrame->getChannels(); ++i) {
                int32_t sample = static_cast<int32_t>(std::clamp(inputData[i] * 2147483648.0f, -2147483648.0f, 2147483647.0f));
                outputData[i * 4] = sample & 0xFF;
                outputData[i * 4 + 1] = (sample >> 8) & 0xFF;
                outputData[i * 4 + 2] = (sample >> 16) & 0xFF;
                outputData[i * 4 + 3] = (sample >> 24) & 0xFF;
            }
            break;
        default:
            throw std::runtime_error("Unsupported format");
    }

    outputFrame->setData(outputData);

    return outputFrame;
}

unsigned int AudioEncoder::getBitrate() const {
	return properties.bitrate;
}

unsigned int AudioEncoder::getChannels() const {
	return properties.channels;
}

int AudioEncoder::getFormat() const {
	return properties.format;
}

void AudioEncoder::setBitrate(unsigned int bitrate) {
	properties.bitrate = bitrate;
}

void AudioEncoder::setChannels(unsigned int channels) {
	properties.channels = channels;
}

void AudioEncoder::setFormat(int format) {
	properties.format = format;
}

void AudioEncoder::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool AudioEncoder::runOnHost() const {
    return runOnHostVar;
}

}
}
