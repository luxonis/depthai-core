#include "depthai/pipeline/node/AudioMixer.hpp"
#include "depthai/utility/AudioHelpers.hpp"

#include <limits>

namespace dai {
namespace node {

AudioMixer::AudioMixer(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioMixer, AudioMixerProperties>(std::move(props)) { }

AudioMixer::~AudioMixer() {}

std::shared_ptr<AudioMixer> AudioMixer::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioMixer>(shared_from_this());;
}

void AudioMixer::registerSource(std::string name, float volume) {
	if(audioSources.find(name) == audioSources.end()) {
		audioSources[name] = std::make_shared<AudioMixerSource>();
		std::shared_ptr<AudioMixerSource> source = audioSources[name];
		source->volume = volume;
	} else {
		// Already registered
	}
}

void AudioMixer::registerSink(std::string name, int format) {
	if(audioSinks.find(name) == audioSinks.end()) {
		audioSinks[name] = std::make_shared<AudioMixerSink>();
		std::shared_ptr<AudioMixerSink> sink = audioSinks[name];
		sink->format = format;
		std::cout << "Format: " << format << std::endl;
		std::cout << "Format: " << sink->format << std::endl;
	} else {
		// Already registered
	}
}

void AudioMixer::linkSourceToSink(std::string sourceName, std::string sinkName) {
	if (audioSources.find(sourceName) != audioSources.end() &&
	    audioSinks.find(sinkName) != audioSinks.end()) {
		std::shared_ptr<AudioMixerSource> source = audioSources[sourceName];
		std::shared_ptr<AudioMixerSink> sink = audioSinks[sinkName];
		std::shared_ptr<AudioFrame> nullBuf = std::make_shared<AudioFrame>();
		source->sinks.insert(sink);
		sink->sourceData[source] = nullBuf;
	} else {
		// they dont exist
	}
}

void AudioMixer::unlinkSourceFromSink(std::string sourceName, std::string sinkName) {
	if (audioSources.find(sourceName) != audioSources.end() &&
	    audioSinks.find(sinkName) != audioSinks.end()) {
		std::shared_ptr<AudioMixerSource> source = audioSources[sourceName];
		std::shared_ptr<AudioMixerSink> sink = audioSinks[sinkName];

		source->sinks.erase(sink);
		sink->sourceData.erase(source);
	} else {
		// they dont exist
	}
}

void AudioMixer::unregisterSource(std::string name) {
	if(audioSources.find(name) != audioSources.end()) {
		audioSources.erase(name);
	} else {
		// Already unregistered
	}
}

void AudioMixer::unregisterSink(std::string name) {
	if(audioSinks.find(name) != audioSinks.end()) {
		audioSinks.erase(name);
	} else {
		// Already unregistered
	}
}

void AudioMixer::run() {
	while(isRunning()) {
		for (auto [name, source] : audioSources) {
			std::shared_ptr<AudioFrame> buf = inputs[name].get<AudioFrame>();
			source->sendOut(buf);
		}

		for (auto [name, sink] : audioSinks) {
			std::shared_ptr<AudioFrame> buf = sink->mix();
			outputs[name].send(buf);
		}
	}
}

void AudioMixer::AudioMixerSource::sendOut(std::shared_ptr<AudioFrame> buf) {
	for (auto sink : sinks) {
		sink->sourceData[this->shared_from_this()] = buf;
	}
}

std::shared_ptr<AudioFrame> AudioMixer::AudioMixerSink::mix() {
	std::shared_ptr<AudioFrame> buf = std::make_shared<AudioFrame>();
	size_t largestSize = 0;

	for (auto [source, data] : sourceData) {
		std::cout << "Size: " << data->getData().size() << std::endl;
		if(data->getData().size() > largestSize) {
			largestSize = data->getData().size();
		}
	}

	std::vector<uint8_t> mixedData(largestSize);
	std::memset(mixedData.data(), 0, largestSize);
	for (auto [source, data] : sourceData) {
		auto dataToMix = data->getData().data();
		size_t count = largestSize > data->getData().size() ?
			       data->getData().size() : largestSize;

		switch(format) {
			case SF_FORMAT_PCM_S8:
			case SF_FORMAT_PCM_16: {
				std::cout << "16 mix" << std::endl;
				short *shortDataToMix = (short*)dataToMix;
				short *shortMixedData = (short*)mixedData.data();
				const long min = std::numeric_limits<short>::min();
				const long max = std::numeric_limits<short>::max();
				count /= sizeof(short);

		        	for (size_t i = 0; i < count; ++i) {
				    long result = shortMixedData[i] + shortDataToMix[i] * source->volume;
				    std::clamp(result, min, max);
	        		    shortMixedData[i] = (short)result;
		        	}
				}
				break;
			case SF_FORMAT_PCM_24:
			case SF_FORMAT_PCM_32: {
				std::cout << "32 mix" << std::endl;
				int *intDataToMix = (int*)dataToMix;
				int *intMixedData = (int*)mixedData.data();

				const long min = std::numeric_limits<int>::min();
				const long max = std::numeric_limits<int>::max();

				count /= sizeof(int);

		        	for (size_t i = 0; i < count; ++i) {
				    long result = intMixedData[i] + intDataToMix[i] * source->volume;
				    std::clamp(result, min, max);
	        		    intMixedData[i] = (int)result;
		        	}
				}
				break;
			case SF_FORMAT_FLOAT: {
				std::cout << "float32 mix" << std::endl;
				float *floatDataToMix = (float*)dataToMix;
				float *floatMixedData = (float*)mixedData.data();

				const double min = std::numeric_limits<float>::min();
				const double max = std::numeric_limits<float>::max();

				count /= sizeof(float);

		        	for (size_t i = 0; i < count; ++i) {
				    double result = floatMixedData[i] + floatDataToMix[i] * source->volume;
				    std::clamp(result, min, max);
	        		    floatMixedData[i] = (float)result;
		        	}
				}
				break;
			case SF_FORMAT_DOUBLE: {
				std::cout << "float64 mix" << std::endl;
				double *doubleDataToMix = (double*)dataToMix;
				double *doubleMixedData = (double*)mixedData.data();

				const long double min = std::numeric_limits<double>::min();
				const long double max = std::numeric_limits<double>::max();

				count /= sizeof(double);

		        	for (size_t i = 0; i < count; ++i) {
				    long double result = doubleMixedData[i] + doubleDataToMix[i] * (double)source->volume;
				    std::clamp(result, min, max);
	        		    doubleMixedData[i] = (double)result;
		        	}
				}
				break;
			default:
				std::cout << "PANICC: " << format << std::endl;
				std::terminate();
				break;
		}
	}

	buf->setData(mixedData);
	return buf;
}

void AudioMixer::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool AudioMixer::runOnHost() const {
    return runOnHostVar;
}

}
}
