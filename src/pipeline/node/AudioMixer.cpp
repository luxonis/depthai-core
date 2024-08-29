#include "depthai/pipeline/node/AudioMixer.hpp"
#include "depthai/utility/AudioHelpers.hpp"

#include <limits>

namespace dai {
namespace node {

AudioMixer::AudioMixer(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioMixer, AudioMixerProperties>(std::move(props)) { }

void AudioMixer::registerSource(std::string name, float volume) {
	if(audioSources.find(name) == audioSources.end()) {
		audioSources[name] = std::make_shared<AudioMixerSource>();
		std::shared_ptr<AudioMixerSource> src = audioSources[name];
		src->volume = volume;
		src->bufferReady = false;
		src->thread = std::thread([this, &name] {
			std::shared_ptr<AudioMixerSource> source = audioSources[name];
			while(!isReady()) {
			}

			while(isRunning()) {
				std::shared_ptr<AudioFrame> frame = inputs[name].get<AudioFrame>();
				if (frame == nullptr) {
					continue;
				}

				{
					std::lock_guard lck(source->currentBufferMtx);
					source->currentBuf = frame;
					source->bufferReady = true;
					//std::cout << "Set current buf" << std::endl;
				}

				//std::cout << "Src call all threads" << std::endl;
				source->notifyBufferChange.notify_all();
			}
		});
	} else {
		// Already registered
	}
}

void AudioMixer::registerSink(std::string name, unsigned int bitrate, unsigned int channels, int format) {
	if(audioSinks.find(name) == audioSinks.end()) {
		audioSinks[name] = std::make_shared<AudioMixerSink>();
		std::shared_ptr<AudioMixerSink> snk = audioSinks[name];
		snk->format = format;
		snk->bitrate = bitrate;
		snk->channels = channels;
		snk->thread = std::thread([this, &name] {
			std::shared_ptr<AudioMixerSink> sink = audioSinks[name];
			while(!isReady()) {
			}

			while(isRunning()) {
				std::shared_ptr<AudioFrame> buf = sink->mix();
				//std::cout << "Send buf" << std::endl;
				outputs[name].send(buf);
			}
		});
	} else {
		// Already registered
	}
}

void AudioMixer::linkSourceToSink(std::string sourceName, std::string sinkName) {
	if (audioSources.find(sourceName) != audioSources.end() &&
	    audioSinks.find(sinkName) != audioSinks.end()) {
		std::shared_ptr<AudioMixerSource> src = audioSources[sourceName];
		std::shared_ptr<AudioMixerSink> snk = audioSinks[sinkName];
		src->sinks[snk] = std::thread([this, &sourceName, &sinkName] {
			std::shared_ptr<AudioMixerSource> source = audioSources[sourceName];
			std::shared_ptr<AudioMixerSink> sink = audioSinks[sinkName];
			sink->sourceData[source] = std::make_shared<AudioMixerSink::sourceData_t>();
			auto data = sink->sourceData[source];
			data->framePresent = false;

			while(!isReady()) {
			}

			while(isRunning()) {
				std::unique_lock lck(source->currentBufferMtx);
				source->notifyBufferChange.wait(lck, [&source]{ return source->bufferReady; });
				source->bufferReady = false;

				//std::cout << "Lock mutex link" << std::endl;

				{
					std::lock_guard dataLck(data->mtx);
					//std::cout << "Set link data" << std::endl;
					if(source->currentBuf != nullptr) {
						data->frames.push_back(source->currentBuf);
						data->framePresent = true;
					}
				}
				data->frameCv.notify_all();

				//std::cout << "Unlock mutex link" << std::endl;
				lck.unlock();
				source->notifyBufferChange.notify_all();
			}
		});
	} else {
		// they dont exist
	}
}

void AudioMixer::unlinkSourceFromSink(std::string sourceName, std::string sinkName) {
	if (audioSources.find(sourceName) != audioSources.end() &&
	    audioSinks.find(sinkName) != audioSinks.end()) {
		std::shared_ptr<AudioMixerSource> src = audioSources[sourceName];
		std::shared_ptr<AudioMixerSink> snk = audioSinks[sinkName];

		src->sinks.erase(snk);
		snk->sourceData.erase(src);
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
		properties.ready = true;

		while(isRunning()) { }
	}
}

std::shared_ptr<AudioFrame> AudioMixer::AudioMixerSink::mix() {
	std::shared_ptr<AudioFrame> buf = std::make_shared<AudioFrame>(0, bitrate, channels, format);
	size_t largestSize = 0;

	for (auto [source, data] : sourceData) {
		bool isMissing = true;
		while(isMissing) {
			std::unique_lock lck(data->mtx);
			data->frameCv.wait(lck, [&data]{ return data->framePresent; });

			if(data->frames.front() == nullptr) {
				//std::cout << "Missing..." << std::endl;
				isMissing = true;
			} else {
				//std::cout << "Not missing..." << std::endl;
				isMissing = false;
			}
		}

		std::lock_guard dataLck(data->mtx);

		//std::cout << "Size: " << data->frames.front()->getData().size() << std::endl;
		if(data->frames.front()->getData().size() > largestSize) {
			largestSize = data->frames.front()->getData().size();
		}
	}

	std::vector<uint8_t> mixedData(largestSize);

	size_t frames = largestSize * bitrate;
	buf->setFrames(frames);

	std::memset(mixedData.data(), 0, largestSize);
	for (auto [source, data] : sourceData) {
		if(data->frames.front()->getBitrate() != bitrate ||
		   data->frames.front()->getFormat() != format ||
		   data->frames.front()->getChannels() != channels) {
			throw std::runtime_error("Data contained different bitrate/format/channel data compared to sink");
		}

		auto dataToMix = data->frames.front()->getData().data();
		size_t count = largestSize > data->frames.front()->getData().size() ?
			       data->frames.front()->getData().size() : largestSize;

		switch(format) {
			case SF_FORMAT_PCM_S8:
			case SF_FORMAT_PCM_16: {
				//std::cout << "16 mix" << std::endl;
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
				//std::cout << "32 mix" << std::endl;
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
				//std::cout << "float32 mix" << std::endl;
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
				//std::cout << "float64 mix" << std::endl;
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
				throw std::runtime_error("Invalid format in mixer");
				break;
		}

		std::unique_lock lck(data->mtx);
		data->frames.pop_front();
		if(data->frames.front() == nullptr) {
			data->framePresent = false;
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
