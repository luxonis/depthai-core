#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioMixerProperties.hpp"
#include "depthai/pipeline/datatype/AudioFrame.hpp"

namespace dai {
namespace node {

class AudioMixer: public DeviceNodeCRTP<DeviceNode, AudioMixer, AudioMixerProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;
   public:  // internal usage
    constexpr static const char* NAME = "AudioMixer";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioMixer() = default;
    AudioMixer(std::unique_ptr<Properties> props);

    ~AudioMixer();

    std::shared_ptr<AudioMixer> build();

    void registerSource(std::string name, float volume);
    void registerSink(std::string name, unsigned int bitrate, unsigned int channels, int format);
    void linkSourceToSink(std::string sourceName, std::string sinkName);
    void unlinkSourceFromSink(std::string sourceName, std::string sinkName);
    void unregisterSource(std::string name);
    void unregisterSink(std::string name);


    OutputMap outputs{*this, "outputs", {DEFAULT_NAME, DEFAULT_GROUP, {{{DatatypeEnum::AudioFrame, true}}}}};
    InputMap inputs{*this, "inputs", {DEFAULT_NAME, DEFAULT_GROUP, false, 1, {{{DatatypeEnum::AudioFrame, true}}}, true}};

        /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
   private:
    class AudioMixerSink;
    class AudioMixerSource;

    std::map<std::string, std::shared_ptr<AudioMixerSource>> audioSources;
    std::map<std::string, std::shared_ptr<AudioMixerSink>> audioSinks;

    class AudioMixerSource : public std::enable_shared_from_this<AudioMixerSource>  {
	    public:
	    std::set<std::shared_ptr<AudioMixerSink>> sinks;

	    float volume;

	    void sendOut(std::shared_ptr<AudioFrame> buf);
    };

    class AudioMixerSink : public std::enable_shared_from_this<AudioMixerSink>{
	    public:
	    std::map<std::shared_ptr<AudioMixerSource>,
		     std::shared_ptr<AudioFrame>> sourceData;

	    int format;
	    unsigned int bitrate;
	    unsigned int channels;

	    std::shared_ptr<AudioFrame> mix();
    };
};

}  // namespace node
}  // namespace dai
